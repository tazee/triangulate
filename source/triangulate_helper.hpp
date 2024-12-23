//
// Triangulation helper class to wrap CDT library.
//
#pragma once

#include <lxsdk/lx_log.hpp>
#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_value.hpp>
#include <lxsdk/lxu_math.hpp>
#include "lxsdk/lxvmath.h"
#include <lxsdk/lxu_matrix.hpp>
#include <lxsdk/lxu_quaternion.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include <vector>

enum TriangulateType : int
{
    ConstraintDelaunay = 0,
    ConformingDelaunay = 1,
};

struct TriangulateHelper
{
    void SetMesh(CLxUser_Mesh& edit_mesh, CLxUser_Mesh& base_mesh)
    {
        m_mesh.set(edit_mesh);
        s_mesh.CreatePolygonEdit(m_poledit);
        m_poledit.SetMesh(edit_mesh, base_mesh);
    }

    //
    // Constrained Delaunay Triangulations: force edges into Delaunay
    // triangulation
    //
    LxResult ConstraintDelaunay(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& tris);
    //
    // Conforming Delaunay Triangulations: add new points into Delaunay
    // triangulation until the edge is present in triangulation
    //
    LxResult ConformingDelaunay(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& tris);

    //
    // Ear Clipping Triangulation: simple and fast method for simple polygons.
    //
    LxResult EarClipping(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& tris);

    CLxUser_Mesh        m_mesh;
    CLxUser_PolygonEdit m_poledit;
    CLxUser_LogService  s_log;
    CLxUser_MeshService s_mesh;
};


struct AxisPlane
{
    AxisPlane(const LXtVector vec)
    {
        static const unsigned axis0[] = { 1, 2, 0 };
        static const unsigned axis1[] = { 2, 0, 1 };

        m_axis = MaxExtent(vec);
        m_ix   = axis0[m_axis];
        m_iy   = axis1[m_axis];

        LXtVector norm;
        LXx_VUNIT(norm, m_axis);
        lx::MatrixIdent(m_m);
        if (VectorEqual(vec, norm) == false)
        {
            VectorRotation(m_m, vec, norm);
        }
        lx::MatrixCopy(m_mInv, m_m);
        lx::MatrixTranspose(m_mInv);
    }

    void ToPlane(const LXtFVector pos, double& x, double& y, double& z)
    {
        LXtFVector r;
        lx::MatrixMultiply(r, m_m, pos);
        x = r[m_ix];
        y = r[m_iy];
        z = r[m_axis];
    }

    void FromPlane(LXtVector pos, double x, double y, double z)
    {
        LXtVector r;
        r[m_ix]   = x;
        r[m_iy]   = y;
        r[m_axis] = z;
        lx::MatrixMultiply(pos, m_mInv, r);
    }

    unsigned MaxExtent(const LXtVector v)
    {
        double a = std::abs(v[0]);
        double b = std::abs(v[1]);
        double c = std::abs(v[2]);
        if (a > b && a > c)
            return 0;
        else if (b >= a && b > c)
            return 1;
        else
            return 2;
    }

    bool VectorEqual(const LXtVector a, const LXtVector b)
    {
        for (auto i = 0u; i < LXdND; i++)
        {
            if (lx::Compare(a[i], b[i]))
                return false;
        }
        return true;
    }

    double AngleVectors(const LXtVector v0, const LXtVector v1)
    {
        double vlen0, vlen1, x;

        vlen0 = LXx_VLEN(v0);
        vlen1 = LXx_VLEN(v1);
        if (vlen0 < lx::Tolerance(vlen0) || vlen1 < lx::Tolerance(vlen1))
            return 0.0;
        x = LXx_VDOT(v0, v1) / vlen0 / vlen1;
        x = LXxCLAMP(x, -1.0, 1.0);
        return std::acos(x);
    }

    void VectorRotation(LXtMatrix m, const LXtVector v0, const LXtVector v1)
    {
        LXtVector vo;
        double    qq[4];

        if (VectorEqual(v0, v1))
        {
            lx::MatrixIdent(m);
            return;
        }

        LXx_VCROSS(vo, v1, v0);
        double theta = AngleVectors(v1, v0);
        if (std::abs(theta) < lx::Tolerance(theta))
        {
            lx::MatrixIdent(m);
            return;
        }
        if (!lx::VectorNormalize(vo))
        {
            qq[0] = 0.0;
            qq[1] = std::sin(theta / 2);
            qq[2] = 0.0;
            qq[3] = std::cos(theta / 2);
        }
        else
        {
            double sint = sin(theta / 2);
            qq[0]       = sint * vo[0];
            qq[1]       = sint * vo[1];
            qq[2]       = sint * vo[2];
            qq[3]       = std::cos(theta / 2);
        }
        CLxQuaternion quat(qq);
        quat.normalize();
        CLxMatrix4 m4 = quat.asMatrix();
        m4.getMatrix3x3(m);
    }

    unsigned  m_axis, m_ix, m_iy;
    LXtMatrix m_m, m_mInv;
};


class CVisitor : public CLxImpl_AbstractVisitor
{
public:
    CLxUser_Point      m_vert;
    CLxUser_Polygon    m_poly;
    CLxUser_Mesh       base_mesh, edit_mesh;
    CLxUser_LogService s_log;
    TriangulateHelper  triHelp;
    int                m_triType;
    bool               succeeded;

    CVisitor()
    {
        succeeded = true;
    }

    bool IsTwisted()
    {
        using namespace boost::geometry;
        typedef model::d2::point_xy<double> Point;
        typedef model::segment<Point> Segment;

        struct VertexPos
        {
            Point pnt;
            LXtPointID vrt;
            LXtFVector pos;
        };
    
        LXtVector norm;
        m_poly.Normal(norm);

        AxisPlane axisPlane(norm);
    
        unsigned nvert;
        m_poly.VertexCount(&nvert);
        std::vector<VertexPos> points;
    
        // Cache the vertex positions and points on the axis plane.
        for (auto i = 0u; i < nvert; i++)
        {
            VertexPos vp;
            m_poly.VertexByIndex(i, &vp.vrt);
            m_vert.Select(vp.vrt);
            m_vert.Pos(vp.pos);
            double x, y, z;
            axisPlane.ToPlane(vp.pos, x, y, z);
            vp.pnt = Point(x, y);
            points.push_back(vp);
        }

        // Check the polygon is convex or not. If the polygon is convex,
        // the polygon is not twisted.
        bool convex = true;
        for (auto i = 0u; i < nvert; i++)
        {
            auto j = (i + 1) % nvert;
            auto k = (i - 1 + nvert) % nvert;
    
            LXtVector a, b, c;

            LXx_VSUB3 (a, points[k].pos, points[i].pos);
            LXx_VSUB3 (b, points[i].pos, points[j].pos);
            LXx_VCROSS (c, a, b);
            if (LXx_VDOT (c, norm) < 0.0)
            {
                convex = false;
                break;
            }
        }
        s_log.DebugOut(LXi_DBLOG_NORMAL, "IsTwisted converx = %d", convex);
        if (convex)
            return false;
        
        // Check the polygon is twisted or not by checking the intersection
        for (auto i = 0u; i < nvert; i++)
        {
            auto j = (i + 1) % nvert;
            Segment s1(points[i].pnt, points[j].pnt);
            for (auto k = 0u; k < nvert; k++)
            {
                auto l = (k + 1) % nvert;
                if (i == k || i == l || j == k || j == l)
                    continue;
                if (points[i].vrt == points[k].vrt || points[i].vrt == points[l].vrt ||
                    points[j].vrt == points[k].vrt || points[j].vrt == points[l].vrt)
                    continue;
                Segment s2(points[k].pnt, points[l].pnt);
                if (intersects(s1, s2))
                {
                    s_log.DebugOut(LXi_DBLOG_NORMAL, "polygon ID %p is twisted", m_poly.ID());
                    return true;
                }
            }
        }
        return false;
    }

    LxResult Evaluate()
    {
        unsigned nvert;
        m_poly.VertexCount(&nvert);
        s_log.DebugOut(LXi_DBLOG_NORMAL, "polygon ID %p nvert = %u", m_poly.ID(), nvert);
        if (nvert < 3)
            return LXe_OK;

        LXtID4 type;
        m_poly.Type(&type);
        if ((type != LXiPTYP_FACE) && (type != LXiPTYP_PSUB) && (type != LXiPTYP_SUBD))
            return LXe_OK;

        std::vector<LXtPolygonID> tris;
        LxResult result;
    
        if (IsTwisted())
            result = triHelp.EarClipping(m_poly, tris);
        else if (m_triType == ConstraintDelaunay)
            result = triHelp.ConstraintDelaunay(m_poly, tris);
        else if (m_triType == ConformingDelaunay)
            result = triHelp.ConformingDelaunay(m_poly, tris);
        
        // check the result
        if (result == LXe_OK)
            m_poly.Remove();
        else
            succeeded = false;
        return LXe_OK;
    }
};
