//
// Utility functions for the mesh processing.
//
#pragma once

#include <lxsdk/lx_log.hpp>
#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_value.hpp>
#include <lxsdk/lxu_math.hpp>
#include <lxsdk/lxvmath.h>
#include <lxsdk/lxu_matrix.hpp>
#include <lxsdk/lxu_quaternion.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include <vector>
#include <unordered_set>
#include <tuple>

//
// Basic vector math functions.
//
namespace MathUtil {

    static unsigned MaxExtent(const LXtVector v)
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

    static bool VectorEqual(const LXtVector a, const LXtVector b)
    {
        for (auto i = 0u; i < LXdND; i++)
        {
            if (lx::Compare(a[i], b[i]))
                return false;
        }
        return true;
    }

    static double AngleVectors(const LXtVector v0, const LXtVector v1)
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

    static void VectorRotation(LXtMatrix m, const LXtVector v0, const LXtVector v1)
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

    static bool CrossNormal (LXtVector norm, const LXtVector a1, const LXtVector a2, const LXtVector a3)
    {
        LXtVector a, b;

        LXx_VSUB3 (a, a1, a2);
        LXx_VSUB3 (b, a2, a3);
        LXx_VCROSS (norm, a, b);
        return lx::VectorNormalize (norm);
    }

    static bool CrossNormal (LXtFVector norm, const LXtFVector a1, const LXtFVector a2, const LXtFVector a3)
    {
        LXtFVector a, b;

        LXx_VSUB3 (a, a1, a2);
        LXx_VSUB3 (b, a2, a3);
        LXx_VCROSS (norm, a, b);
        return lx::VectorNormalize (norm);
    }
};


//
// Axis plane class to convert the 3D position to 2D position.
//
struct AxisPlane
{
    AxisPlane()
    {
        m_axis = 0;
        m_ix   = 1;
        m_iy   = 2;
        lx::MatrixIdent(m_m);
        lx::MatrixIdent(m_mInv);
    }

    AxisPlane(const LXtVector vec)
    {
        static const unsigned axis0[] = { 1, 2, 0 };
        static const unsigned axis1[] = { 2, 0, 1 };

        m_axis = MathUtil::MaxExtent(vec);
        m_ix   = axis0[m_axis];
        m_iy   = axis1[m_axis];

        printf("AxisPlane vec %f %f %f axis %u %u %u\n", vec[0], vec[1], vec[2], m_axis, m_ix, m_iy);
        LXtVector norm;
        LXx_VUNIT(norm, m_axis);
        lx::MatrixIdent(m_m);
        if (MathUtil::VectorEqual(vec, norm) == false)
        {
            MathUtil::VectorRotation(m_m, vec, norm);
        }
        lx::MatrixCopy(m_mInv, m_m);
        lx::MatrixTranspose(m_mInv);
        printf("matrix %f %f %f %f %f %f %f %f %f\n",
            m_m[0][0], m_m[0][1], m_m[0][2], m_m[1][0], m_m[1][1], m_m[1][2],m_m[2][0], m_m[2][1], m_m[2][2]);
    }

    void ToPlane(const LXtVector pos, double& x, double& y, double& z)
    {
        LXtVector r;
        lx::MatrixMultiply(r, m_m, pos);
        x = r[m_ix];
        y = r[m_iy];
        z = r[m_axis];
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

    unsigned  m_axis, m_ix, m_iy;
    LXtMatrix m_m, m_mInv;
};


//
// Compute weights at the given position on polygon.
//
struct AxisTriangles
{

    AxisTriangles(CLxUser_Mesh& mesh, CLxUser_Polygon& poly)
    {
        CLxUser_Point point;
        point.fromMesh(mesh);

        LXtVector norm;
        poly.Normal(norm);
        m_axisPlane = AxisPlane(norm);

        std::unordered_map<LXtPointID,unsigned> indices;
        unsigned nvert;
        poly.VertexCount(&nvert);
        for (auto i = 0u; i < nvert; i++)
        {
            LXtPointID vrt;
            poly.VertexByIndex(i, &vrt);
            indices.insert(std::make_pair(vrt, i));
            LXtFVector pos;
            point.Select(vrt);
            point.Pos(pos);
            m_points.push_back(CLxVector(pos));
        }

        unsigned ntri;
        poly.GenerateTriangles(&ntri);
        for (auto i = 0u; i < ntri; i++)
        {
            LXtPointID v0, v1, v2;
            poly.TriangleByIndex(i, &v0, &v1, &v2);
            auto it0 = indices.find(v0);
            auto it1 = indices.find(v1);
            auto it2 = indices.find(v2);
            m_triangles.push_back(std::make_tuple(it0->second, it1->second, it2->second));
        }
    }

    void BaryCentric(CLxVector p0, CLxVector p1, CLxVector p2, double u, double v, double& w0, double& w1, double& w2)
    {
        CLxVector U = p1 - p0;
        CLxVector V = p2 - p0;
        CLxVector T = p2 - p1;
    
        U.normalize();
        V.normalize();
        T.normalize();
    
        double c1 = T.dot(U);
        double s1 = std::sqrt(1.0 - c1 * c1);
    
        T *= -1.0;
        double c2 = T.dot(U);
        double s2 = std::sqrt(1.0 - c2 * c2);
    
        double dt = 0.0;
        double ds = u;
        double dd = std::sqrt(1.0 - ds) * (s1 / s2);
        if (dd)
            dt = v / dd;
    
        w0 = (1.0 - ds) * (1.0 - dt);
        w1 = ds;
        w2 = (1.0 - ds) * dt;
    }

    bool InTriangle(CLxVector pos, CLxVector p0, CLxVector p1, CLxVector p2, double& u, double& v)
    {
        CLxVector vec1(p1 - p0);
        CLxVector vec2(p2 - p0);

        double b[2], c[2], p[2];

        b[0] = vec1[m_axisPlane.m_ix];
        b[1] = vec1[m_axisPlane.m_iy];
        c[0] = vec2[m_axisPlane.m_ix];
        c[1] = vec2[m_axisPlane.m_iy];
        p[0] = pos[m_axisPlane.m_ix] - p0[m_axisPlane.m_ix];
        p[1] = pos[m_axisPlane.m_iy] - p0[m_axisPlane.m_iy];

        u = (p[1] * c[0] - p[0] * c[1]) / (b[1] * c[0] - b[0] * c[1]);
	    v = (p[1] * b[0] - p[0] * b[1]) / (c[1] * b[0] - c[0] * b[1]);
    
        return (u >= 0.0) && (v >= 0.0) && ((u + v) <= 1.0);
    }

    double PointLineSegment (CLxVector& p, CLxVector& x0, CLxVector& x1, double* dist)
    {
        double t, d;

        CLxVector x = CLxVector(x1 - x0);
        CLxVector a = CLxVector(p - x0);
        d = x.dot(x);
        if (!d)
            t = 0.0;
        else
            t = a.dot(x) / d;

        if (dist) {
            if (t < 0.0) {
                *dist = a.length();
            } else if (t > 1.0) {
                a = p - x1;
                *dist = a.length();
            } else {
                x *= t;
                a -= x;
                *dist = a.length();
            }
        }
        return t;
    }

    void MakePositionWeights(const LXtVector pos, std::vector<double>& weights)
    {
        CLxVector p = CLxVector(pos);

        weights.resize(m_points.size(), 0.0);
        for (auto i = 0u; i < weights.size(); i++)
            weights[i] = 0.0;

        for (auto i = 0u; i < m_points.size(); i++)
        {
            auto& p0 = m_points[i];
            if (p0 == p)
            {
                weights[i] = 1.0;
                return;
            }
        }

        for (auto i = 0u; i < m_points.size(); i++)
        {
            auto j = (i + 1) % m_points.size();
            auto& p0 = m_points[i];
            auto& p1 = m_points[j];
            double dist;
            double t = PointLineSegment(p, p0, p1, &dist);
            if ((t >= 0.0) && (t <= 1.0) && (std::abs(dist) <= lx::Tolerance(0.0)))
            {
                weights[j] = t;
                weights[i] = 1.0 - t;
                return;
            }
        }

        for (auto i = 0u; i < m_triangles.size(); i++)
        {
            auto& t  = m_triangles[i];
            auto  i0 = std::get<0>(t);
            auto  i1 = std::get<1>(t);
            auto  i2 = std::get<2>(t);
            auto& p0 = m_points[i0];
            auto& p1 = m_points[i1];
            auto& p2 = m_points[i2];

            double u, v;

            if (InTriangle(p, p0, p1, p2, u, v))
            {
                BaryCentric(p0, p1, p2, u, v, weights[i0], weights[i1], weights[i2]);
                return;
            }
        }
    }

    AxisPlane m_axisPlane;
    std::vector<CLxVector> m_points;
    std::vector<std::tuple<unsigned,unsigned,unsigned>> m_triangles;
};


//
// Polygon and vertex utility functions.
//
namespace MeshUtil {
//
// Get maximum tolerance of the given polygon.
//
static double PolygonTolerance(CLxUser_Polygon& polygon, CLxUser_Point& point)
{
    double m = 0.0;

    unsigned nvert;
    polygon.VertexCount(&nvert);
    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID vrt;
        LXtFVector pos;
        polygon.VertexByIndex(i, &vrt);
        point.Select(vrt);
        point.Pos(pos);
        for (auto j = 0u; j < LXdND; j++)
        {
            double z = std::abs(pos[j]);
            if (z > m)
                m = z;
        }
    }
    return lx::Tolerance(m);
}

//
// Return true if the vertex pair is appeared twice.
//
static bool IsKeyholeBridge(CLxUser_Point& point, CLxUser_Point& point1)
{
    CLxUser_MeshService s_mesh;
    LXtMarkMode mark_dupl = s_mesh.SetMode(LXsMARK_USER_0);

    if (point.TestMarks(mark_dupl) == LXe_FALSE)
        return false;
    if (point1.TestMarks(mark_dupl) == LXe_FALSE)
        return false;
    return true;
}


//
// Make vertex index table of polygon.
//
static void MakeVertexTable(CLxUser_Polygon& polygon, CLxUser_Point& point, std::vector<LXtPointID>& vertices, std::unordered_map<LXtPointID,unsigned>& indices)
{
    CLxUser_MeshService s_mesh;

    LXtMarkMode mark_dupl;
    mark_dupl = s_mesh.ClearMode(LXsMARK_USER_0);

    unsigned nvert;
    polygon.VertexCount(&nvert);
    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID vrt;
        polygon.VertexByIndex(i, &vrt);
        point.Select(vrt);
        point.SetMarks(mark_dupl);
    }

    mark_dupl = s_mesh.SetMode(LXsMARK_USER_0);

    unsigned n = 0;
    polygon.VertexCount(&nvert);
    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID vrt;
        polygon.VertexByIndex(i, &vrt);
        if (indices.find(vrt) == indices.end())
        {
            indices.insert(std::make_pair(vrt, n ++));
            vertices.push_back(vrt);
        }
        else
        {
            point.Select(vrt);
            point.SetMarks(mark_dupl);
        }
    }
}


//
// Compute the area of the loop on axis plane.
//
static double LoopAreaSize(CLxUser_Mesh& mesh, AxisPlane& axisPlane, std::vector<LXtPointID>& loop)
{
    if (loop.size() < 3)
        return 0.0;

    CLxUser_Point point;
    point.fromMesh(mesh);

    LXtFVector pos;
    double x0, y0, z0, x1, y1, z1;
    point.Select(loop[0]);
    point.Pos(pos);
    axisPlane.ToPlane(pos, x0, y0, z0);
    point.Select(loop[1]);
    point.Pos(pos);
    axisPlane.ToPlane(pos, x1, y1, z1);
    double ax, ay, bx, by;
    double S = 0.0;
    ax = x1 - x0;
    ay = y1 - y0;
    for (auto i = 2u; i < loop.size(); i++)
    {
        point.Select(loop[i]);
        point.Pos(pos);
        axisPlane.ToPlane(pos, x1, y1, z1);
        bx = x1 - x0;
        by = y1 - y0;
		S += (ax * by - ay * bx) * 0.5;
		ax = bx;
		ay = by;
    }
    return std::abs(S);
}

//
// Make boundary vertex list and loops from the given polygon.
//
static bool MakeBoundaryVertexList(CLxUser_Mesh& mesh, CLxUser_Polygon& polygon, std::vector<LXtPointID>& vertices, std::vector<std::vector<LXtPointID>>& loops)
{
    CLxUser_Point point0, point1;
    point0.fromMesh(mesh);
    point1.fromMesh(mesh);

    vertices.clear();
    loops.clear();

    std::unordered_map<LXtPointID,unsigned> indices;
    MakeVertexTable(polygon, point0, vertices, indices);

    bool hasHole = false;

    std::vector<std::pair<LXtPointID,LXtPointID>> edges;

    unsigned nvert;
    polygon.VertexCount(&nvert);

    for (auto i = 0u; i < nvert; i++)
    {
        auto j = nvert - i - 1;
        LXtPointID vrt0, vrt1;
        polygon.VertexByIndex(j, &vrt0);
        polygon.VertexByIndex((j + 1) % nvert, &vrt1);
        point0.Select(vrt0);
        point1.Select(vrt1);
        if (IsKeyholeBridge(point0, point1))
        {
            hasHole = true;
        }
        else
        {
            edges.push_back(std::make_pair(vrt0, vrt1));
        }
    }

    if (!hasHole)
    {
        return false;
    }

    // split the polygon into loops
    std::vector<LXtPointID> loop;
    std::pair<LXtPointID,LXtPointID> edge;

    while (edges.size() > 0)
    {
        if (loop.empty())
        {
            edge = edges.back();
            edges.pop_back();
            loop.push_back(edge.first);
        }

        bool found = false;
        for(auto it = edges.begin(); it != edges.end(); it++)
        {
            if (it->first == edge.second)
            {
                loop.push_back(it->first);
                edge = *it;
                if (it->second == loop[0])
                {
                    loops.push_back(loop);
                    loop.clear();
                }
                edges.erase(it);
                found = true;
                break;
            }
        }
        if (!found)
        {
            return false;
        }
    }

    // find the outer loop
    LXtVector norm;
    polygon.Normal(norm);

    AxisPlane axisPlane(norm);

    double max_area = LoopAreaSize (mesh, axisPlane, loops[0]);
    unsigned index = 0;

    for (auto i = 1u; i < loops.size(); i++)
    {
        double area = LoopAreaSize(mesh, axisPlane, loops[i]);
        if (area > max_area)
        {
            max_area = area;
            index = i;
        }
    }

    vertices = loops[index];
    loops.erase(loops.begin() + index);

    return true;
}

//
// Return true if the polygon is opened.
//
static bool PolygonIsOpened (CLxUser_Mesh& mesh, CLxUser_Polygon& polygon)
{
    unsigned nvert;
    polygon.VertexCount(&nvert);
    if (nvert < 3)
        return true;

    CLxUser_Edge edge;
    edge.fromMesh(mesh);

    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID A, B;
        polygon.VertexByIndex(i, &A);
        polygon.VertexByIndex((i + 1) % nvert, &B);
        edge.SelectEndpoints(A, B);
        unsigned npol;
        edge.PolygonCount(&npol);
        if (npol != 1)
            return false;
    }

    return true;
}

static bool PolygonMedianCenter (CLxUser_Mesh& mesh, CLxUser_Polygon& polygon, LXtVector center)
{
    unsigned nvert;

    LXx_VCLR(center);
    CLxUser_Point point;
    point.fromMesh(mesh);

    polygon.VertexCount(&nvert);
    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID vrt;
        polygon.VertexByIndex(i, &vrt);
        point.Select(vrt);
        LXtFVector pos;
        point.Pos(pos);
        LXx_VADD(center, pos);
    }

    if (nvert > 0)
    {
        double scale = 1.0 / static_cast<double>(nvert);
        LXx_VSCL(center, scale);
        return true;
    }
    else
        return false;
}

static bool PolygonPlaneMatrix (CLxUser_Mesh& mesh, CLxUser_Polygon& polygon, LXtMatrix m, LXtMatrix m_Inv)
{
    LXtVector norm, vec;
    polygon.Normal(norm);

    LXx_VUNIT(vec, 2);
    MathUtil::VectorRotation(m, vec, norm);
    if (m_Inv != nullptr)
    {
        lx::MatrixCopy(m_Inv, m);
        lx::MatrixTranspose(m_Inv);
    }
    return true;
}

}; // MeshUtil

//
// Polygon plane class to convert the 3D position to 2D position.
//
struct PolygonPlane
{
    PolygonPlane(CLxUser_Mesh& mesh, CLxUser_Polygon& polygon)
    {
        LXtVector vec;
        polygon.Normal(m_norm);

        LXx_VUNIT(vec, 2);
        MathUtil::VectorRotation(m_m, vec, m_norm);
        lx::MatrixCopy(m_mInv, m_m);
        lx::MatrixTranspose(m_mInv);
        MeshUtil::PolygonMedianCenter(mesh, polygon, m_center);
    }

    void ToPlane(const LXtFVector pos, double& x, double& y, double& z)
    {
        LXtVector a, b;
        LXx_VSUB3(a, pos, m_center);
        lx::MatrixMultiply(b, m_m, a);
        x = b[0];
        y = b[1];
        z = b[2];
    }

    void ToPlane(const LXtVector pos, double& x, double& y, double& z)
    {
        LXtVector a, b;
        LXx_VSUB3(a, pos, m_center);
        lx::MatrixMultiply(b, m_m, a);
        x = b[0];
        y = b[1];
        z = b[2];
    }

    void FromPlane(LXtFVector pos, double x, double y, double z)
    {
        LXtVector a, b;
        a[0] = x;
        a[1] = y;
        a[2] = z;
        lx::MatrixMultiply(b, m_mInv, a);
        LXx_VADD3(pos, b, m_center);
    }

    void FromPlane(LXtVector pos, double x, double y, double z)
    {
        LXtVector a, b;
        a[0] = x;
        a[1] = y;
        a[2] = z;
        lx::MatrixMultiply(b, m_mInv, a);
        LXx_VADD3(pos, b, m_center);
    }

    LXtMatrix m_m, m_mInv;
    LXtVector m_norm, m_center;
};

