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
#include <unordered_set>

#include "util.hpp"

enum TriangulateType : int
{
    ConstraintDelaunay = 0,
    ConformingDelaunay = 1,
};

enum QuadMethod : int
{
    Split_1_3 = 0,
    Split_2_4 = 1,
    ShortestDiagonal = 2,
    LongestDiagonal = 3,
};

enum MinimumEdgeSize : int
{
    ByRatio = 0,
    ByLength = 1,
};

struct TriangulateHelper
{
    TriangulateHelper()
    {
        m_angle_min = 20.7 * LXx_DEG2RAD;
        m_edge_size = 0.5;
    }
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
    // Modo default triangulation: simple and fast method for simple polygons
    // using GenerateTriangles () method of CLxUser_Polygon.
    //
    LxResult ModoTriangulation1(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& tris);

    //
    // Modo default triangulation: simple and fast method for simple polygons
    // using TriangulateFace () method in lxu_geometry_triangulation.hpp.
    //
    LxResult ModoTriangulation2(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& tris);

    //
    // Triangulate quadrangles.
    //
    LxResult Quadrangles(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& tris, int method = ShortestDiagonal);

    CLxUser_Mesh        m_mesh;
    CLxUser_PolygonEdit m_poledit;
    CLxUser_LogService  s_log;
    CLxUser_MeshService s_mesh;

    double m_angle_min;     // Minimum angle of triangle
    double m_edge_size;     // Maximum edge length of triangle
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
    int                m_quad;
    int                m_edge;  // edge size
    double             m_edge_scale;   // edge related scale
    double             m_edge_size;    // edge maximum size
    double             m_angle;        // angle
    bool               succeeded;

    std::vector<LXtPolygonID> tris;     // result triangles

    CVisitor()
    {
        succeeded = true;
    }

    bool IsValid()
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
        if (convex)
            return true;

        // Concave quad polygon should be triangulated by Quadrangles method.
        if (nvert == 4)
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
                    return false;
                }
            }
        }
        return true;
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

        LxResult result;
    
        // the polygon is valid or not.
        bool is_valid = IsValid();
    
        triHelp.m_angle_min = m_angle;
        if (m_edge == MinimumEdgeSize::ByRatio)
        {
            double area;
            m_poly.Area(&area);
            triHelp.m_edge_size = m_edge_scale * std::sqrt(area);
        }
        else
            triHelp.m_edge_size = m_edge_size;
    
        if (!is_valid)
            result = triHelp.ModoTriangulation2(m_poly, tris);
        else if (m_triType == ConformingDelaunay)
            result = triHelp.ConformingDelaunay(m_poly, tris);
        else if ((nvert == 4) && is_valid)
            result = triHelp.Quadrangles(m_poly, tris, m_quad);
        else if (m_triType == ConstraintDelaunay)
            result = triHelp.ConstraintDelaunay(m_poly, tris);
        
        // check the result
        if (result == LXe_OK)
            m_poly.Remove();
        else
            succeeded = false;
        return LXe_OK;
    }
};
