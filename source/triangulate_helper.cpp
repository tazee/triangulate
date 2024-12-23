//
// Triangulation helper class to wrap CDT library.
//
#include "lxsdk/lxresult.h"
#include "lxsdk/lxvmath.h"
#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_value.hpp>
#include <lxsdk/lxu_math.hpp>
#include <lxsdk/lxu_matrix.hpp>
#include <lxsdk/lxu_quaternion.hpp>

#include <CDT.h>
#include <VerifyTopology.h>
#include <vector>
#include <map>
#include <iostream>

#include "triangulate_helper.hpp"

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
// Constrained Delaunay Triangulations: force edges into Delaunay triangulation
//
LxResult TriangulateHelper::ConstraintDelaunay(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& tris)
{
    std::cout << "** ConstraintDelaunay **" << std::endl;
    CLxUser_Point point, point1;
    point.fromMesh(m_mesh);
    point1.fromMesh(m_mesh);

    double tol = PolygonTolerance(polygon, point);

    LXtVector norm;
    polygon.Normal(norm);

    CDT::Triangulation cdt =
    CDT::Triangulation<double>(CDT::VertexInsertionOrder::Enum::Auto, CDT::IntersectingConstraintEdges::Enum::TryResolve, tol);

    std::vector<CDT::V2d<double>> points;
    std::vector<CDT::Edge>        edges;

    // Set axis plane to compute the triangulation on 2D space.
    AxisPlane axisPlane(norm);

    std::vector<LXtPointID> vertices;
    std::unordered_map<LXtPointID,unsigned> indices;

    MakeVertexTable(polygon, point, vertices, indices);

    for (auto i = 0u; i < vertices.size(); i++)
    {
        point.Select(vertices[i]);
        LXtFVector pos;
        point.Pos(pos);
        double x, y, z;
        axisPlane.ToPlane(pos, x, y, z);
        points.push_back(CDT::V2d<double>::make(x, y));
    }

    // Set vertex projected positions and edge links.
    unsigned nvert;
    polygon.VertexCount(&nvert);
    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID vrt, vrt1;
        polygon.VertexByIndex(i, &vrt);
        polygon.VertexByIndex((i + 1) % nvert, &vrt1);
        point.Select(vrt);
        point1.Select(vrt1);
        if (IsKeyholeBridge(point, point1))
        {
            continue;
        }
        CDT::VertInd v1 = indices[vrt];
        CDT::VertInd v2 = indices[vrt1];
        edges.push_back(CDT::Edge(v1, v2));
    }

    // Compute constrained delaunay triangulations
    cdt.insertVertices(points);
    cdt.insertEdges(edges);
    cdt.eraseOuterTrianglesAndHoles();
    if (CDT::verifyTopology(cdt) == false)
        return LXe_FAILED;

    LXtID4       type = polygon.Type(&type);
    LXtPolygonID polyID;
    LXtPointID   vert[3];

    tris.clear();

    // Make new triangle polygons using vertices of source polygon.
    for (auto i = 0u; i < cdt.triangles.size(); i++)
    {
        vert[0] = vertices[cdt.triangles[i].vertices[0]];
        vert[1] = vertices[cdt.triangles[i].vertices[1]];
        vert[2] = vertices[cdt.triangles[i].vertices[2]];
        polygon.NewProto(type, vert, 3, 0, &polyID);
        tris.push_back(polyID);
    }

    return LXe_OK;
}

//
// Conforming Delaunay Triangulations: add new points into Delaunay triangulation until the edge is
// present in triangulation
//
LxResult TriangulateHelper::ConformingDelaunay(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& tris)
{
    std::cout << "** ConformingDelaunay **" << std::endl;
    CLxUser_Point point, point1;
    point.fromMesh(m_mesh);
    point1.fromMesh(m_mesh);

    double tol = PolygonTolerance(polygon, point);

    LXtVector norm;
    polygon.Normal(norm);

    CDT::Triangulation cdt =
    CDT::Triangulation<double>(CDT::VertexInsertionOrder::Enum::Auto, CDT::IntersectingConstraintEdges::Enum::TryResolve, tol);

    std::vector<CDT::V2d<double>> points;
    std::vector<CDT::Edge>        edges;

    // Set axis plane to compute the triangulation on 2D space.
    AxisPlane axisPlane(norm);

    std::vector<LXtPointID> vertices;
    std::unordered_map<LXtPointID,unsigned> indices;

    MakeVertexTable(polygon, point, vertices, indices);

    double   z_ave = 0.0;
    for (auto i = 0u; i < vertices.size(); i++)
    {
        point.Select(vertices[i]);
        LXtFVector pos;
        point.Pos(pos);
        double x, y, z;
        axisPlane.ToPlane(pos, x, y, z);
        points.push_back(CDT::V2d<double>::make(x, y));
        z_ave += z;
    }

    // averaged z value on axis plane
    z_ave /= static_cast<double>(vertices.size());

    // Set vertex projected positions and edge links.
    unsigned nvert;
    polygon.VertexCount(&nvert);
    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID vrt, vrt1;
        polygon.VertexByIndex(i, &vrt);
        polygon.VertexByIndex((i + 1) % nvert, &vrt1);
        point.Select(vrt);
        point1.Select(vrt1);
        if (IsKeyholeBridge(point, point1))
        {
            continue;
        }
        CDT::VertInd v1 = indices[vrt];
        CDT::VertInd v2 = indices[vrt1];
        edges.push_back(CDT::Edge(v1, v2));
    }

    // Compute conforming delaunay triangulations
    cdt.insertVertices(points);
    cdt.conformToEdges(edges);
    cdt.eraseOuterTrianglesAndHoles();
    if (CDT::verifyTopology(cdt) == false)
        return LXe_FAILED;

    LXtID4       type = polygon.Type(&type);
    LXtPolygonID polyID;
    LXtPointID   vert[3];

    tris.clear();
    std::cout << "vertices (" <<  cdt.vertices.size() << ") triangles (" << cdt.triangles.size() << ")" << std::endl;
    m_poledit.SetSearchPolygon(polygon.ID(), true);

    // Make new additional vertices
    for (auto i = vertices.size(); i < cdt.vertices.size(); i++)
    {
        LXtPointID              vrt;
        LXtVector               pos;
        const CDT::V2d<double>& v = cdt.vertices[i];
        axisPlane.FromPlane(pos, v.x, v.y, z_ave);
        m_poledit.AddFaceVertex(pos, polygon.ID(), nullptr, &vrt);
        vertices.push_back(vrt);
    }

    // Make new triangle polygons using vertices of source polygon and new vertices.
    for (auto i = 0u; i < cdt.triangles.size(); i++)
    {
        vert[0] = vertices[cdt.triangles[i].vertices[0]];
        vert[1] = vertices[cdt.triangles[i].vertices[1]];
        vert[2] = vertices[cdt.triangles[i].vertices[2]];
        polygon.NewProto(type, vert, 3, 0, &polyID);
        tris.push_back(polyID);
    }

    return LXe_OK;
}

//
// Ear Clipping Triangulation: simple and fast method for simple polygons.
//
LxResult TriangulateHelper::EarClipping(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& tris)
{
    std::cout << "** EarClipping **" << std::endl;

    LXtID4       type = polygon.Type(&type);
    LXtPolygonID polyID;
    LXtPointID   vert[3];

    tris.clear();

    // Make new triangle polygons using vertices of source polygon.
    unsigned count;
    polygon.GenerateTriangles(&count);
    std::cout << "EarClipping count = " <<  count << std::endl;
    for (auto i = 0u; i < count; i++)
    {
        polygon.TriangleByIndex(i, &vert[0], &vert[1], &vert[2]);
        polygon.NewProto(type, vert, 3, 0, &polyID);
        tris.push_back(polyID);
    }

    return LXe_OK;
}

