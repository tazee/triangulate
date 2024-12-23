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
#include <lxsdk/lxu_geometry_triangulation.hpp>

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
    printf("nvert = %u\n", nvert);
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
// Modo default triangulation: simple and fast method for simple polygons
// using GenerateTriangles () method of CLxUser_Polygon.
//
LxResult TriangulateHelper::ModoTriangulation1(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& tris)
{
    std::cout << "** ModoTriangulation1 **" << std::endl;

    LXtID4       type = polygon.Type(&type);
    LXtPolygonID polyID;
    LXtPointID   vert[3];
    CLxUser_Point point;
    point.fromMesh(m_mesh);

    tris.clear();

    // Make new triangle polygons using vertices of source polygon.
    unsigned nvert;
    polygon.GenerateTriangles(&nvert);
    for (auto i = 0u; i < nvert; i++)
    {
        polygon.TriangleByIndex(i, &vert[0], &vert[1], &vert[2]);
        polygon.NewProto(type, vert, 3, 0, &polyID);
        tris.push_back(polyID);
    }

    return LXe_OK;
}

//
// Modo default triangulation: simple and fast method for simple polygons
// using TriangulateFace () method in lxu_geometry_triangulation.hpp.
//
LxResult TriangulateHelper::ModoTriangulation2(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& tris)
{
    std::cout << "** ModoTriangulation2 **" << std::endl;

    LXtID4       type = polygon.Type(&type);
    LXtPolygonID polyID;
    LXtPointID   vert[3];
    CLxUser_Point point;
    point.fromMesh(m_mesh);

    tris.clear();

    // Make new triangle polygons using vertices of source polygon.
    std::vector<lx::GeoTriangle> geoTris = lx::TriangulateFace (polygon, point);
    for (auto i = 0u; i < geoTris.size(); i++)
    {
        polygon.VertexByIndex(geoTris[i].v0, &vert[0]);
        polygon.VertexByIndex(geoTris[i].v1, &vert[1]);
        polygon.VertexByIndex(geoTris[i].v2, &vert[2]);
        polygon.NewProto(type, vert, 3, 0, &polyID);
        tris.push_back(polyID);
    }

    return LXe_OK;
}

//
// Triangulate quadrangles.
//
LxResult TriangulateHelper::Quadrangles(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& tris, int method)
{
    std::cout << "** Quadrangles method = " << method << std::endl;

    LXtID4       type = polygon.Type(&type);
    LXtPolygonID polyID;
    LXtPointID   A[3], B[3], vert[4];
    LXtFVector   pos[4];
    CLxUser_Point point;
    point.fromMesh(m_mesh);

    tris.clear();

    polygon.VertexByIndex(0, &vert[0]);
    polygon.VertexByIndex(1, &vert[1]);
    polygon.VertexByIndex(2, &vert[2]);
    polygon.VertexByIndex(3, &vert[3]);

    for (auto i = 0u; i < 4; i++)
    {
        point.Select(vert[i]);
        point.Pos(pos[i]);
    }

    double d1, d2;

    switch (method)
    {
    case ShortestDiagonal:
        d1 = LXx_VDIST(pos[0], pos[2]);
        d2 = LXx_VDIST(pos[1], pos[3]);
        if (d1 < d2)
        {
            A[0] = vert[0];
            A[1] = vert[1];
            A[2] = vert[2];
            B[0] = vert[0];
            B[1] = vert[2];
            B[2] = vert[3];
        }
        else
        {
            A[0] = vert[0];
            A[1] = vert[1];
            A[2] = vert[3];
            B[0] = vert[1];
            B[1] = vert[2];
            B[2] = vert[3];
        }
        break;
    case LongestDiagonal:
        d1 = LXx_VDIST(pos[0], pos[2]);
        d2 = LXx_VDIST(pos[1], pos[3]);
        if (d1 > d2)
        {
            A[0] = vert[0];
            A[1] = vert[1];
            A[2] = vert[2];
            B[0] = vert[0];
            B[1] = vert[2];
            B[2] = vert[3];
        }
        else
        {
            A[0] = vert[0];
            A[1] = vert[1];
            A[2] = vert[3];
            B[0] = vert[1];
            B[1] = vert[2];
            B[2] = vert[3];
        }
        break;
    case Split_2_4:
        A[0] = vert[0];
        A[1] = vert[1];
        A[2] = vert[3];
        B[0] = vert[1];
        B[1] = vert[2];
        B[2] = vert[3];
        break;
    default:
        A[0] = vert[0];
        A[1] = vert[1];
        A[2] = vert[2];
        B[0] = vert[0];
        B[1] = vert[2];
        B[2] = vert[3];
        break;
    }

    polygon.NewProto(type, A, 3, 0, &polyID);
    tris.push_back(polyID);

    polygon.NewProto(type, B, 3, 0, &polyID);
    tris.push_back(polyID);

    return LXe_OK;
}

