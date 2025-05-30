//
// Triangulation helper class to wrap CGAL Traingulation library.
//
#include "lxsdk/lxresult.h"
#include "lxsdk/lxvmath.h"
#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_value.hpp>
#include <lxsdk/lxu_math.hpp>
#include <lxsdk/lxu_matrix.hpp>
#include <lxsdk/lxu_quaternion.hpp>
#include <lxsdk/lxu_geometry_triangulation.hpp>

#include <vector>
#include <map>
#include <iostream>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

#include <CGAL/mark_domain_in_triangulation.h>

#include "triangulate_helper.hpp"


//
// Constrained Delaunay Triangulations: force edges into Delaunay triangulation
//
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
typedef CGAL::Delaunay_mesher_2<CDT, Criteria> Mesher;

typedef CDT::Vertex_handle Vertex_handle;
typedef CDT::Face_handle Face_handle;
typedef CDT::Point CPoint;

LxResult TriangulateHelper::ConstraintDelaunay(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& tris)
{
    std::cout << "** ConstraintDelaunay **" << std::endl;
    CLxUser_Point point, point1;
    point.fromMesh(m_mesh);
    point1.fromMesh(m_mesh);

    LXtVector norm;
    polygon.Normal(norm);

    CDT cdt;

    std::vector<Vertex_handle> vertex_handles;

    // Set axis plane to compute the triangulation on 2D space.
    AxisPlane axisPlane(norm);

    std::vector<LXtPointID> source;
    std::unordered_map<LXtPointID,unsigned> indices;

    MeshUtil::MakeVertexTable(polygon, point, source, indices);

    double   z_ave = 0.0;
    for (auto i = 0u; i < source.size(); i++)
    {
        point.Select(source[i]);
        LXtFVector pos;
        point.Pos(pos);
        double x, y, z;
        axisPlane.ToPlane(pos, x, y, z);
        vertex_handles.push_back(cdt.insert(CPoint(x, y)));
        z_ave += z;
    }

    // averaged z value on axis plane
    z_ave /= static_cast<double>(source.size());

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
        if (MeshUtil::IsKeyholeBridge(point, point1))
        {
            continue;
        }
        int v1 = indices[vrt];
        int v2 = indices[vrt1];
        cdt.insert_constraint(vertex_handles[v1], vertex_handles[v2]);
    }

    if (!cdt.is_valid())
    {
        s_log.DebugOut(LXi_DBLOG_ERROR, "Invalid CDT context");
        return LXe_FAILED;
    }
    
    std::unordered_map<Face_handle, bool> in_domain_map;
    boost::associative_property_map< std::unordered_map<Face_handle,bool> > in_domain(in_domain_map);
 
    // Mark facets that are inside the domain bounded by the polygon
    CGAL::mark_domain_in_triangulation(cdt, in_domain);

    LXtID4       type = polygon.Type(&type);
    LXtPolygonID polyID;
    LXtPointID   vert[3];

    tris.clear();
    std::cout << "vertices (" <<  cdt.number_of_vertices() << ") triangles (" << cdt.number_of_faces() << ")" << std::endl;

    // Make a map to get index from vertex handle.
    std::unordered_map<Vertex_handle,int> vertex_to_index;
    int index = 0;
    for (auto vertex = cdt.finite_vertices_begin(); vertex != cdt.finite_vertices_end(); vertex++)
    {
        vertex_to_index[vertex] = index++;
    }
    assert(static_cast<size_t>(index) == source.size());

    // Make triangle face polygons into the edit mesh.
    for (auto face = cdt.finite_faces_begin(); face != cdt.finite_faces_end(); face++)
    {
        // Skip if the face is not in domain.
        if (!get(in_domain,face))
            continue;

        // Get three vertices of the triangle
        for (auto i = 0; i < 3; i++)
        {
            Vertex_handle vh = face->vertex(i);
            index = vertex_to_index[vh];
            vert[i] = source[index];
        }
        // Make a new triangle polygon
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

    CDT cdt;

    std::vector<Vertex_handle> vertex_handles;

    PolygonPlane polyPlane(m_mesh, polygon);

    std::vector<LXtPointID> source;
    std::unordered_map<LXtPointID,unsigned> indices;

    MeshUtil::MakeVertexTable(polygon, point, source, indices);

    double   z_ave = 0.0;
    for (auto i = 0u; i < source.size(); i++)
    {
        point.Select(source[i]);
        LXtFVector pos;
        point.Pos(pos);
        double x, y, z;
        polyPlane.ToPlane(pos, x, y, z);
        //printf("[%u] input pos %f %f %f x %f y %f z %f\n", i, pos[0], pos[1], pos[2], x, y, z);
        vertex_handles.push_back(cdt.insert(CPoint(x, y)));
        z_ave += z;
    }

    // averaged z value on axis plane
    z_ave /= static_cast<double>(source.size());

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
        if (MeshUtil::IsKeyholeBridge(point, point1))
        {
            continue;
        }
        int v1 = indices[vrt];
        int v2 = indices[vrt1];
        cdt.insert_constraint(vertex_handles[v1], vertex_handles[v2]);
    }

    // 
    //  angle_min = std::asin (std::sqrt(B));
    //
    //  B = std::sin (angle_min);
    //  B = B * B;
    //
    Mesher mesher(cdt);
    double B = std::sin (m_angle_min);
    mesher.set_criteria(Criteria(B * B, m_edge_size));

    // Generate triangles
    mesher.refine_mesh();
    
    std::unordered_map<Face_handle, bool> in_domain_map;
    boost::associative_property_map< std::unordered_map<Face_handle,bool> > in_domain(in_domain_map);
 
    // Mark facets that are inside the domain bounded by the polygon
    CGAL::mark_domain_in_triangulation(cdt, in_domain);

    LXtID4       type = polygon.Type(&type);
    LXtPolygonID polyID;
    LXtPointID   vert[3];

    tris.clear();
    std::cout << "vertices (" <<  cdt.number_of_vertices() << ") triangles (" << cdt.number_of_faces() << ")" << std::endl;
    m_poledit.SetSearchPolygon(polygon.ID(), true);

    AxisTriangles axisTriangles(m_mesh, polygon);
    std::vector<double> weights;

    // Make new additional vertices. 
    std::vector<LXtPointID> vertices;
    std::unordered_map<Vertex_handle,int> vertex_to_index;
    int index = 0;
    for (auto vertex = cdt.finite_vertices_begin(); vertex != cdt.finite_vertices_end(); vertex++)
    {
        // Make a map to get index from vertex handle.
        vertex_to_index[vertex] = index++;
        // Use the original vertex if it is not new vertex.
        if (index <= source.size())
        {
            vertices.push_back(source[index - 1]);
            continue;
        }
        LXtPointID vrt;
        LXtVector  pos;
        auto& p = vertex->point();

        polyPlane.FromPlane(pos, p.x(), p.y(), z_ave);
        //printf("[%u] output pos %f %f %f x %f y %f z %f\n", index, pos[0], pos[1], pos[2], p.x(), p.y(), z_ave);
        // This interpolates vertex map values at the new position on the source polygon.
        axisTriangles.MakePositionWeights(pos, weights);
        m_poledit.AddFaceVertex(pos, polygon.ID(), weights.data(), &vrt);
        vertices.push_back(vrt);
    }

    // Make triangle face polygons into the edit mesh.
    for (auto face = cdt.finite_faces_begin(); face != cdt.finite_faces_end(); face++)
    {
        // Skip if the face is not in domain.
        if (!get(in_domain,face))
            continue;

        // Get three vertices of the triangle
        for (auto i = 0; i < 3; i++)
        {
            Vertex_handle vh = face->vertex(i);
            index = vertex_to_index[vh];
            vert[i] = vertices[index];
        }
        // Make a new triangle polygon
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

class DiscoMapVisitor : public CLxImpl_AbstractVisitor
{
public:
    CLxUser_MeshMap            m_maps;
    std::vector<LXtMeshMapID>& m_disco_maps;

    DiscoMapVisitor(CLxUser_Mesh& mesh, std::vector<LXtMeshMapID>& disco_maps) : m_disco_maps(disco_maps)
    {
        mesh.GetMaps(m_maps);
        m_disco_maps.clear();
    }

    LxResult Evaluate() override
    {
        if ((m_maps.IsContinuous() == LXe_FALSE) && (m_maps.IsEdgeMap() == LXe_FALSE))
        {
            m_disco_maps.push_back(m_maps.ID());
        }
        return LXe_OK;
    }
};

void TriangulateHelper::CopyDiscoValues(CLxUser_Polygon& polygon, std::vector<LXtPolygonID>& tris)
{
    std::vector<LXtMeshMapID> disco_maps;
    DiscoMapVisitor visMap(m_mesh, disco_maps);
    visMap.m_maps.Enum(&visMap);

    CLxUser_Polygon triangle;
    triangle.fromMesh(m_mesh);

    for (auto map : disco_maps)
    {
        unsigned dim;
        visMap.m_maps.Select(map);
        visMap.m_maps.Dimension(&dim);
        auto nvert = 0u, nvert1 = 0u;
        polygon.VertexCount(&nvert);
        float* value = new float[dim];
        for (auto i = 0u; i < nvert; i++)
        {
            LXtPointID vrt;
            polygon.VertexByIndex(i, &vrt);
            if (polygon.MapValue(map, vrt, value) == LXe_FALSE)
            {
                continue;
            }
            for (auto tri : tris)
            {
                triangle.Select(tri);
                triangle.VertexCount(&nvert1);
                for (auto j = 0u; j < nvert1; j++)
                {
                    LXtPointID vrt1;
                    triangle.VertexByIndex(j, &vrt1);
                    if (vrt1 == vrt)
                    {
                        triangle.SetMapValue(vrt1, map, value);
                    }
                }
            }
        }
        delete[] value;
    }
}
