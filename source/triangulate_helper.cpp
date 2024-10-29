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
