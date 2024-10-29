//
// Triangulation helper class to wrap CDT library.
//
#pragma once

#include <lxsdk/lx_log.hpp>
#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_value.hpp>
#include <lxsdk/lxu_math.hpp>

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

    CLxUser_Mesh        m_mesh;
    CLxUser_PolygonEdit m_poledit;
    CLxUser_LogService  s_log;
    CLxUser_MeshService s_mesh;
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
    
        if (m_triType == ConstraintDelaunay)
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
