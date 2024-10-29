//
// The procedural mesh modifier version of CDT Traingulation named "mesh.cdt.item"
//

#include "triangulate_meshop.hpp"

LXxC_MeshOperation_Evaluate(CMeshOp)  // (ILxUnknownID mesh, LXtID4 type, LXtMarkMode mode)
{
    CVisitor          vis;

    vis.base_mesh.set(mesh);
    vis.edit_mesh.set(mesh);

    if (!vis.edit_mesh.test() || type != LXiSEL_POLYGON)
        return LXe_OK;

    attr_GetInt(ARGi_MESHOP_TYPE, &vis.m_triType);

    vis.triHelp.SetMesh(vis.edit_mesh, vis.base_mesh);

    check(vis.m_vert.fromMesh(vis.edit_mesh));
    check(vis.m_poly.fromMesh(vis.edit_mesh));

    // Triangulate selected polygons using CDT traingulation method.
    check(vis.m_poly.Enum(&vis, select_mode));

    vis.edit_mesh.SetMeshEdits(LXf_MESHEDIT_GEOMETRY);

    return LXe_OK;
}

LXtTagInfoDesc CMeshOp::descInfo[] = { { LXsMESHOP_PMODEL, "." },
                                        { LXsPMODEL_SELECTIONTYPES, LXsSELOP_TYPE_POLYGON },
                                        { LXsPMODEL_NOTRANSFORM, "." },
                                        { nullptr } };
