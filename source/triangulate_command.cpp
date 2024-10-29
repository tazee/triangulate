//
// The command version of CDT Traingulation named "poly.cdt"
//
#include "triangulate_command.hpp"

void CCommand::basic_Execute(unsigned int /*flags*/)
{
    CLxUser_LayerScan scan;
    CVisitor          vis;
    unsigned          n;

    attr_GetInt(ARGi_COMMAND_TYPE, &vis.m_triType);

    check(lyr_S.BeginScan(LXf_LAYERSCAN_EDIT_POLYS, scan));
    check(scan.Count(&n));

    for (auto i = 0u; i < n; i++)
    {
        check(scan.BaseMeshByIndex(i, vis.base_mesh));
        check(scan.EditMeshByIndex(i, vis.edit_mesh));

        vis.triHelp.SetMesh(vis.edit_mesh, vis.base_mesh);

        check(vis.m_vert.fromMesh(vis.edit_mesh));
        check(vis.m_poly.fromMesh(vis.edit_mesh));
        check(vis.m_poly.Enum(&vis, select_mode));

        scan.SetMeshChange(i, LXf_MESHEDIT_GEOMETRY);
    }

    if (vis.succeeded == false)
    {
        basic_Message().SetMsg("cdt_tri", "invalid");
        basic_Message().SetCode(LXe_FAILED);
    }

    scan.Apply();
}

int CCommand::basic_CmdFlags()
{
    return LXfCMD_MODEL | LXfCMD_UNDO;
}

bool CCommand::basic_Enable(CLxUser_Message& /*msg*/)
{
    int      flags = 0;
    unsigned count;

    check(lyr_S.SetScene(0));
    check(lyr_S.Count(&count));

    for (unsigned i = 0; i < count; i++)
    {
        check(lyr_S.Flags(i, &flags));
        if (flags & LXf_LAYERSCAN_ACTIVE)
            return true;
    }

    return false;
}
