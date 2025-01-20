//
// The command version of CDT Traingulation named "poly.cdt"
//
#include "triangulate_command.hpp"

void CCommand::basic_Execute(unsigned int /*flags*/)
{
    CLxUser_LayerScan scan;
    CVisitor          vis;
    unsigned          n;

    attr_GetInt(ARGi_TYPE, &vis.m_triType);
    attr_GetInt(ARGi_QUAD, &vis.m_quad);
    attr_GetInt(ARGi_EDGE, &vis.m_edge);
    attr_GetFlt(ARGi_EDGE_SCALE, &vis.m_edge_scale);
    attr_GetFlt(ARGi_EDGE_SIZE, &vis.m_edge_size);
    attr_GetFlt(ARGi_ANGLE, &vis.m_angle);

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

LxResult CCommand::cmd_DialogInit(void)
{
    if (LXxCMDARG_ISSET(dyna_GetFlags(ARGi_TYPE)) == false)
    {
        attr_SetInt(ARGi_TYPE, ConstraintDelaunay);
    }
    if (LXxCMDARG_ISSET(dyna_GetFlags(ARGi_QUAD)) == false)
    {
        attr_SetInt(ARGi_QUAD, Split_1_3);
    }
    if (LXxCMDARG_ISSET(dyna_GetFlags(ARGi_EDGE)) == false)
    {
        attr_SetInt(ARGi_EDGE, ByRatio);
    }
    if (LXxCMDARG_ISSET(dyna_GetFlags(ARGi_EDGE_SCALE)) == false)
    {
        TriangulateHelper triHelp;
        attr_SetFlt(ARGi_EDGE_SCALE, triHelp.m_edge_size);
    }
    if (LXxCMDARG_ISSET(dyna_GetFlags(ARGi_EDGE_SIZE)) == false)
    {
        TriangulateHelper triHelp;
        attr_SetFlt(ARGi_EDGE_SIZE, triHelp.m_edge_size);
    }
    if (LXxCMDARG_ISSET(dyna_GetFlags(ARGi_ANGLE)) == false)
    {
        TriangulateHelper triHelp;
        attr_SetFlt(ARGi_ANGLE, triHelp.m_angle_min);
    }

    return LXe_OK;
}

	
void CCommand::atrui_UIHints2(unsigned int index, CLxUser_UIHints &hints)
{
    switch (index)
    {
        case ARGi_EDGE_SCALE:
            hints.MinFloat(0.0);
            hints.MaxFloat(1.0);
            break;

        case ARGi_EDGE_SIZE:
            hints.MinFloat(0.0);
            break;

        case ARGi_ANGLE:
            hints.MinFloat(1.0 * LXx_DEG2RAD);
            hints.MaxFloat(30.0 * LXx_DEG2RAD);
            break;
    }
}

LxResult CCommand::cmd_ArgEnable (unsigned int arg)
{
    int	type, edge;

    attr_GetInt (ARGi_TYPE, &type);
    attr_GetInt (ARGi_EDGE, &edge);

    switch (arg)
    {
        case ARGi_QUAD:
            if (type != ConstraintDelaunay)
                return LXe_CMD_DISABLED;
            break;

        case ARGi_EDGE:
            if (type != ConformingDelaunay)
                return LXe_CMD_DISABLED;
            break;

        case ARGi_EDGE_SCALE:
            if ((type != ConformingDelaunay) || (edge != ByRatio))
                return LXe_CMD_DISABLED;
            break;

        case ARGi_EDGE_SIZE:
            if ((type != ConformingDelaunay) || (edge != ByLength))
                return LXe_CMD_DISABLED;
            break;

        case ARGi_ANGLE:
            if (type != ConformingDelaunay)
                return LXe_CMD_DISABLED;
            break;
    }
    return LXe_OK;
}