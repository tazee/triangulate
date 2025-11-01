//
// The procedural mesh modifier version of CDT Traingulation named "tool.cdt.item"
//

#include "triangulate_helper.hpp"
#include "triangulate_tool.hpp"

//
// On create we add our one tool attribute. We also allocate a vector type
// and select mode mask.
//
CTool::CTool()
{
    static const LXtTextValueHint triangulate_type[] = {
        { ConstraintDelaunay, "constraint" }, 
        { ConformingDelaunay, "conforming" }, 
        { ConvexPartitioning, "convexPartitioning" }, 
        { 0, "=triangulate_type" }, 0
    };
    static const LXtTextValueHint triangulate_quad[] = {
        { ShortestDiagonal, "shortest" },
        { LongestDiagonal, "longest" },
        { Split_1_3, "split1to3" },
        { Split_2_4, "split2to4" },
        { 0, "=triangulate_quad" }, 0
    };
    static const LXtTextValueHint triangulate_edge[] = {
        { 0, "ratio" }, 
        { 1, "length" }, 
        { 0, "=triangulate_edge" }, 0
    };

    dyna_Add(ARGs_TYPE, LXsTYPE_INTEGER);
    dyna_SetHint(ARGi_TYPE, triangulate_type);

    dyna_Add(ARGs_QUAD, LXsTYPE_INTEGER);
    dyna_SetHint(ARGi_QUAD, triangulate_quad);

    dyna_Add(ARGs_EDGE, LXsTYPE_INTEGER);
    dyna_SetHint(ARGi_EDGE, triangulate_edge);

    dyna_Add(ARGs_EDGE_SCALE, LXsTYPE_PERCENT);

    dyna_Add(ARGs_EDGE_SIZE, LXsTYPE_DISTANCE);

    dyna_Add(ARGs_ANGLE, LXsTYPE_ANGLE);
    
    dyna_Add(ARGs_EDGE, LXsTYPE_INTEGER);

    CLxUser_PacketService sPkt;
    CLxUser_MeshService   sMesh;
	CLxUser_SceneService  svc;

    tool_Reset();

    sPkt.NewVectorType(LXsCATEGORY_TOOL, v_type);
    sPkt.AddPacket(v_type, LXsP_TOOL_SUBJECT2, LXfVT_GET);

    offset_subject = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_SUBJECT2);

    m_itemType =  svc.ItemType (SRVNAME_TOOL".item");
}

//
// Reset sets the attributes back to defaults.
//
void CTool::tool_Reset()
{
    TriangulateHelper triHelp;
    dyna_Value(ARGi_TYPE).SetInt(ConstraintDelaunay);
    dyna_Value(ARGi_QUAD).SetInt(Split_1_3);
    dyna_Value(ARGi_EDGE).SetInt(ByRatio);
    dyna_Value(ARGi_EDGE_SCALE).SetFlt(triHelp.m_edge_size);
    dyna_Value(ARGi_EDGE_SIZE).SetFlt(triHelp.m_edge_size);
    dyna_Value(ARGi_ANGLE).SetFlt(triHelp.m_angle_min);
}

LXtObjectID CTool::tool_VectorType()
{
    return v_type.m_loc;  // peek method; does not add-ref
}

const char* CTool::tool_Order()
{
    return LXs_ORD_ACTR;
}

LXtID4 CTool::tool_Task()
{
    return LXi_TASK_ACTR;
}

LxResult CTool::tool_GetOp(void** ppvObj, unsigned flags)
{
    CLxSpawner<CToolOp> spawner(SRVNAME_TOOLOP);
    CToolOp*            toolop = spawner.Alloc(ppvObj);

	if (!toolop)
	{
		return LXe_FAILED;
	}

    dyna_Value(ARGi_TYPE).GetInt(&toolop->m_type);
    dyna_Value(ARGi_QUAD).GetInt(&toolop->m_quad);
    dyna_Value(ARGi_EDGE).GetInt(&toolop->m_edge);
    dyna_Value(ARGi_EDGE_SCALE).GetFlt(&toolop->m_edge_scale);
    dyna_Value(ARGi_EDGE_SIZE).GetFlt(&toolop->m_edge_size);
    dyna_Value(ARGi_ANGLE).GetFlt(&toolop->m_angle);

    toolop->offset_subject = offset_subject;

	return LXe_OK;
}

LXtTagInfoDesc CTool::descInfo[] =
{
	{LXsTOOL_PMODEL, "."},
	{LXsTOOL_USETOOLOP, "."},
	{LXsPMODEL_SELECTIONTYPES, LXsSELOP_TYPE_POLYGON},
    {LXsPMODEL_NOTRANSFORM, "."},
	{0}

};

LxResult CTool::tmod_Enable(ILxUnknownID obj)
{
    CLxUser_Message msg(obj);

    if (TestPolygon() == false)
    {
        msg.SetCode(LXe_CMD_DISABLED);
        msg.SetMessage(SRVNAME_TOOL, "NoPolygon", 0);
        return LXe_DISABLED;
    }
    return LXe_OK;
}

void CTool::atrui_UIHints2(unsigned int index, CLxUser_UIHints& hints)
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

bool CTool::TestPolygon()
{
    //
    // Start the scan in read-only mode.
    //
    CLxUser_LayerScan scan;
    CLxUser_Mesh      mesh;
    unsigned          i, n, count;
    bool              ok = false;

    s_layer.BeginScan(LXf_LAYERSCAN_ACTIVE | LXf_LAYERSCAN_MARKPOLYS, scan);

    //
    // Count the polygons in all mesh layers.
    //
    if (scan)
    {
        n = scan.NumLayers();
        for (i = 0; i < n; i++)
        {
            scan.BaseMeshByIndex(i, mesh);
            mesh.PolygonCount(&count);
            if (count > 0)
            {
                ok = true;
                break;
            }
        }
        scan.Apply();
    }

    //
    // Return false if there is no polygons in any active layers.
    //
    return ok;
}

LxResult CTool::cui_Enabled (const char *channelName, ILxUnknownID msg_obj, ILxUnknownID item_obj, ILxUnknownID read_obj)
{
	CLxUser_Item	 	 item (item_obj);
	CLxUser_ChannelRead	 chan_read (read_obj);

    std::string name(channelName);

	if (name == ARGs_EDGE)
    {
        if (chan_read.IValue (item, ARGs_TYPE) != ConformingDelaunay)
		    return LXe_CMD_DISABLED;
    }
	else if (name == ARGs_QUAD)
    {
        if (chan_read.IValue (item, ARGs_TYPE) != ConstraintDelaunay)
		    return LXe_CMD_DISABLED;
    }
	else if (name == ARGs_EDGE_SCALE)
    {
        if (chan_read.IValue (item, ARGs_TYPE) != ConformingDelaunay)
		    return LXe_CMD_DISABLED;
        if (chan_read.IValue (item, ARGs_EDGE) != ByRatio)
		    return LXe_CMD_DISABLED;
    }
	else if (name == ARGs_EDGE_SIZE)
    {
        if (chan_read.IValue (item, ARGs_TYPE) != ConformingDelaunay)
		    return LXe_CMD_DISABLED;
        if (chan_read.IValue (item, ARGs_EDGE) != ByLength)
		    return LXe_CMD_DISABLED;
    }
	else if (name == ARGs_ANGLE)
    {
        if (chan_read.IValue (item, ARGs_TYPE) != ConformingDelaunay)
		    return LXe_CMD_DISABLED;
    }
	
	return LXe_OK;
}

LxResult CTool::cui_DependencyCount (const char *channelName, unsigned *count)
{
	count[0] = 0;

	if (std::string(channelName) == ARGs_EDGE)
		count[0] = 1;
	else if (std::string(channelName) == ARGs_QUAD)
		count[0] = 1;
	else if (std::string(channelName) == ARGs_EDGE_SCALE)
		count[0] = 2;
	else if (std::string(channelName) == ARGs_EDGE_SIZE)
		count[0] = 2;
	else if (std::string(channelName) == ARGs_ANGLE)
		count[0] = 1;
	
	return LXe_OK;
}

LxResult CTool::cui_DependencyByIndex (const char *channelName, unsigned index, LXtItemType *depItemType, const char **depChannel)
{
printf("DependencyByIndex: %s\n", channelName);

	depItemType[0] = m_itemType;
	
	if (std::string(channelName) == ARGs_EDGE)
	{
		depChannel[0] = ARGs_TYPE;
		return LXe_OK;
	}	
	else if (std::string(channelName) == ARGs_QUAD)
	{
		depChannel[0] = ARGs_TYPE;
		return LXe_OK;
	}
	else if (std::string(channelName) == ARGs_EDGE_SCALE)
	{
        if (index == 0)
            depChannel[0]  = ARGs_TYPE;
        else
            depChannel[0]  = ARGs_EDGE;
        return LXe_OK;
	}
	else if (std::string(channelName) == ARGs_EDGE_SIZE)
	{
        if (index == 0)
            depChannel[0]  = ARGs_TYPE;
        else
            depChannel[0]  = ARGs_EDGE;
        return LXe_OK;
	}
	else if (std::string(channelName) == ARGs_ANGLE)
	{
		depChannel[0] = ARGs_TYPE;
		return LXe_OK;
	}
		
	return LXe_OUTOFBOUNDS;
}

//
// Tool evaluation uses layer scan interface to walk through all the active
// meshes and visit all the selected polygons.
//
LxResult CToolOp::top_Evaluate(ILxUnknownID vts)
{
    CLxUser_VectorStack vec(vts);

    //
    // Start the scan in edit mode.
    //
    CLxUser_LayerScan  scan;

    if (vec.ReadObject(offset_subject, subject) == false)
        return LXe_FAILED;

    CLxUser_MeshService   s_mesh;

    LXtMarkMode pick = s_mesh.SetMode(LXsMARK_SELECT);

    subject.BeginScan(LXf_LAYERSCAN_EDIT_POLVRT, scan);

    auto n = scan.NumLayers();
    for (auto i = 0u; i < n; i++)
    {
        CVisitor vis;

        scan.BaseMeshByIndex(i, vis.base_mesh);
        scan.EditMeshByIndex(i, vis.edit_mesh);

        vis.m_triType    = m_type;
        vis.m_quad       = m_quad;
        vis.m_edge       = m_edge;
        vis.m_edge_scale = m_edge_scale;
        vis.m_edge_size  = m_edge_size;
        vis.m_angle      = m_angle;

        printf("** edge (%d) scale (%f) szie (%f) angle (%f)\n", m_edge, m_edge_scale, m_edge_size, m_angle);
        vis.triHelp.SetMesh(vis.edit_mesh, vis.base_mesh);

        check(vis.m_vert.fromMesh(vis.edit_mesh));
        check(vis.m_poly.fromMesh(vis.edit_mesh));

        // Triangulate selected polygons using CDT traingulation method.
        check(vis.m_poly.Enum(&vis, pick));

        // Cache triangles for element group
        SetElementGroup(vis);

        scan.SetMeshChange(i, LXf_MESHEDIT_GEOMETRY);
    }

    scan.Apply();
    return LXe_OK;
}

void CToolOp::SetElementGroup(CVisitor& vis)
{
    m_polygon_set.insert(vis.tris.begin(), vis.tris.end());
    for (auto pol : vis.tris)
    {
        unsigned count;
        vis.m_poly.Select(pol);
        vis.m_poly.VertexCount(&count);
        for (auto i = 0u; i < count; i++)
        {
            LXtPointID pnt;
            vis.m_poly.VertexByIndex(i, &pnt);
            m_point_set.insert(pnt);
        }
    }
    vis.edit_mesh.GetEdges(m_cedge);
}

LxResult CToolOp::eltgrp_GroupCount(unsigned int* count)
{
    count[0] = 3;
    return LXe_OK;
}

LxResult CToolOp::eltgrp_GroupName(unsigned int index, const char** name)
{
    const static char* names[] = {"newPoly",
                            "newEdge",
                            "newVerx"};
    name[0] = names[index];
    return LXe_OK;
}

LxResult CToolOp::eltgrp_GroupUserName(unsigned int index, const char** username)
{
    const static char* names[] = {"@tool.cdt@newPoly@0",
                            "@tool.cdt@newEdge@",
                            "@tool.cdt@newVerx@"};
    username[0] = names[index];
    return LXe_OK;
}

LxResult CToolOp::eltgrp_TestPolygon(unsigned int index, LXtPolygonID polygon)
{
    if (m_polygon_set.find(polygon) != m_polygon_set.end())
        return LXe_TRUE;
    return LXe_FALSE;
}

LxResult CToolOp::eltgrp_TestEdge(unsigned int index, LXtEdgeID edge)
{
	LXtPointID   p0, p1;

    m_cedge.Select(edge);
    m_cedge.Endpoints(&p0, &p1);
    if (m_point_set.find(p0) == m_point_set.end())
        return LXe_FALSE;
    if (m_point_set.find(p1) == m_point_set.end())
        return LXe_FALSE;
    return LXe_TRUE;
}

LxResult CToolOp::eltgrp_TestPoint(unsigned int index, LXtPointID point)
{
    if (m_point_set.find(point) != m_point_set.end())
        return LXe_TRUE;
    return LXe_FALSE;
}
