//
// The procedural mesh modifier version of CDT Traingulation named "tool.cdt.item"
//

#pragma once

#include <lxsdk/lxu_attributes.hpp>
#include <lxsdk/lxu_select.hpp>

#include <lxsdk/lx_plugin.hpp>
#include <lxsdk/lx_seltypes.hpp>
#include <lxsdk/lx_tool.hpp>
#include <lxsdk/lx_toolui.hpp>
#include <lxsdk/lx_layer.hpp>
#include <lxsdk/lx_vector.hpp>
#include <lxsdk/lx_pmodel.hpp>
#include <lxsdk/lx_vmodel.hpp>
#include <lxsdk/lx_channelui.hpp>

#include "triangulate_helper.hpp"

using namespace lx_err;

#define SRVNAME_TOOL   "tool.cdt"
#define SRVNAME_TOOLOP "toolop.cdt"

#define ARGs_TYPE           "type"
#define ARGs_QUAD           "quad"
#define ARGs_EDGE           "edge"
#define ARGs_EDGE_SCALE     "edgeScale"
#define ARGs_EDGE_SIZE      "edgeSize"
#define ARGs_ANGLE          "angle"

#define ARGi_TYPE           0
#define ARGi_QUAD           1
#define ARGi_EDGE           2
#define ARGi_EDGE_SCALE     3
#define ARGi_EDGE_SIZE      4
#define ARGi_ANGLE          5

#ifndef LXx_OVERRIDE
#define LXx_OVERRIDE override
#endif

//
// The Tool Operation is evaluated by the procedural modeling system.
//
class CToolOp : public CLxImpl_ToolOperation, public CLxImpl_MeshElementGroup
{
	public:
        // ToolOperation Interface
		LxResult    top_Evaluate(ILxUnknownID vts)  LXx_OVERRIDE;
    
        // MeshElementGroup Interface
        LxResult	eltgrp_GroupCount	(unsigned int *count)				LXx_OVERRIDE;
        LxResult	eltgrp_GroupName	(unsigned int index, const char **name)		LXx_OVERRIDE;
        LxResult	eltgrp_GroupUserName	(unsigned int index, const char **username)	LXx_OVERRIDE;
        LxResult	eltgrp_TestPolygon	(unsigned int index, LXtPolygonID polygon)	LXx_OVERRIDE;
        LxResult    eltgrp_TestEdge(unsigned int index, LXtEdgeID edge)	LXx_OVERRIDE;
        LxResult    eltgrp_TestPoint(unsigned int index, LXtPointID point)	LXx_OVERRIDE;

        void        SetElementGroup(CVisitor& vis);

        CLxUser_Subject2Packet subject;

        unsigned offset_subject;

        int     m_type;
        int     m_quad;
        int     m_edge;
        double  m_edge_scale;
        double  m_edge_size;
        double  m_angle;

        CLxUser_Edge m_cedge;
        std::unordered_set<LXtPointID> m_point_set;
        std::unordered_set<LXtPolygonID> m_polygon_set;
};

/*
 * CDT triangulation tool operator. Basic tool and tool model methods are defined here. The
 * attributes interface is inherited from the utility class.
 */

class CTool : public CLxImpl_Tool, public CLxImpl_ToolModel, public CLxDynamicAttributes, public CLxImpl_ChannelUI
{
public:
    CTool();

    void        tool_Reset() LXx_OVERRIDE;
    LXtObjectID tool_VectorType() LXx_OVERRIDE;
    const char* tool_Order() LXx_OVERRIDE;
    LXtID4      tool_Task() LXx_OVERRIDE;
	LxResult	tool_GetOp(void **ppvObj, unsigned flags) LXx_OVERRIDE;

    LxResult    tmod_Enable(ILxUnknownID obj) LXx_OVERRIDE;

    using CLxDynamicAttributes::atrui_UIHints;  // to distinguish from the overloaded version in CLxImpl_AttributesUI

    void atrui_UIHints2(unsigned int index, CLxUser_UIHints& hints) LXx_OVERRIDE;

    LxResult    cui_Enabled           (const char *channelName, ILxUnknownID msg, ILxUnknownID item, ILxUnknownID read)	LXx_OVERRIDE;
    LxResult    cui_DependencyCount   (const char *channelName, unsigned *count) LXx_OVERRIDE;
    LxResult    cui_DependencyByIndex (const char *channelName, unsigned index, LXtItemType *depItemType, const char **depChannelName) LXx_OVERRIDE;

    bool TestPolygon();

    CLxUser_LogService   s_log;
    CLxUser_LayerService s_layer;
    CLxUser_VectorType   v_type;
    CLxUser_SelectionService s_sel;

    unsigned offset_subject;
	
	LXtItemType m_itemType;

    static void initialize()
    {
        CLxGenericPolymorph* srv;

        srv = new CLxPolymorph<CTool>;
        srv->AddInterface(new CLxIfc_Tool<CTool>);
        srv->AddInterface(new CLxIfc_ToolModel<CTool>);
        srv->AddInterface(new CLxIfc_Attributes<CTool>);
        srv->AddInterface(new CLxIfc_AttributesUI<CTool>);
        srv->AddInterface(new CLxIfc_ChannelUI<CTool>);
        srv->AddInterface(new CLxIfc_StaticDesc<CTool>);
        thisModule.AddServer(SRVNAME_TOOL, srv);

        srv = new CLxPolymorph<CToolOp>;
        srv->AddInterface(new CLxIfc_ToolOperation<CToolOp>);
	    srv->AddInterface(new CLxIfc_MeshElementGroup<CToolOp>);
        lx::AddSpawner(SRVNAME_TOOLOP, srv);
    }

    static LXtTagInfoDesc descInfo[];
};

