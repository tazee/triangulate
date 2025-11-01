//
// The command version of CDT Traingulation named "poly.cdt"
//

#pragma once

#include <lxsdk/lxu_command.hpp>

#include <lxsdk/lx_layer.hpp>
#include <lxsdk/lx_log.hpp>
#include <lxsdk/lxu_attributes.hpp>
#include <lxsdk/lxu_math.hpp>
#include <lxsdk/lxu_vector.hpp>

#include "lxsdk/lxvalue.h"
#include "triangulate_helper.hpp"

using namespace lx_err;

#define SRVNAME_COMMAND "poly.cdt"

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

class CCommand : public CLxBasicCommand
{
public:
    CLxUser_LayerService lyr_S;
    CLxUser_MeshService  msh_S;
    unsigned             select_mode;

    CCommand()
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

        check(msh_S.ModeCompose("select", NULL, &select_mode));
    }

    static void initialize()
    {
        CLxGenericPolymorph* srv;

        srv = new CLxPolymorph<CCommand>;
        srv->AddInterface(new CLxIfc_Command<CCommand>);
        srv->AddInterface(new CLxIfc_Attributes<CCommand>);
        srv->AddInterface(new CLxIfc_AttributesUI<CCommand>);
        lx::AddServer(SRVNAME_COMMAND, srv);
    }

    int  basic_CmdFlags() LXx_OVERRIDE;
    bool basic_Enable(CLxUser_Message& msg) LXx_OVERRIDE;

    void basic_Execute(unsigned int flags) LXx_OVERRIDE;
	
	void atrui_UIHints2(unsigned int index, CLxUser_UIHints &hints) LXx_OVERRIDE;

    LxResult cmd_DialogInit(void) LXx_OVERRIDE;
    LxResult cmd_ArgEnable (unsigned int arg) LXx_OVERRIDE;
};
