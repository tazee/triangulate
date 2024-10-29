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

#define ARGs_COMMAND_TYPE "type"
#define ARGi_COMMAND_TYPE 0

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
        static LXtTextValueHint triangulate_type[] = {
            { ConstraintDelaunay, "constraint" }, { ConformingDelaunay, "conforming" }, { 0, "=triangulate_type" }, 0
        };
        dyna_Add(ARGs_COMMAND_TYPE, LXsTYPE_INTEGER);
        dyna_SetHint(ARGi_COMMAND_TYPE, triangulate_type);

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
};
