//
// The procedural mesh modifier version of CDT Traingulation named "mesh.cdt.item"
//

#pragma once

#include <lxsdk/lxu_attributes.hpp>
#include <lxsdk/lxu_math.hpp>
#include <lxsdk/lxu_matrix.hpp>
#include <lxsdk/lxu_modifier.hpp>
#include <lxsdk/lxu_package.hpp>
#include <lxsdk/lxu_select.hpp>
#include <lxsdk/lxu_vector.hpp>

#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_package.hpp>
#include <lxsdk/lx_particle.hpp>
#include <lxsdk/lx_plugin.hpp>
#include <lxsdk/lx_pmodel.hpp>
#include <lxsdk/lx_seltypes.hpp>
#include <lxsdk/lx_server.hpp>
#include <lxsdk/lx_vector.hpp>
#include <lxsdk/lx_vertex.hpp>
#include <lxsdk/lx_visitor.hpp>
#include <lxsdk/lx_wrap.hpp>

#include <lxsdk/lxw_mesh.hpp>
#include <lxsdk/lxw_pmodel.hpp>
#include <lxsdk/lxvalue.h>

#include "triangulate_helper.hpp"

using namespace lx_err;

#define SRVNAME_MESHOP "meshop.cdt"

#define ARGs_MESHOP_TYPE "type"
#define ARGi_MESHOP_TYPE 0

#ifndef LXx_OVERRIDE
#define LXx_OVERRIDE override
#endif

class CMeshOp : public CLxImpl_MeshOperation, public CLxDynamicAttributes, public CLxImpl_MeshElementGroup
{
public:
    CLxUser_MeshService  msh_S;
    unsigned             select_mode;

    CMeshOp()
    {
        static LXtTextValueHint triangulate_type[] = {
            { ConstraintDelaunay, "constraint" }, { ConformingDelaunay, "conforming" }, { 0, "=triangulate_type" }, 0
        };

        dyna_Add(ARGs_MESHOP_TYPE, LXsTYPE_INTEGER);
        dyna_SetHint(ARGi_MESHOP_TYPE, triangulate_type);

        check(msh_S.ModeCompose("select", NULL, &select_mode));
    }

    static void initialize()
    {
        CLxGenericPolymorph* srv = new CLxPolymorph<CMeshOp>;

        srv->AddInterface(new CLxIfc_MeshOperation<CMeshOp>);
        srv->AddInterface(new CLxIfc_Attributes<CMeshOp>);
        srv->AddInterface(new CLxIfc_StaticDesc<CMeshOp>);

        lx::AddServer(SRVNAME_MESHOP, srv);
    }

    LXxO_MeshOperation_Evaluate;

    static LXtTagInfoDesc descInfo[];
                                        
};
