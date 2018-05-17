#pragma once

#include <Eigen/Core>
#include <string>

#include "mc_cable_regrasp_controller.h"

namespace mc_control
{

struct BasicPrimitive
{
    public:
        BasicPrimitive() {};
        BasicPrimitive(int primitiveID, std::string primitiveDes);
        ~BasicPrimitive();
        // Virtual function.
        virtual void reset();
        virtual void prim_fsm_run(MCCableRegraspController & ctl);
        //
        int getID();
        std::string getDes();
    
    public:
        // ID and description 
        int primitiveID = 0;
        std::string primitiveDes = "Please Input Description.";
};


}
