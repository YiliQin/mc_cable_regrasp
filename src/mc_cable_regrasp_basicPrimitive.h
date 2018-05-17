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
        virtual void prim_config(double par1, double par2, MCCableRegraspController & ctl);
        virtual void prim_fsm_run(MCCableRegraspController & ctl);
        //
        int getID();
        std::string getDes();
        // FSM interface.
        bool get_finish();    
        void idle();
        void set_stepByStep(bool stepByStep_);

    protected:
        // ID and description. 
        int primitiveID = 0;
        std::string primitiveDes = "Please Input Description.";
        // FSM control signal.
        bool finish = false;
        bool paused = false;
        bool stepByStep = false;
};


}
