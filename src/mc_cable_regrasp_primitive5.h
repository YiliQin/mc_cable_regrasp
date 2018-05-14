#pragma once
#include <iostream>

#include "mc_cable_regrasp_basicPrimitive.h"
#include "mc_cable_regrasp_primitive5_fsm.h"

namespace mc_control
{

struct Primitive5 : public BasicPrimitive
{
    public:
        Primitive5();
        Primitive5(int primitiveID, std::string primitiveDes, MCCableRegraspController & ctl);
        ~Primitive5();

        void reset();
        void prim5_fsm_run(MCCableRegraspController & ctl);

    public:
        // FSM
        Prim5Step * step = nullptr;        

        bool stepByStep = false;
        bool paused = false;
        bool finish = false;
};

}
