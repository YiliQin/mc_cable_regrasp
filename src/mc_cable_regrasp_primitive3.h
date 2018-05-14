#pragma once
#include <iostream>

#include "mc_cable_regrasp_basicPrimitive.h"
#include "mc_cable_regrasp_primitive3_fsm.h"

namespace mc_control
{

struct Primitive3 : public BasicPrimitive
{
    public:
        Primitive3();
        Primitive3(int primitiveID, std::string primitiveDes, MCCableRegraspController & ctl);
        ~Primitive3();

        void reset();
        void prim3_fsm_run(MCCableRegraspController & ctl);

    public:
        // FSM
        Prim3Step * step = nullptr;  
        bool stepByStep = false;
        bool paused = false;
        bool finish = false;

};

}
