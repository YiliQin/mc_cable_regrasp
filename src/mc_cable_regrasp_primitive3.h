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
        // Vritual functions.
        virtual void reset() override;
        virtual void prim_fsm_run(MCCableRegraspController & ctl) override;

    public:
        // FSM
        Prim3Step * step = nullptr;  
        bool stepByStep = false;
        bool paused = false;
        bool finish = false;

};

}
