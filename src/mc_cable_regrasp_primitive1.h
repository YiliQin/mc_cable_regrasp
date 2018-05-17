#pragma once
#include <iostream>

#include "mc_cable_regrasp_basicPrimitive.h"
#include "mc_cable_regrasp_primitive1_fsm.h"

namespace mc_control
{

struct Primitive1 : public BasicPrimitive
{
    public:
        Primitive1();
        Primitive1(int primitiveID, std::string primitiveDes, double len, MCCableRegraspController & ctl);
        ~Primitive1();
        // Virtual functions.
        virtual void reset() override;
        virtual void prim_fsm_run(MCCableRegraspController & ctl) override;

    public:
        // FSM
        Prim1Step * step = nullptr;  
        bool stepByStep = false;
        bool paused = false;
        bool finish = false;
        double slideLen = 0.0;
};

}
