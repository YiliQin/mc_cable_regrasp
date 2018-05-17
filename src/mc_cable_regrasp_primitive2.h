#pragma once
#include <iostream>

#include "mc_cable_regrasp_basicPrimitive.h"
#include "mc_cable_regrasp_primitive2_fsm.h"

namespace mc_control
{

struct Primitive2 : public BasicPrimitive
{
    public:
        Primitive2();
        Primitive2(int primitiveID, std::string primitiveDes, double len, MCCableRegraspController & ctl);
        ~Primitive2();
        // Virtual functions.
        virtual void reset() override;
        virtual void prim_fsm_run(MCCableRegraspController & ctl) override;

    public:
        // FSM
        Prim2Step * step = nullptr;  
        bool stepByStep = false;
        bool paused = false;
        bool finish = false;
        double slideLen = 0.0;
};

}
