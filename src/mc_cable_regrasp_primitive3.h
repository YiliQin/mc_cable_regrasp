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
        virtual void prim_config(double par1, double par2, MCCableRegraspController & ctl) override;
        virtual void prim_fsm_run(MCCableRegraspController & ctl) override;

    public:
        // FSM pointer.
        Prim3Step * step = nullptr;  
};

}
