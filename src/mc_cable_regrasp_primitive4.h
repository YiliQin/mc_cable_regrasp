#pragma once
#include <iostream>

#include "mc_cable_regrasp_basicPrimitive.h"
#include "mc_cable_regrasp_primitive4_fsm.h"

namespace mc_control
{

struct Primitive4 : public BasicPrimitive
{
    public:
        Primitive4();
        Primitive4(int primitiveID, std::string primitiveDes, MCCableRegraspController & ctl);
        ~Primitive4();
        // Virtual functions.
        virtual void reset() override;
        virtual void prim_fsm_run(MCCableRegraspController & ctl) override;

    public:
        // FSM
        Prim4Step * step = nullptr;  
        bool stepByStep = false;
        bool paused = false;
        bool finish = false;

};

}
