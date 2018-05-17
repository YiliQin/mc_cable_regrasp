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
        Primitive5(int primitiveID, std::string primitiveDes, double disBetHands, MCCableRegraspController & ctl);
        ~Primitive5();
        // Virtual functions.
        virtual void reset() override;
        virtual void prim_fsm_run(MCCableRegraspController & ctl) override;

    public:
        // FSM
        Prim5Step * step = nullptr;        

        bool stepByStep = false;
        bool paused = false;
        bool finish = false;
        double disBetHands = 0.6;
};

}
