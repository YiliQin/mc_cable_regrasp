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
        // Virtual functions.
        virtual void reset() override;
        virtual void prim_config(double par1, double par2, MCCableRegraspController & ctl) override;
        virtual void prim_fsm_run(MCCableRegraspController & ctl) override;
        //
        double get_distance();
    private:
        // FSM pointer.
        Prim5Step * step = nullptr;        
        // Parameter 1 - the distance between two grippers.
        double disBetHands = 0.0;
};

}
