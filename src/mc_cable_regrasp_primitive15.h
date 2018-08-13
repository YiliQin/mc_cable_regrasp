#pragma once
#include <iostream>

#include "mc_cable_regrasp_basicPrimitive.h"
#include "mc_cable_regrasp_primitive15_fsm.h"

namespace mc_control
{

struct Primitive15 : public BasicPrimitive
{
    public:
        Primitive15();
        Primitive15(int primitiveID, std::string primitiveDes, MCCableRegraspController & ctl);
        ~Primitive15();
        // Virtual functions.
        virtual void reset() override;
        virtual void prim_config(double par1, double par2, MCCableRegraspController & ctl) override;
        virtual void prim_fsm_run(MCCableRegraspController & ctl) override;
        //
        double get_diff_x();
        double get_diff_y();

    private:
        // FSM pointer.
        Prim15Step * step = nullptr;  
        // Parameter 1 - slide length along the cable.
        double diff_x = 0.0;
        double diff_y = 0.0;
};

}
