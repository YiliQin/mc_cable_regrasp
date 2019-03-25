#pragma once
#include <iostream>

#include "mc_cable_regrasp_basicPrimitive.h"
#include "mc_cable_regrasp_primitive17_fsm.h"

namespace mc_control
{

struct Primitive17 : public BasicPrimitive
{
    public:
        Primitive17();
        Primitive17(int primitiveID, std::string primitiveDes, MCCableRegraspController & ctl);
        ~Primitive17();
        // Virtual functions.
        virtual void reset() override;
        virtual void prim_config(double par1, double par2, MCCableRegraspController & ctl) override;
        virtual void prim_fsm_run(MCCableRegraspController & ctl) override;
        //
        double get_slideLen();

    private:
        // FSM pointer.
        Prim17Step * step = nullptr;  
        // Parameter 1 - slide length along the cable.
        double slideLen = 0.0;
};

}
