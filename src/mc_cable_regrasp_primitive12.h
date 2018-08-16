#pragma once
#include <iostream>

#include "mc_cable_regrasp_basicPrimitive.h"
#include "mc_cable_regrasp_primitive12_fsm.h"

namespace mc_control
{

struct Primitive12 : public BasicPrimitive
{
    public:
        Primitive12();
        Primitive12(int primitiveID, std::string primitiveDes, MCCableRegraspController & ctl);
        ~Primitive12();
        // Virtual functions.
        virtual void reset() override;
        virtual void prim_config(double par1, double par2, MCCableRegraspController & ctl) override;
        virtual void prim_fsm_run(MCCableRegraspController & ctl) override;
        //
        double get_slideLen();
        int get_spreadType();
    private:
        // FSM pointer.
        Prim12Step * step = nullptr;  
        // Parameter 1 - slide length along the cable.
        double slideLen = 0.0;
        int spreadType = 1; 
};

}
