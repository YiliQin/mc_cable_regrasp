#pragma once
#include <iostream>

#include "mc_cable_regrasp_basicPrimitive.h"
#include "mc_cable_regrasp_primitive14_fsm.h"

namespace mc_control
{

struct Primitive14 : public BasicPrimitive
{
    public:
        Primitive14();
        Primitive14(int primitiveID, std::string primitiveDes, MCCableRegraspController & ctl);
        ~Primitive14();
        // Virtual functions.
        virtual void reset() override;
        virtual void prim_config(double par1, double par2, MCCableRegraspController & ctl) override;
        virtual void prim_fsm_run(MCCableRegraspController & ctl) override;
        //
        double get_dis_x();
        double get_dis_y();
    private:
        // FSM pointer.
        Prim14Step * step = nullptr;        
        // Parameter 1 - the distance between two grippers.
        double dis_x = 0.0;
        double dis_y = 0.0;
};

}
