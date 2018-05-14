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

        void reset();
        void prim4_fsm_run(MCCableRegraspController & ctl);

    public:
        // FSM
        Prim4Step * step = nullptr;  
        bool stepByStep = false;
        bool paused = false;
        bool finish = false;

};

}
