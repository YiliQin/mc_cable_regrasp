#include <iostream>

#include "mc_cable_regrasp_primitive1.h"

namespace mc_control
{

Primitive1::Primitive1()
{
}

Primitive1::Primitive1(int primitiveID, std::string primitiveDes, double slideLen, MCCableRegraspController & ctl)
    : BasicPrimitive(primitiveID, primitiveDes), slideLen(slideLen)
{
    // For test.
    //std::cout << "Primitive1 construction: " << primitiveID << " " <<  primitiveDes << std::endl; 
    ctl.fsmtest = 1;

    // Reset.
    reset();
}

Primitive1::~Primitive1()
{
    // For test.
    //std::cout << "Primitive1 destroy!" << std::endl;
}

void Primitive1::reset()
{
    stepByStep = false;
    paused = false;
    finish = false;
    // Create Primitive1.
    step = new Prim1InitStep();
}

void Primitive1::prim1_fsm_run(MCCableRegraspController & ctl)
{    
    // For test.
    if (!paused)
    {
        if (step)
        {
            auto nstep = step->update(ctl);
            if (nstep != step)
            {
                if (stepByStep == true)
                {
                 
                }
                else if (stepByStep == false)
                {
                    LOG_SUCCESS("Completed: " << step->name);
                }
                delete step; 
                step = nstep;
                //// Control the FSM step.
                //paused = stepByStep;
                if (step == nullptr)
                {
                    LOG_SUCCESS("Completed: Primitive1 FSM");
                }
            }
        } 
    }
}

}
