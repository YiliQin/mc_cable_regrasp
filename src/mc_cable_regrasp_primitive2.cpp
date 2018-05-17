#include <iostream>

#include "mc_cable_regrasp_primitive2.h"

namespace mc_control
{

Primitive2::Primitive2()
{
}

Primitive2::Primitive2(int primitiveID, std::string primitiveDes, double slideLen, MCCableRegraspController & ctl)
    : BasicPrimitive(primitiveID, primitiveDes), slideLen(slideLen)
{
    // For test.
    //std::cout << "Primitive2 construction: " << primitiveID << " " <<  primitiveDes << std::endl; 
    ctl.neglectFctInp = ctl.neglectFctInp;

    // Reset.
    reset();
}

Primitive2::~Primitive2()
{
    // For test.
    //std::cout << "Primitive2 destroy!" << std::endl;
}

void Primitive2::reset()
{
    stepByStep = false;
    paused = false;
    finish = false;
    // Create Primitive2.
    step = new Prim2InitStep();
}

void Primitive2::prim2_fsm_run(MCCableRegraspController & ctl)
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
                    LOG_SUCCESS("Completed: Primitive2 FSM");
                }
            }
        } 
    }
}

}
