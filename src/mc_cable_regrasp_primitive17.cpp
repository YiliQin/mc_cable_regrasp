#include <iostream>

#include "mc_cable_regrasp_primitive17.h"

namespace mc_control
{

Primitive17::Primitive17()
{
}

Primitive17::Primitive17(int primitiveID, std::string primitiveDes, MCCableRegraspController &)
    : BasicPrimitive(primitiveID, primitiveDes)
{
    // For test.
    //std::cout << "Primitive17 construction: " << primitiveID << " " <<  primitiveDes << std::endl; 

    reset();
}

Primitive17::~Primitive17()
{
    // For test.
    //std::cout << "Primitive17 destroy!" << std::endl;
}

void Primitive17::reset()
{
    // Reset FSM signal.
    stepByStep = false;
    paused = false;
    finish = false;
}

void Primitive17::prim_config(double par1, double par2, MCCableRegraspController &)
{
    // Set parameter1.
    slideLen = par1;
    par2 = par2;

    // create FSM
    step = new Prim17InitStep();
}

void Primitive17::prim_fsm_run(MCCableRegraspController & ctl)
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
                    finish = true;
                    LOG_SUCCESS("Completed: Primitive17 FSM");
                }
            }
        } 
    }
}

double Primitive17::get_slideLen()
{
    return slideLen;
}

}
