#include <iostream>

#include "mc_cable_regrasp_primitive18.h"

namespace mc_control
{

Primitive18::Primitive18()
{
}

Primitive18::Primitive18(int primitiveID, std::string primitiveDes, MCCableRegraspController &)
    : BasicPrimitive(primitiveID, primitiveDes)
{
    // For test.
    //std::cout << "Primitive18 construction: " << primitiveID << " " <<  primitiveDes << std::endl; 

    reset();
}

Primitive18::~Primitive18()
{
    // For test.
    //std::cout << "Primitive18 destroy!" << std::endl;
}

void Primitive18::reset()
{
    // Reset FSM signal.
    stepByStep = false;
    paused = false;
    finish = false;
}

void Primitive18::prim_config(double par1, double par2, MCCableRegraspController &)
{
    // Set parameter1.
    slideLen = par1;
    par2 = par2;

    // create FSM
    step = new Prim18InitStep();
}

void Primitive18::prim_fsm_run(MCCableRegraspController & ctl)
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
                    LOG_SUCCESS("Completed: Primitive18 FSM");
                }
            }
        } 
    }
}

double Primitive18::get_slideLen()
{
    return slideLen;
}

}
