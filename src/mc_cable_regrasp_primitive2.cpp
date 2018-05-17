#include <iostream>

#include "mc_cable_regrasp_primitive2.h"

namespace mc_control
{

Primitive2::Primitive2()
{
}

Primitive2::Primitive2(int primitiveID, std::string primitiveDes, MCCableRegraspController & ctl)
    : BasicPrimitive(primitiveID, primitiveDes)
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
    // Reset FSM signal.
    stepByStep = false;
    paused = false;
    finish = false;
}

void Primitive2::prim_fsm_run(MCCableRegraspController & ctl)
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
                    LOG_SUCCESS("Completed: Primitive2 FSM");
                }
            }
        } 
    }
}

void Primitive2::prim_config(double par1, double par2, MCCableRegraspController & ctl)
{
    // Set parameter1.
    slideLen = par1;
    par2 = par2;
    ctl.neglectFctInp = ctl.neglectFctInp;

    step = new Prim2InitStep();
}

double Primitive2::get_slideLen()
{
    return slideLen;
}

}
