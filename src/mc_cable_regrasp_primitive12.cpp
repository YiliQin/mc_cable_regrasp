#include <iostream>

#include "mc_cable_regrasp_primitive12.h"

namespace mc_control
{

Primitive12::Primitive12()
{
}

Primitive12::Primitive12(int primitiveID, std::string primitiveDes, MCCableRegraspController &)
    : BasicPrimitive(primitiveID, primitiveDes)
{
    // For test.
    //std::cout << "Primitive12 construction: " << primitiveID << " " <<  primitiveDes << std::endl; 

    reset();
}

Primitive12::~Primitive12()
{
    // For test.
    //std::cout << "Primitive12 destroy!" << std::endl;
}

void Primitive12::reset()
{
    // Reset FSM signal.
    stepByStep = false;
    paused = false;
    finish = false;
}

void Primitive12::prim_config(double par1, double par2, MCCableRegraspController &)
{
    // Set parameter1.
    slideLen = par1;
    spreadType = int(par2);

    // create FSM
    step = new Prim12InitStep();
}

void Primitive12::prim_fsm_run(MCCableRegraspController & ctl)
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
                    LOG_SUCCESS("Completed: Primitive12 FSM");
                }
            }
        } 
    }
}

double Primitive12::get_slideLen()
{
    return slideLen;
}

int Primitive12::get_spreadType()
{
    return spreadType;
}

}
