#include <iostream>

#include "mc_cable_regrasp_primitive4.h"

namespace mc_control
{

Primitive4::Primitive4()
{
}

Primitive4::Primitive4(int primitiveID, std::string primitiveDes, MCCableRegraspController &)
    : BasicPrimitive(primitiveID, primitiveDes)
{
    // For test.
    //std::cout << "Primitive4 construction: " << primitiveID << " " <<  primitiveDes << std::endl; 

    reset();
}

Primitive4::~Primitive4()
{
    // For test.
    //std::cout << "Primitive4 destroy!" << std::endl;
}

void Primitive4::reset()
{
    // Reset FSM signal.
    stepByStep = false;
    paused = false;
    finish = false;
}

void Primitive4::prim_config(double par1, double par2, MCCableRegraspController &)
{
    // Set parameter.
    par1 = par1;
    par2 = par2;

    step = new Prim4InitStep();
}

void Primitive4::prim_fsm_run(MCCableRegraspController & ctl)
{    
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
                    LOG_SUCCESS("Completed: Primitive4 FSM");
                }
            }
        } 
    }
}

}
