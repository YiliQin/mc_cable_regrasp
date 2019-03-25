#include <iostream>

#include "mc_cable_regrasp_primitive15.h"

namespace mc_control
{

Primitive15::Primitive15()
{
}

Primitive15::Primitive15(int primitiveID, std::string primitiveDes, MCCableRegraspController &)
    : BasicPrimitive(primitiveID, primitiveDes)
{
    // For test.
    //std::cout << "Primitive15 construction: " << primitiveID << " " <<  primitiveDes << std::endl; 

    reset();
}

Primitive15::~Primitive15()
{
    // For test.
    //std::cout << "Primitive15 destroy!" << std::endl;
}

void Primitive15::reset()
{
    // Reset FSM signal.
    stepByStep = false;
    paused = false;
    finish = false;
}

void Primitive15::prim_config(double par1, double par2, MCCableRegraspController &)
{
    // Set parameter1.
    diff_x = par1;
    diff_y = par2;

    // create FSM
    step = new Prim15InitStep();
}

void Primitive15::prim_fsm_run(MCCableRegraspController & ctl)
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
                    LOG_SUCCESS("Completed: Primitive15 FSM");
                }
            }
        } 
    }
}

double Primitive15::get_diff_x()
{
    return diff_x;
}

double Primitive15::get_diff_y()
{
    return diff_y;
}

}
