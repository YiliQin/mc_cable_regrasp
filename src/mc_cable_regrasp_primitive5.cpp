#include <iostream>

#include "mc_cable_regrasp_primitive5.h"

namespace mc_control
{

Primitive5::Primitive5()
{
}

Primitive5::Primitive5(int primitiveID, std::string primitiveDes, MCCableRegraspController &)
    : BasicPrimitive(primitiveID, primitiveDes)
{
    // For test.
    //std::cout << "Primitive5 construction: " << primitiveID << " " <<  primitiveDes << std::endl; 

    reset();
}

Primitive5::~Primitive5()
{
    // For test.
    //std::cout << "Primitive5 destroy!" << std::endl;
}

void Primitive5::reset()
{
    // Reset FSM signal.
    stepByStep = false;
    paused = false;
    finish = false;
}

void Primitive5::prim_config(double par1, double par2, MCCableRegraspController &)
{
    // Set parameter1.
    disBetHands = par1;
    par2 = par2;

    step = new Prim5InitStep();
}

void Primitive5::prim_fsm_run(MCCableRegraspController & ctl)
{
    // For test.
    if (!paused)
    {
        if (step)
        {
            auto nstep = step->update(ctl);
            //nstep = step->update(ctl);
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
                    LOG_SUCCESS("Completed: Primitive5 FSM");
                }
            }
        } 
    }

}

double Primitive5::get_distance()
{
    return disBetHands;
}

}
