#include <iostream>

#include "mc_cable_regrasp_primitive13.h"

namespace mc_control
{

Primitive13::Primitive13()
{
}

Primitive13::Primitive13(int primitiveID, std::string primitiveDes, MCCableRegraspController &)
    : BasicPrimitive(primitiveID, primitiveDes)
{
    // For test.
    //std::cout << "Primitive13 construction: " << primitiveID << " " <<  primitiveDes << std::endl; 

    reset();
}

Primitive13::~Primitive13()
{
    // For test.
    //std::cout << "Primitive13 destroy!" << std::endl;
}

void Primitive13::reset()
{
    // Reset FSM signal.
    stepByStep = false;
    paused = false;
    finish = false;
}

void Primitive13::prim_config(double par1, double par2, MCCableRegraspController &)
{
    // Set parameter1.
    dis_x = par1;
    dis_y = par2;

    step = new Prim13InitStep();
}

void Primitive13::prim_fsm_run(MCCableRegraspController & ctl)
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
                    LOG_SUCCESS("Completed: Primitive13 FSM");
                }
            }
        } 
    }

}

double Primitive13::get_dis_x()
{
    return dis_x;
}

double Primitive13::get_dis_y()
{
    return dis_y;
}

}
