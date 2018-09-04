#include <iostream>

#include "mc_cable_regrasp_primitive14.h"

namespace mc_control
{

Primitive14::Primitive14()
{
}

Primitive14::Primitive14(int primitiveID, std::string primitiveDes, MCCableRegraspController &)
    : BasicPrimitive(primitiveID, primitiveDes)
{
    // For test.
    //std::cout << "Primitive14 construction: " << primitiveID << " " <<  primitiveDes << std::endl; 

    reset();
}

Primitive14::~Primitive14()
{
    // For test.
    //std::cout << "Primitive14 destroy!" << std::endl;
}

void Primitive14::reset()
{
    // Reset FSM signal.
    stepByStep = false;
    paused = false;
    finish = false;
}

void Primitive14::prim_config(double par1, double par2, MCCableRegraspController &)
{
    // Set parameter1.
    dis_x = par1;
    dis_y = par2;

    step = new Prim14InitStep();
}

void Primitive14::prim_fsm_run(MCCableRegraspController & ctl)
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
                    LOG_SUCCESS("Completed: Primitive14 FSM");
                }
            }
        } 
    }

}

double Primitive14::get_dis_x()
{
    return dis_x;
}

double Primitive14::get_dis_y()
{
    return dis_y;
}

}
