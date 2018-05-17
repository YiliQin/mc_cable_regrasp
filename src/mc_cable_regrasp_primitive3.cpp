#include <iostream>

#include "mc_cable_regrasp_primitive3.h"

namespace mc_control
{

Primitive3::Primitive3()
{
}

Primitive3::Primitive3(int primitiveID, std::string primitiveDes, MCCableRegraspController & ctl)
    : BasicPrimitive(primitiveID, primitiveDes)
{
    // For test.
    //std::cout << "Primitive3 construction: " << primitiveID << " " <<  primitiveDes << std::endl; 
    ctl.neglectFctInp = ctl.neglectFctInp;

    // Reset.
    reset();
}

Primitive3::~Primitive3()
{
    // For test.
    //std::cout << "Primitive3 destroy!" << std::endl;
}

void Primitive3::reset()
{
    stepByStep = false;
    paused = false;
    finish = false;

    // Create Primitive3.
    step = new Prim3InitStep();
}

void Primitive3::prim3_fsm_run(MCCableRegraspController & ctl)
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
                    LOG_SUCCESS("Completed: Primitive3 FSM");
                }
            }
        } 
    }
}

}
