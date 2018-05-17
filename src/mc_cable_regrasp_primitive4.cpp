#include <iostream>

#include "mc_cable_regrasp_primitive4.h"

namespace mc_control
{

Primitive4::Primitive4()
{
}

Primitive4::Primitive4(int primitiveID, std::string primitiveDes, MCCableRegraspController & ctl)
    : BasicPrimitive(primitiveID, primitiveDes)
{
    // For test.
    //std::cout << "Primitive4 construction: " << primitiveID << " " <<  primitiveDes << std::endl; 
    ctl.neglectFctInp = ctl.neglectFctInp;

    // Reset.
    reset();
}

Primitive4::~Primitive4()
{
    // For test.
    //std::cout << "Primitive4 destroy!" << std::endl;
}

void Primitive4::reset()
{
    stepByStep = false;
    paused = false;
    finish = false;

    // Create Primitive4.
    step = new Prim4InitStep();
}

void Primitive4::prim4_fsm_run(MCCableRegraspController & ctl)
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
                    LOG_SUCCESS("Completed: Primitive4 FSM");
                }
            }
        } 
    }
}

}
