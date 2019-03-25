#include "mc_cable_regrasp_primitive15_fsm.h"
#include "mc_cable_regrasp_primitive15.h"

namespace mc_control
{

Prim15Step::Prim15Step(const std::string & name)
    : name(name)
{
    //std::cout << "Prim15Step Constructed." << std::endl;
}

Prim15Step * Prim15Step::update(MCCableRegraspController & ctl)
{
    if(first_call)
    {
        __init(ctl);
        first_call = false;
        return this;
    }

    return __update(ctl);
}

/////////////////////////////////////////////////////////////
//  Primitive15 Initialization Step
/////////////////////////////////////////////////////////////

void Prim15InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive15: InitSetp: __init()."<< std::endl;

    ctl.prim15->set_stepByStep(stepByStep_);
}

Prim15Step * Prim15InitStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive15: Prim15InitStep: __update()." << std::endl;

    return new Prim15BaseMoveStep;
}

/////////////////////////////////////////////////////////////
//  Primitive15 Base Move Step
/////////////////////////////////////////////////////////////

void Prim15BaseMoveStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive15: Prim15BaseMoveStep: __init()." << std::endl;

    ctl.prim15->set_stepByStep(stepByStep_);
    //
    LOG_SUCCESS("Stop mc_rtc and connect HMC! Move robot to this position:")
    LOG_SUCCESS("x direction: " << ctl.prim15->get_diff_x());
    LOG_SUCCESS("y direction: " << ctl.prim15->get_diff_y());
}

Prim15Step * Prim15BaseMoveStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive15: Prim15BaseMoveStep: __update()." << std::endl;

    return new Prim15ResponseStep;
}

/////////////////////////////////////////////////////////////
//  Primitive15 Move Response Step
/////////////////////////////////////////////////////////////

void Prim15ResponseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive15: Prim15ResponseStep: __init()." << std::endl;

    ctl.prim15->set_stepByStep(stepByStep_);
}

Prim15Step * Prim15ResponseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive15: Prim15ResponseStep: __update()." << std::endl;

    if (ctl.prim15Continue == true)
    {
        ctl.prim15Continue = false;
        LOG_SUCCESS("Robot arrived desired position! Continue running mc_rtc ... ");
        return new Prim15InitPoseStep;
    }
    else
        return this;
}

/////////////////////////////////////////////////////////////
//  Primitive15 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim15InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive15: Prim15SecondStep: __init()." << std::endl;
    ctl.prim15->set_stepByStep(stepByStep_);
}

Prim15Step * Prim15InitPoseStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive15: Prim15SecondStep: __update()." << std::endl;

    return new Prim15EndStep;
}

/////////////////////////////////////////////////////////////
//  Primitive15 End Step
/////////////////////////////////////////////////////////////

void Prim15EndStep::__init(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive15: Prim15EndStep: init." << std::endl;
}

Prim15Step * Prim15EndStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive15: Prim15EndStep: update." << std::endl;

    return nullptr;
}

}
