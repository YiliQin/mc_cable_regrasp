#include "mc_cable_regrasp_primitive2_fsm.h"
#include "mc_cable_regrasp_primitive2.h"

namespace mc_control
{

Prim2Step::Prim2Step(const std::string & name)
    : name(name)
{
    //std::cout << "Prim2Step Constructed." << std::endl;
}

Prim2Step * Prim2Step::update(MCCableRegraspController & ctl)
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
//  Primitive2 Initialization Step
/////////////////////////////////////////////////////////////

void Prim2InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive2: InitSetp: __init()."<< std::endl;

    ctl.prim2->set_stepByStep(stepByStep_);
}

Prim2Step * Prim2InitStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive2: Prim2InitStep: __update()." << std::endl;
    //return this;
    ctl.neglectFctInp = ctl.neglectFctInp;

    //return this;
    return new Prim2OpenGripperStep;
}

/////////////////////////////////////////////////////////////
//  Primitive2 Open Gripper Step
/////////////////////////////////////////////////////////////

void Prim2OpenGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive2: Prim2OpenGripperStep: __init()." << std::endl;

    ctl.prim2->set_stepByStep(stepByStep_);
}

Prim2Step * Prim2OpenGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive2: Prim2OpenGripperStep: __update()." << std::endl;

    // Loose left gripper.
    auto gripper = ctl.grippers["l_gripper"].get();
    gripper->setTargetQ({0.0});
    // Fixed right gripper.        
    gripper = ctl.grippers["r_gripper"].get();
    gripper->setTargetQ({-0.5});

    // Wait.
    static int wait = 0;
    wait++;
    if (wait == 200)
    {
        wait = 0;
        return new Prim2SpreadStep;
    }
    else 
    {
        return this;
    }
}

/////////////////////////////////////////////////////////////
//  Primitive2 Spread Step
/////////////////////////////////////////////////////////////

void Prim2SpreadStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive2: Prim2SpreadStep: __init()." << std::endl;

    ctl.prim2->set_stepByStep(stepByStep_);
}

Prim2Step * Prim2SpreadStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive2: Prim2SpreadStep: __update()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    // Left hand.
    Eigen::Vector3d leftDiff;
    leftDiff << 0.0, ctl.prim2->get_slideLen()/2, 0.0;
    sva::PTransformd leftGripper;
    leftGripper = ctl.lh2Task->get_ef_pose();
    Eigen::Matrix3d leftRot;
    leftRot = leftGripper.rotation();
    Eigen::Vector3d leftTrans;
    leftTrans = leftGripper.translation() + leftDiff;
    ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot, leftTrans));
    
    // Right hand.
    Eigen::Vector3d rightDiff;
    rightDiff << 0.0, -(ctl.prim2->get_slideLen())/2, 0.0;
    sva::PTransformd rightGripper;
    rightGripper = ctl.rh2Task->get_ef_pose();
    Eigen::Matrix3d rightRot;
    rightRot = rightGripper.rotation();
    Eigen::Vector3d rightTrans;
    rightTrans = rightGripper.translation() + rightDiff;
    ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot, rightTrans));

    return new Prim2CloseGripperStep;
}

/////////////////////////////////////////////////////////////
//  Primitive2 Close Gripper Step
/////////////////////////////////////////////////////////////

void Prim2CloseGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive2: Prim2CloseGripperStep: __init()." << std::endl;

    ctl.prim2->set_stepByStep(stepByStep_);
}

Prim2Step * Prim2CloseGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive2: Prim2CloseGripperStep: __update()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    // Wait.
    double diff;    
    diff = ctl.rh2Task->eval().norm();
    if (diff <= 1e-2)
    {    
        return new Prim2InitPoseStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive2 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim2InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive2: Prim2SecondStep: __init()." << std::endl;
    ctl.prim2->set_stepByStep(stepByStep_);
}

Prim2Step * Prim2InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive2: Prim2SecondStep: __update()." << std::endl;
    ctl.neglectFctInp = ctl.neglectFctInp;

    return new Prim2EndStep;
}

/////////////////////////////////////////////////////////////
//  Primitive2 End Step
/////////////////////////////////////////////////////////////

void Prim2EndStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive2: Prim2EndStep: init." << std::endl;
    ctl.neglectFctInp = ctl.neglectFctInp;
}

Prim2Step * Prim2EndStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive2: Prim2EndStep: update." << std::endl;
    ctl.neglectFctInp = ctl.neglectFctInp;

    //ctl.prim2->finish = true; 
    return nullptr;
}

}
