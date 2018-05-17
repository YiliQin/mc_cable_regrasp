#include "mc_cable_regrasp_primitive1_fsm.h"
#include "mc_cable_regrasp_primitive1.h"

namespace mc_control
{

Prim1Step::Prim1Step(const std::string & name)
    : name(name)
{
    //std::cout << "Prim1Step Constructed." << std::endl;
}

Prim1Step * Prim1Step::update(MCCableRegraspController & ctl)
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
//  Primitive1 Initialization Step
/////////////////////////////////////////////////////////////

void Prim1InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: InitSetp: __init()."<< std::endl;

    ctl.prim1->stepByStep = stepByStep_;
}

Prim1Step * Prim1InitStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1InitStep: __update()." << std::endl;
    //return this;
    ctl.neglectFctInp = ctl.neglectFctInp;

    //return this;
    return new Prim1OpenGripperStep;
}

/////////////////////////////////////////////////////////////
//  Primitive1 Open Gripper Step
/////////////////////////////////////////////////////////////

void Prim1OpenGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1OpenGripperStep: __init()." << std::endl;

    ctl.prim1->stepByStep = stepByStep_;
}

Prim1Step * Prim1OpenGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1OpenGripperStep: __update()." << std::endl;

    // Fxied left gripper.
    auto gripper = ctl.grippers["l_gripper"].get();
    gripper->setTargetQ({-0.5});
    // Loose right gripper.        
    gripper = ctl.grippers["r_gripper"].get();
    gripper->setTargetQ({0.0});

    // Wait.
    static int wait = 0;
    wait++;
    if (wait == 200)
    {
        wait = 0;
        return new Prim1SpreadStep;
    }
    else 
    {
        return this;
    }
}

/////////////////////////////////////////////////////////////
//  Primitive1 Spread Step
/////////////////////////////////////////////////////////////

void Prim1SpreadStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1SpreadStep: __init()." << std::endl;

    ctl.prim1->stepByStep = stepByStep_;
}

Prim1Step * Prim1SpreadStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1SpreadStep: __update()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    // Left hand.
    Eigen::Vector3d leftDiff;
    leftDiff << 0.0, ctl.prim1->slideLen/2, 0.0;
    sva::PTransformd leftGripper;
    leftGripper = ctl.lh2Task->get_ef_pose();
    Eigen::Matrix3d leftRot;
    leftRot = leftGripper.rotation();
    Eigen::Vector3d leftTrans;
    leftTrans = leftGripper.translation() + leftDiff;
    ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot, leftTrans));
    
    // Right hand.
    Eigen::Vector3d rightDiff;
    rightDiff << 0.0, -ctl.prim1->slideLen/2, 0.0;
    sva::PTransformd rightGripper;
    rightGripper = ctl.rh2Task->get_ef_pose();
    Eigen::Matrix3d rightRot;
    rightRot = rightGripper.rotation();
    Eigen::Vector3d rightTrans;
    rightTrans = rightGripper.translation() + rightDiff;
    ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot, rightTrans));

    return new Prim1CloseGripperStep;
}

/////////////////////////////////////////////////////////////
//  Primitive1 Close Gripper Step
/////////////////////////////////////////////////////////////

void Prim1CloseGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1CloseGripperStep: __init()." << std::endl;

    ctl.prim1->stepByStep = stepByStep_;
}

Prim1Step * Prim1CloseGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1CloseGripperStep: __update()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    // Wait.
    double diff;    
    diff = ctl.rh2Task->eval().norm();
    if (diff <= 1e-2)
    {    
        return new Prim1InitPoseStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive1 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim1InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1SecondStep: __init()." << std::endl;
    ctl.prim1->stepByStep = stepByStep_;
}

Prim1Step * Prim1InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1SecondStep: __update()." << std::endl;
    ctl.neglectFctInp = ctl.neglectFctInp;

    return new Prim1EndStep;
}

/////////////////////////////////////////////////////////////
//  Primitive1 End Step
/////////////////////////////////////////////////////////////

void Prim1EndStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1EndStep: init." << std::endl;
    ctl.neglectFctInp = ctl.neglectFctInp;
}

Prim1Step * Prim1EndStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1EndStep: update." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    ctl.prim1->finish = true; 
    return nullptr;
}

}
