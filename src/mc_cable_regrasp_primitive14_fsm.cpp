#include "mc_cable_regrasp_primitive14_fsm.h"
#include "mc_cable_regrasp_primitive14.h"

namespace mc_control
{

Prim14Step::Prim14Step(const std::string & name)
    : name(name)
{
    //std::cout << "Prim14Step Constructed." << std::endl;
}

Prim14Step * Prim14Step::update(MCCableRegraspController & ctl)
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
//  Primitive14 Initialization Step
/////////////////////////////////////////////////////////////

void Prim14InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive14: Prim14InitSetp: __init()."<< std::endl;

    ctl.prim14->set_stepByStep(stepByStep_);
}

Prim14Step * Prim14InitStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive14: Prim14InitStep: __update()." << std::endl;

    return new Prim14PreGraspStep;
}

/////////////////////////////////////////////////////////////
//  Primitive14 Prepare Grasp Step
/////////////////////////////////////////////////////////////

void Prim14PreGraspStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive14: Prim14PreGraspStep: __init()." << std::endl;

    ctl.prim14->set_stepByStep(stepByStep_);
}

Prim14Step * Prim14PreGraspStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive14: Prim14PreGraspStep: __update()." << std::endl;

    //
    auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
    auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
    auto X_lf_rf = X_0_rf * (X_0_lf.inv());
    X_lf_rf.translation() = X_lf_rf.translation() / 2;
    auto X_0_mid = X_lf_rf * X_0_lf;
    
    // right gripper
    sva::PTransformd rightGripper;
    rightGripper = ctl.rh2Task->get_ef_pose() * X_0_mid.inv();
    Eigen::Vector3d rightPos;
    rightPos = rightGripper.translation();

    // Left gripper.
    Eigen::Matrix3d leftRot;
    // rotz(-90)
    leftRot << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    Eigen::Vector3d leftPos;
    //leftPos << 0.3, ctl.prim14->get_distance()/2, 1.1;
    leftPos << rightPos[0], ctl.prim14->get_distance(), rightPos[2] + 0.2;
    ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot.inverse(), leftPos) * X_0_mid);

    return new Prim14OpenGripperStep;
}

/////////////////////////////////////////////////////////////
//  Primitive14 Open Gripper Step
/////////////////////////////////////////////////////////////

void Prim14OpenGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive14: Prim14OpenGripperStep: __init()." << std::endl;

    ctl.prim14->set_stepByStep(stepByStep_);
}

Prim14Step * Prim14OpenGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive14: Prim14OpenGripperStep: __update()." << std::endl;

    double diffLeft;
    diffLeft = ctl.lh2Task->eval().norm();
    if (diffLeft < 1e-2)
    {
        static bool opened = false;
        if (opened == false)
        {
            opened = true;
            // Open left gripper.
            auto gripper = ctl.grippers["l_gripper"].get();
            gripper->setTargetQ({0.5});
        }
        static int wait = 0;
        wait++;
        if (wait == 500)
        {
            wait = 0;
            opened = false;  
            return new Prim14GraspStep;
        }
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive14 Grasp Step
/////////////////////////////////////////////////////////////

void Prim14GraspStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive14: Prim14GraspStep: __init()." << std::endl;

    ctl.prim14->set_stepByStep(stepByStep_);
}

Prim14Step * Prim14GraspStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive14: Prim14GraspStep: __update()." << std::endl;
    //
    auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
    auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
    auto X_lf_rf = X_0_rf * (X_0_lf.inv());
    X_lf_rf.translation() = X_lf_rf.translation() / 2;
    auto X_0_mid = X_lf_rf * X_0_lf;

    // right gripper
    sva::PTransformd rightGripper;
    rightGripper = ctl.rh2Task->get_ef_pose() * X_0_mid.inv();
    Eigen::Vector3d rightPos;
    rightPos = rightGripper.translation();

    // Left gripper.
    Eigen::Matrix3d leftRot;
    // rotz(-90)
    leftRot << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    Eigen::Vector3d leftPos;
    //leftPos << 0.3, ctl.prim14->get_distance()/2, 1.0;
    leftPos << rightPos[0], ctl.prim14->get_distance(), rightPos[2];
    ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot.inverse(), leftPos) * X_0_mid);

    return new Prim14CloseGripperStep;
}

/////////////////////////////////////////////////////////////
//  Primitive14 Close Gripper Step
/////////////////////////////////////////////////////////////

void Prim14CloseGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive14: Prim14CloseGripperStep: __init()." << std::endl;

    ctl.prim14->set_stepByStep(stepByStep_);
}

Prim14Step * Prim14CloseGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive14: Prim14CloseGripperStep: __update()." << std::endl;

    double diffLeft;
    diffLeft = ctl.lh2Task->eval().norm();
    if (diffLeft < 1e-2)
    {
        static bool closed = false;
        if(closed == false)
        {
          closed = true;
          // Close left gripper.
          auto gripper = ctl.grippers["l_gripper"].get();
          gripper->setTargetQ({-0.5});
        }
        static int wait = 0;
        if(wait++ == 500)
        {
          wait = 0;
          closed = false;
          return new Prim14InitPoseStep;
        }
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive14 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim14InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive14: Prim14InitPoseStep: __init()." << std::endl;

    ctl.prim14->set_stepByStep(stepByStep_);
}

Prim14Step * Prim14InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive14: Prim14InitPoseStep: __update()." << std::endl;

    double diffLeft;
    diffLeft = ctl.lh2Task->eval().norm();
    if (diffLeft < 1e-2)
    {
        return new Prim14EndStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive14 End Step
/////////////////////////////////////////////////////////////

void Prim14EndStep::__init(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive14: Prim14EndStep: init." << std::endl;
}

Prim14Step * Prim14EndStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive14: Prim14EndStep: update." << std::endl;

    return nullptr;
}

}
