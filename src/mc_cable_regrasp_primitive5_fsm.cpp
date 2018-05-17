#include "mc_cable_regrasp_primitive5_fsm.h"
#include "mc_cable_regrasp_primitive5.h"

namespace mc_control
{

Prim5Step::Prim5Step(const std::string & name)
    : name(name)
{
    //std::cout << "Prim5Step Constructed." << std::endl;
}

Prim5Step * Prim5Step::update(MCCableRegraspController & ctl)
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
//  Primitive5 Initialization Step
/////////////////////////////////////////////////////////////

void Prim5InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5InitSetp: __init()."<< std::endl;

    ctl.prim5->stepByStep = stepByStep_;
}

Prim5Step * Prim5InitStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5InitStep: __update()." << std::endl;
    ctl.neglectFctInp = ctl.neglectFctInp;

    return new Prim5PreGraspStep;
}

/////////////////////////////////////////////////////////////
//  Primitive5 Prepare Grasp Step
/////////////////////////////////////////////////////////////

void Prim5PreGraspStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5PreGraspStep: __init()." << std::endl;

    ctl.prim5->stepByStep = stepByStep_;
}

Prim5Step * Prim5PreGraspStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5PreGraspStep: __update()." << std::endl;

    // move chest
    Eigen::Matrix3d rot;
    rot << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::Vector3d t;     
    t << 0.0320, 0.0, 1.0;
    ctl.chestTask->set_ef_pose(sva::PTransformd(rot.inverse(), t));
    
    // Left gripper.
    Eigen::Matrix3d leftRot;
    // rotz(-90)
    leftRot << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    Eigen::Vector3d leftPos;
    leftPos << 0.3, ctl.prim5->disBetHands/2, 0.9;
    // Right gripper.
    Eigen::Matrix3d rightRot;
    // rotz(90)
    rightRot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    Eigen::Vector3d rightPos;
    rightPos << 0.3, -(ctl.prim5->disBetHands/2), 0.9;
    //
    ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot.inverse(), leftPos));
    ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot.inverse(), rightPos));

    return new Prim5OpenGripperStep;
}

/////////////////////////////////////////////////////////////
//  Primitive5 Open Gripper Step
/////////////////////////////////////////////////////////////

void Prim5OpenGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    std::cout << "Primitive5: Prim5OpenGripperStep: __init()." << std::endl;

    ctl.prim5->stepByStep = stepByStep_;
}

Prim5Step * Prim5OpenGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5OpenGripperStep: __update()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    double diff;
    diff = ctl.rh2Task->eval().norm();
    if (diff < 1e-2)
    {
        // Open left gripper.
        auto gripper = ctl.grippers["l_gripper"].get();
        gripper->setTargetQ({0.5});
        // Open right gripper.        
        gripper = ctl.grippers["r_gripper"].get();
        gripper->setTargetQ({0.5});

        return new Prim5GraspStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive5 Grasp Step
/////////////////////////////////////////////////////////////

void Prim5GraspStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5GraspStep: __init()." << std::endl;

    ctl.prim5->stepByStep = stepByStep_;
}

Prim5Step * Prim5GraspStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5GraspStep: __update()." << std::endl;
    
    double diff;
    diff = ctl.rh2Task->eval().norm();
    if (diff < 1e-2)
    {
        // Left gripper.
        Eigen::Matrix3d leftRot;
        // rotz(-90)
        leftRot << 0, 1, 0, -1, 0, 0, 0, 0, 1;
        Eigen::Vector3d leftPos;
        leftPos << 0.3, ctl.prim5->disBetHands/2, 0.7;
        // Right gripper.
        Eigen::Matrix3d rightRot;
        // rotz(90)
        rightRot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
        Eigen::Vector3d rightPos;
        rightPos << 0.3, -(ctl.prim5->disBetHands/2), 0.7;
        //
        ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot.inverse(), leftPos));
        ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot.inverse(), rightPos));

        return new Prim5CloseGripperStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive5 Close Gripper Step
/////////////////////////////////////////////////////////////

void Prim5CloseGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5CloseGripperStep: __init()." << std::endl;

    ctl.prim5->stepByStep = stepByStep_;
}

Prim5Step * Prim5CloseGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5CloseGripperStep: __update()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    double diff;
    diff = ctl.rh2Task->eval().norm();
    if (diff < 1e-2)
    {
        // Close left gripper.
        auto gripper = ctl.grippers["l_gripper"].get();
        gripper->setTargetQ({-0.5});
        // Close right gripper.        
        gripper = ctl.grippers["r_gripper"].get();
        gripper->setTargetQ({-0.5});

        return new Prim5HangStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive5 Hang Step
/////////////////////////////////////////////////////////////

void Prim5HangStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5HangStep: __init()." << std::endl;

    ctl.prim5->stepByStep = stepByStep_;
}

Prim5Step * Prim5HangStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5HangStep: __update()." << std::endl;

    double diff;    
    diff = ctl.rh2Task->eval().norm();
    if (diff <= 1e-2)
    {   
        // Cheset task.
        Eigen::Matrix3d rot;
        rot << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        Eigen::Vector3d t;     
        t << 0.0320, 0.0, 1.122;
        ctl.chestTask->set_ef_pose(sva::PTransformd(rot.inverse(), t));

        // Left gripper.
        Eigen::Matrix3d leftRot;
        // rotz(-90)
        leftRot << 0, 1, 0, -1, 0, 0, 0, 0, 1;
        Eigen::Vector3d leftPos;
        leftPos << 0.20, ctl.prim5->disBetHands/2, 1.0;
        // Right gripper.
        Eigen::Matrix3d rightRot;
        // rotz(90)
        rightRot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
        Eigen::Vector3d rightPos;
        rightPos << 0.20, -(ctl.prim5->disBetHands/2), 1.0;
        //
        ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot.inverse(), leftPos));
        ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot.inverse(), rightPos));

        return new Prim5InitPoseStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive5 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim5InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5InitPoseStep: __init()." << std::endl;

    ctl.prim5->stepByStep = stepByStep_;
}

Prim5Step * Prim5InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5InitPoseStep: __update()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    double diff;
    diff = ctl.rh2Task->eval().norm();
    if (diff < 1e-2)
    {
        return new Prim5EndStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive5 End Step
/////////////////////////////////////////////////////////////

void Prim5EndStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5EndStep: init." << std::endl;

    ctl.neglectFctInp = ctl.neglectFctInp;
}

Prim5Step * Prim5EndStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5EndStep: update." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    ctl.prim5->finish = true;
    return nullptr;
}

}
