#include "mc_cable_regrasp_primitive5_fsm.h"
#include "mc_cable_regrasp_primitive5.h"

namespace mc_control
{

Prim5Step::Prim5Step(const std::string & name)
    : name(name)
{
    std::cout << "Prim5Step Constructed." << std::endl;
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
    ctl.fsmtest = 1;
    //return this;
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
    
    // bar local frame - lef hand, right hand pos 
    Eigen::Vector3d lfLeftHand;
    lfLeftHand << 0, 0.3, 0.4;
    Eigen::Vector3d lfRightHand;
    lfRightHand << 0, -0.3, 0.4;
    // cal bar local frame - left hand orientation
    Eigen::Matrix3d lfLeftHandOri;
    lfLeftHandOri << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    // cal bar local frame - right hand orientation
    Eigen::Matrix3d lfRightHandOri;
    lfRightHandOri << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    // For test.
    Eigen::Matrix3d srcOri;
    srcOri << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::Vector3d srcPos;
    srcPos << 0.3, 0.0, 0.5;
    // call move_hands_to_obj
    ctl.move_hands_to_obj(srcOri, srcPos, lfLeftHandOri, lfLeftHand, lfRightHandOri, lfRightHand);

    static bool delay = true;
    if (delay == false)
    {
        // Open Gripper.
        double diff;    
        diff = ctl.rh2Task->eval().norm();
        if (diff <= 1e-2)
        {    
            // Open left gripper.
            auto gripper = ctl.grippers["l_gripper"].get();
            gripper->setTargetQ({0.5});
            // Open right gripper.        
            gripper = ctl.grippers["r_gripper"].get();
            gripper->setTargetQ({0.5});

            return new Prim5GraspStep;
        }
    }
    delay = false;
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

    // bar local frame - lef hand, right hand pos 
    Eigen::Vector3d lfLeftHand;
    lfLeftHand << 0, 0.3, 0.2;
    Eigen::Vector3d lfRightHand;
    lfRightHand << 0, -0.3, 0.2;
    // cal bar local frame - left hand orientation
    Eigen::Matrix3d lfLeftHandOri;
    lfLeftHandOri << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    Eigen::Matrix3d lfRightHandOri;
    lfRightHandOri << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    // For test.
    Eigen::Matrix3d srcOri;
    srcOri << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::Vector3d srcPos;
    srcPos << 0.3, 0.0, 0.5;
    ctl.move_hands_to_obj(srcOri, srcPos, lfLeftHandOri, lfLeftHand, lfRightHandOri, lfRightHand);   

    // Close gripper.
    static bool delay = true;
    if (delay == false)
    {
        double diff;    
        diff = ctl.rh2Task->eval().norm();
        if (diff <= 1e-2)
        {    
            // Close left gripper.
            auto gripper = ctl.grippers["l_gripper"].get();
            gripper->setTargetQ({-0.5});
            // Close right gripper.        
            gripper = ctl.grippers["r_gripper"].get();
            gripper->setTargetQ({-0.5});
            return new Prim5HangStep;
        }
    }
    delay = false;
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive5 Hang Step
/////////////////////////////////////////////////////////////

void Prim5HangStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    std::cout << "Primitive5: Prim5HangStep: __init()." << std::endl;
    ctl.prim5->stepByStep = stepByStep_;
}

Prim5Step * Prim5HangStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim5HangStep: __update()." << std::endl;
    Eigen::Matrix3d rot;
    rot << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::Vector3d t;     
    t << 0.0320, 0.0, 1.122;
    ctl.chestTask->set_ef_pose(sva::PTransformd(rot.inverse(), t));
    
    Eigen::Matrix3d midRot1;
    midRot1 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::Vector3d midPos1;
    midPos1 << 0.2, 0.0, 1.0;
    ctl.move_rigid_obj(midRot1, midPos1, 0.3);
    //
    static bool delay = true;
    double diff;
    diff = ctl.rh2Task->eval().norm();
    if (delay == false)
    {
        if (diff <= 1e-2)
        {
            return new Prim5InitPoseStep;
        }
    }
    delay = false;
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive5 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim5InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    std::cout << "Primitive5: Prim5InitPoseStep: __init()." << std::endl;
    ctl.prim5->stepByStep = stepByStep_;
}

Prim5Step * Prim5InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    std::cout << "Primitive5: Prim5InitPoseStep: __update()." << std::endl;
    //return this;
    ctl.fsmtest = 1;
    return new Prim5EndStep;
}

/////////////////////////////////////////////////////////////
//  Primitive5 End Step
/////////////////////////////////////////////////////////////

void Prim5EndStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    std::cout << "Primitive5: Prim5EndStep: init." << std::endl;
    ctl.fsmtest = 1;
}

Prim5Step * Prim5EndStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    std::cout << "Primitive5: Prim5EndStep: update." << std::endl;
    //ctl.fsmtest = 1;

    ctl.prim5->finish = true;
    return nullptr;
}

}
