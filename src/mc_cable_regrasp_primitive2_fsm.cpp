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

Prim2Step * Prim2InitStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive2: Prim2InitStep: __update()." << std::endl;

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

    // Loose left gripper.
    auto gripper = ctl.grippers["l_gripper"].get();
    gripper->setTargetQ({0.0});
    // Fixed right gripper.        
    gripper = ctl.grippers["r_gripper"].get();
    gripper->setTargetQ({-0.5});

}

Prim2Step * Prim2OpenGripperStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive2: Prim2OpenGripperStep: __update()." << std::endl;

    // Wait.
    static int wait = 0;
    wait++;
    if (wait == 500)
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
    // left gripper
    sva::PTransformd leftGripper;
    leftGripper = ctl.lh2Task->get_ef_pose();
    Eigen::Vector3d startPosLeft;
    startPosLeft = leftGripper.translation();
    Eigen::Matrix3d startRotLeft;
    startRotLeft = leftGripper.rotation();
    Eigen::Vector3d leftDiff;
    leftDiff << 0.0, ctl.prim2->get_slideLen()/2, 0.0;
    Eigen::Vector3d endPosLeft;
    endPosLeft = startPosLeft + leftDiff;
    Eigen::Matrix3d endRotLeft;
    endRotLeft = startRotLeft;
    leftHandLinearTraj = new LinearTrajectory(startPosLeft, endPosLeft, startRotLeft, endRotLeft, nr_points_traj);
    // right gripper
    sva::PTransformd rightGripper;
    rightGripper = ctl.rh2Task->get_ef_pose();
    Eigen::Vector3d startPosRight;
    startPosRight = rightGripper.translation();
    Eigen::Matrix3d startRotRight;
    startRotRight = rightGripper.rotation();
    Eigen::Vector3d rightDiff;
    rightDiff << 0.0, -(ctl.prim2->get_slideLen()/2), 0.0;
    Eigen::Vector3d endPosRight;
    endPosRight = startPosRight + rightDiff;
    Eigen::Matrix3d endRotRight;
    endRotRight = startRotRight;
    rightHandLinearTraj = new LinearTrajectory(startPosRight, endPosRight, startRotRight, endRotRight, nr_points_traj);
}

Prim2Step * Prim2SpreadStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim2SpreadStep: __update()." << std::endl;

    Eigen::Vector3d vzero = Eigen::Vector3d::Zero();
    Eigen::Matrix3d mide = Eigen::Matrix3d::Identity();
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> tt(vzero, vzero, vzero, mide); 
    if (cntRun >= nr_points_traj)
    {
        cntRun = 0;
        delete leftHandLinearTraj;
        delete rightHandLinearTraj;
        return new Prim2CloseGripperStep;
    }
    else
    {
        // left gripper
        tt = leftHandLinearTraj->pop();
        Eigen::Vector3d leftPos;
        leftPos = std::get<0>(tt);
        Eigen::Vector3d leftVel;
        leftVel = std::get<1>(tt);
        //ctl.lh2Task->positionTask->refVel(leftVel);
        Eigen::Vector3d leftAccel;
        leftAccel = std::get<2>(tt);
        //ctl.lh2Task->positionTask->refAccel(leftAccel)
        Eigen::Matrix3d leftRot;
        leftRot = std::get<3>(tt);
        ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot, leftPos));
        // right gripper
        tt = rightHandLinearTraj->pop();
        Eigen::Vector3d rightPos;
        rightPos = std::get<0>(tt);
        Eigen::Vector3d rightVel;
        rightVel = std::get<1>(tt);
        //ctl.rh2Task->positionTask->refVel(rightVel);
        Eigen::Vector3d rightAccel;
        rightAccel = std::get<2>(tt);
        //ctl.rh2Task->positionTask->refAccel(rightAccel);
        Eigen::Matrix3d rightRot;
        rightRot = std::get<3>(tt);
        ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot, rightPos));
        //
        cntRun++;
        return this;
    }
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

    // Wait.
    double diffLeft;    
    diffLeft = ctl.lh2Task->eval().norm();
    double diffRight;
    diffRight = ctl.rh2Task->eval().norm();
    if ((diffLeft <= 1e-2) && (diffRight <= 1e-2))
    {
        static bool gripper_changed = false;
        if (gripper_changed == false)
        {
            gripper_changed = true;
            // close left gripper
            auto gripper = ctl.grippers["l_gripper"].get();
            gripper->setTargetQ({-0.5});
            // close right gripper        
            gripper = ctl.grippers["r_gripper"].get();
            gripper->setTargetQ({-0.5});
        }
        static int wait = 0;
        wait++;
        if (wait == 500)
        {
            wait = 0;
            gripper_changed = false;
            return new Prim2InitPoseStep;
        }
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

Prim2Step * Prim2InitPoseStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive2: Prim2SecondStep: __update()." << std::endl;

    return new Prim2EndStep;
}

/////////////////////////////////////////////////////////////
//  Primitive2 End Step
/////////////////////////////////////////////////////////////

void Prim2EndStep::__init(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive2: Prim2EndStep: init." << std::endl;
}

Prim2Step * Prim2EndStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive2: Prim2EndStep: update." << std::endl;

    return nullptr;
}

}
