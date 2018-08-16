#include "mc_cable_regrasp_primitive12_fsm.h"
#include "mc_cable_regrasp_primitive12.h"

namespace mc_control
{

Prim12Step::Prim12Step(const std::string & name)
    : name(name)
{
    //std::cout << "Prim12Step Constructed." << std::endl;
}

Prim12Step * Prim12Step::update(MCCableRegraspController & ctl)
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
//  Primitive12 Initialization Step
/////////////////////////////////////////////////////////////

void Prim12InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: InitSetp: __init()."<< std::endl;

    ctl.prim12->set_stepByStep(stepByStep_);
}

Prim12Step * Prim12InitStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive12: Prim12InitStep: __update()." << std::endl;

    return new Prim12OpenGripperStep;
}

/////////////////////////////////////////////////////////////
//  Primitive12 Open Gripper Step
/////////////////////////////////////////////////////////////

void Prim12OpenGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: Prim12OpenGripperStep: __init()." << std::endl;

    ctl.prim12->set_stepByStep(stepByStep_);

    // loose left gripper.
    auto gripper = ctl.grippers["l_gripper"].get();
    gripper->setTargetQ({0.0});
    // fix right gripper.        
    gripper = ctl.grippers["r_gripper"].get();
    gripper->setTargetQ({-0.7});
}

Prim12Step * Prim12OpenGripperStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive12: Prim12OpenGripperStep: __update()." << std::endl;

    // Wait.
    static int wait = 0;
    wait++;
    if (wait == 500)
    {
        wait = 0;
        return new Prim12SpreadStep;
    }
    else 
    {
        return this;
    }
}

/////////////////////////////////////////////////////////////
//  Primitive12 Spread Step
/////////////////////////////////////////////////////////////

void Prim12SpreadStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: Prim12SpreadStep: __init()." << std::endl;
    //
    auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
    auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
    auto X_lf_rf = X_0_rf * (X_0_lf.inv());
    X_lf_rf.translation() = X_lf_rf.translation() / 2;
    auto X_0_mid = X_lf_rf * X_0_lf;
    //
    ctl.prim12->set_stepByStep(stepByStep_);
    // left gripper
    sva::PTransformd leftGripper;
    leftGripper = ctl.lh2Task->get_ef_pose() * X_0_mid.inv();
    Eigen::Vector3d startPosLeft;
    startPosLeft = leftGripper.translation();
    Eigen::Matrix3d startRotLeft;
    startRotLeft = leftGripper.rotation();
    Eigen::Vector3d leftDiff;
    leftDiff << 0.0, ctl.prim12->get_slideLen(), 0.0;
    // different trajectory
    Eigen::Vector3d endPosLeft;
    if (ctl.prim12->get_spreadType() == 1)
        endPosLeft = startPosLeft + leftDiff;
    else if (ctl.prim12->get_spreadType() == 2)
        //endPosLeft << 0, 0.4, startPosLeft[2]; 
        endPosLeft << 0.05, 0.5, startPosLeft[2]; 
    else
        endPosLeft = endPosLeft;
    //
    Eigen::Matrix3d endRotLeft;
    endRotLeft = startRotLeft;
    leftHandLinearTraj = new LinearTrajectory(startPosLeft, endPosLeft, startRotLeft, endRotLeft, nr_points_traj);

}

Prim12Step * Prim12SpreadStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: Prim12SpreadStep: __update()." << std::endl;

    Eigen::Vector3d vzero = Eigen::Vector3d::Zero();
    Eigen::Matrix3d mide = Eigen::Matrix3d::Identity();
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> tt(vzero, vzero, vzero, mide); 
    if (cntRun >= nr_points_traj)
    {
        cntRun = 0;
        delete leftHandLinearTraj;
        //delete rightHandLinearTraj;
        return new Prim12CloseGripperStep;
    }
    else
    {
        //
        auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
        auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
        auto X_lf_rf = X_0_rf * (X_0_lf.inv());
        X_lf_rf.translation() = X_lf_rf.translation() / 2;
        auto X_0_mid = X_lf_rf * X_0_lf;

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
        ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot, leftPos) * X_0_mid);

        //
        cntRun++;
        return this;
    }
}

/////////////////////////////////////////////////////////////
//  Primitive12 Close Gripper Step
/////////////////////////////////////////////////////////////

void Prim12CloseGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: Prim12CloseGripperStep: __init()." << std::endl;

    ctl.prim12->set_stepByStep(stepByStep_);
}

Prim12Step * Prim12CloseGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: Prim12CloseGripperStep: __update()." << std::endl;

    // Wait.
    double diffLeft;
    diffLeft = ctl.lh2Task->eval().norm();
    if (diffLeft <= 1e-2)
    {    
        static bool gripper_changed = false;
        if (gripper_changed == false)
        {
            gripper_changed = true;
            // close left gripper
            auto gripper = ctl.grippers["l_gripper"].get();
            gripper->setTargetQ({-0.7});
            // close right gripper        
            gripper = ctl.grippers["r_gripper"].get();
            gripper->setTargetQ({-0.7});
        }
        static int wait = 0;
        wait++;
        if (wait == 500) 
        {
            wait = 0;
            gripper_changed = false;
            return new Prim12BackStep;
        }
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive12 Back Step
/////////////////////////////////////////////////////////////

void Prim12BackStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: Prim12BackStep: __init()." << std::endl;
    //
    auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
    auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
    auto X_lf_rf = X_0_rf * (X_0_lf.inv());
    X_lf_rf.translation() = X_lf_rf.translation() / 2;
    auto X_0_mid = X_lf_rf * X_0_lf;
    //
    ctl.prim12->set_stepByStep(stepByStep_);
    // left gripper
    sva::PTransformd leftGripper;
    leftGripper = ctl.lh2Task->get_ef_pose() * X_0_mid.inv();
    Eigen::Vector3d startPosLeft;
    startPosLeft = leftGripper.translation();
    Eigen::Matrix3d startRotLeft;
    startRotLeft = leftGripper.rotation();
    Eigen::Vector3d leftDiff;
    leftDiff << 0.0, -ctl.prim12->get_slideLen(), 0.0;
    // different trajectory
    Eigen::Vector3d endPosLeft;
    if (ctl.prim12->get_spreadType() == 1)
        endPosLeft = startPosLeft + leftDiff;
    else if (ctl.prim12->get_spreadType() == 2)
        endPosLeft << 0.2, 0.2, startPosLeft[2];
    else
        endPosLeft = endPosLeft;
    //
    Eigen::Matrix3d endRotLeft;
    endRotLeft = startRotLeft;
    leftHandLinearTraj = new LinearTrajectory(startPosLeft, endPosLeft, startRotLeft, endRotLeft, nr_points_traj);

}

Prim12Step * Prim12BackStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: Prim12BackStep: __update()." << std::endl;

    Eigen::Vector3d vzero = Eigen::Vector3d::Zero();
    Eigen::Matrix3d mide = Eigen::Matrix3d::Identity();
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> tt(vzero, vzero, vzero, mide); 
    if (cntRun >= nr_points_traj)
    {
        cntRun = 0;
        delete leftHandLinearTraj;
        //delete rightHandLinearTraj;
        return new Prim12InitPoseStep;
    }
    else
    {
        //
        auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
        auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
        auto X_lf_rf = X_0_rf * (X_0_lf.inv());
        X_lf_rf.translation() = X_lf_rf.translation() / 2;
        auto X_0_mid = X_lf_rf * X_0_lf;

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
        ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot, leftPos) * X_0_mid);

        //
        cntRun++;
        return this;
    }
}

/////////////////////////////////////////////////////////////
//  Primitive12 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim12InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: Prim12SecondStep: __init()." << std::endl;
    ctl.prim12->set_stepByStep(stepByStep_);
}

Prim12Step * Prim12InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: Prim12SecondStep: __update()." << std::endl;

    // Wait.
    double diffRight;
    diffRight = ctl.rh2Task->eval().norm();
    if (diffRight <= 1e-2)
    {    
        static int wait = 0;
        wait++;
        if (wait == 500) 
        {
            wait = 0;
            return new Prim12EndStep;
        }
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive12 End Step
/////////////////////////////////////////////////////////////

void Prim12EndStep::__init(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive12: Prim12EndStep: init." << std::endl;
}

Prim12Step * Prim12EndStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive12: Prim12EndStep: update." << std::endl;

    return nullptr;
}

}
