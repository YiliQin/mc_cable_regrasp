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
    Eigen::Vector3d endPosLeft;
    endPosLeft = startPosLeft + leftDiff;
    Eigen::Matrix3d endRotLeft;
    endRotLeft = startRotLeft;
    leftHandLinearTraj = new LinearTrajectory(startPosLeft, endPosLeft, startRotLeft, endRotLeft, nr_points_traj);

    //// right gripper
    //sva::PTransformd rightGripper;
    //rightGripper = ctl.rh2Task->get_ef_pose() * X_0_mid.inv();
    //Eigen::Vector3d startPosRight;
    //startPosRight = rightGripper.translation();
    //Eigen::Matrix3d startRotRight;
    //startRotRight = rightGripper.rotation();
    //Eigen::Vector3d rightDiff;
    //rightDiff << 0.0, -ctl.prim12->get_slideLen(), 0.0;
    //Eigen::Vector3d endPosRight;
    //endPosRight = startPosRight + rightDiff;
    //Eigen::Matrix3d endRotRight;
    //endRotRight = startRotRight;
    //rightHandLinearTraj = new LinearTrajectory(startPosRight, endPosRight, startRotRight, endRotRight, nr_points_traj);
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

        // right gripper
        //tt = rightHandLinearTraj->pop();
        //Eigen::Vector3d rightPos;
        //rightPos = std::get<0>(tt);
        //Eigen::Vector3d rightVel;
        //rightVel = std::get<1>(tt);
        ////ctl.rh2Task->positionTask->refVel(rightVel);
        //Eigen::Vector3d rightAccel;
        //rightAccel = std::get<2>(tt);
        ////ctl.rh2Task->positionTask->refAccel(rightAccel);
        //Eigen::Matrix3d rightRot;
        //rightRot = std::get<3>(tt);
        //ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot, rightPos) * X_0_mid);

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
    Eigen::Vector3d endPosLeft;
    endPosLeft = startPosLeft + leftDiff;
    Eigen::Matrix3d endRotLeft;
    endRotLeft = startRotLeft;
    leftHandLinearTraj = new LinearTrajectory(startPosLeft, endPosLeft, startRotLeft, endRotLeft, nr_points_traj);

    //// right gripper
    //sva::PTransformd rightGripper;
    //rightGripper = ctl.rh2Task->get_ef_pose() * X_0_mid.inv();
    //Eigen::Vector3d startPosRight;
    //startPosRight = rightGripper.translation();
    //Eigen::Matrix3d startRotRight;
    //startRotRight = rightGripper.rotation();
    //Eigen::Vector3d rightDiff;
    //rightDiff << 0.0, ctl.prim12->get_slideLen(), 0.0;
    //Eigen::Vector3d endPosRight;
    //endPosRight = startPosRight + rightDiff;
    //Eigen::Matrix3d endRotRight;
    //endRotRight = startRotRight;
    //rightHandLinearTraj = new LinearTrajectory(startPosRight, endPosRight, startRotRight, endRotRight, nr_points_traj);
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
        return new Prim12OpenRightStep;
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

        // right gripper
        //tt = rightHandLinearTraj->pop();
        //Eigen::Vector3d rightPos;
        //rightPos = std::get<0>(tt);
        //Eigen::Vector3d rightVel;
        //rightVel = std::get<1>(tt);
        ////ctl.rh2Task->positionTask->refVel(rightVel);
        //Eigen::Vector3d rightAccel;
        //rightAccel = std::get<2>(tt);
        ////ctl.rh2Task->positionTask->refAccel(rightAccel);
        //Eigen::Matrix3d rightRot;
        //rightRot = std::get<3>(tt);
        //ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot, rightPos) * X_0_mid);

        //
        cntRun++;
        return this;
    }
}

/////////////////////////////////////////////////////////////
//  Primitive12 Open Right Gripper Step
/////////////////////////////////////////////////////////////

void Prim12OpenRightStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: Prim12OpenRightStep: __init()." << std::endl;

    ctl.prim12->set_stepByStep(stepByStep_);
}

Prim12Step * Prim12OpenRightStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: Prim12OpenRightStep: __update()." << std::endl;

    double diffLeft;
    diffLeft = ctl.lh2Task->eval().norm();
    if (diffLeft < 1e-2)
    {
        static bool gripper_changed = false;
        if (gripper_changed == false)
        {
            gripper_changed = true;
            // Fix left gripper.
            auto gripper = ctl.grippers["l_gripper"].get();
            gripper->setTargetQ({-0.7});
            // Loose right gripper.        
            gripper = ctl.grippers["r_gripper"].get();
            gripper->setTargetQ({0.5});
        }
        // Wait.
        static int wait = 0;
        wait++;
        if (wait == 500)
        {
            wait = 0;
            gripper_changed = false;
            return new Prim12ReleRightStep;
        }
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive12 Release Right Gripper Step
/////////////////////////////////////////////////////////////

void Prim12ReleRightStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: Prim12ReleRightStep: __init()." << std::endl;
    ctl.prim12->set_stepByStep(stepByStep_);
}

Prim12Step * Prim12ReleRightStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: Prim12ReleRightStep: __update()." << std::endl;

    //
    auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
    auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
    auto X_lf_rf = X_0_rf * (X_0_lf.inv());
    X_lf_rf.translation() = X_lf_rf.translation() / 2;
    auto X_0_mid = X_lf_rf * X_0_lf;

    // right gripper
    sva::PTransformd rightGripper;
    rightGripper = ctl.rh2Task->get_ef_pose() * X_0_mid.inv();
    Eigen::Vector3d startPosRight;
    startPosRight = rightGripper.translation();
    Eigen::Matrix3d startRotRight;
    startRotRight = rightGripper.rotation();

    Eigen::Vector3d rightDiff;
    rightDiff << 0.0, 0.0, 0.15;
    Eigen::Vector3d endPosRight;
    endPosRight = startPosRight + rightDiff;
    Eigen::Matrix3d endRotRight;
    endRotRight = startRotRight;

    ctl.rh2Task->set_ef_pose(sva::PTransformd(endRotRight, endPosRight) * X_0_mid);

    return new Prim12RightBackStep;
}

/////////////////////////////////////////////////////////////
//  Primitive12 Right Gripper Back Step
/////////////////////////////////////////////////////////////

void Prim12RightBackStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: Prim12RightBackStep: __init()." << std::endl;
    ctl.prim12->set_stepByStep(stepByStep_);
}

Prim12Step * Prim12RightBackStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive12: Prim12RightBackStep: __update()." << std::endl;

    // Wait.
    double diffRight;
    diffRight = ctl.rh2Task->eval().norm();
    if (diffRight <= 1e-2)
    {    
        //
        auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
        auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
        auto X_lf_rf = X_0_rf * (X_0_lf.inv());
        X_lf_rf.translation() = X_lf_rf.translation() / 2;
        auto X_0_mid = X_lf_rf * X_0_lf;

        // left gripper
        Eigen::Matrix3d endRotRight; 
        //endRotRight << 0.9637, 0.0877, -0.2521, -0.1328, 0.9768, -0.1680, 0.2315, 0.1954  ,0.9530;
        endRotRight << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        Eigen::Vector3d endPosRight;
        endPosRight << -0.10, -0.3475, 0.7319; 

        ctl.rh2Task->set_ef_pose(sva::PTransformd(endRotRight, endPosRight) * X_0_mid);

        return new Prim12InitPoseStep;
    }
    return this;
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
