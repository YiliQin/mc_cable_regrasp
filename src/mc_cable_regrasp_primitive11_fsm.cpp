#include "mc_cable_regrasp_primitive11_fsm.h"
#include "mc_cable_regrasp_primitive11.h"

namespace mc_control
{

Prim11Step::Prim11Step(const std::string & name)
    : name(name)
{
    //std::cout << "Prim11Step Constructed." << std::endl;
}

Prim11Step * Prim11Step::update(MCCableRegraspController & ctl)
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
//  Primitive11 Initialization Step
/////////////////////////////////////////////////////////////

void Prim11InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive11: InitSetp: __init()."<< std::endl;

    ctl.prim11->set_stepByStep(stepByStep_);
}

Prim11Step * Prim11InitStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive11: Prim11InitStep: __update()." << std::endl;

    return new Prim11OpenGripperStep;
}

/////////////////////////////////////////////////////////////
//  Primitive11 Open Gripper Step
/////////////////////////////////////////////////////////////

void Prim11OpenGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive11: Prim11OpenGripperStep: __init()." << std::endl;

    ctl.prim11->set_stepByStep(stepByStep_);

    // Fxied left gripper.
    auto gripper = ctl.grippers["l_gripper"].get();
    gripper->setTargetQ({-0.7});
    // Loose right gripper.        
    gripper = ctl.grippers["r_gripper"].get();
    gripper->setTargetQ({0.0});
}

Prim11Step * Prim11OpenGripperStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive11: Prim11OpenGripperStep: __update()." << std::endl;

    // Wait.
    static int wait = 0;
    wait++;
    if (wait == 500)
    {
        wait = 0;
        return new Prim11SpreadStep;
    }
    else 
    {
        return this;
    }
}

/////////////////////////////////////////////////////////////
//  Primitive11 Spread Step
/////////////////////////////////////////////////////////////

void Prim11SpreadStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive11: Prim11SpreadStep: __init()." << std::endl;
    //
    auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
    auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
    auto X_lf_rf = X_0_rf * (X_0_lf.inv());
    X_lf_rf.translation() = X_lf_rf.translation() / 2;
    auto X_0_mid = X_lf_rf * X_0_lf;
    //
    ctl.prim11->set_stepByStep(stepByStep_);
    //// left gripper
    //sva::PTransformd leftGripper;
    //leftGripper = ctl.lh2Task->get_ef_pose() * X_0_mid.inv();
    //Eigen::Vector3d startPosLeft;
    //startPosLeft = leftGripper.translation();
    //Eigen::Matrix3d startRotLeft;
    //startRotLeft = leftGripper.rotation();
    //Eigen::Vector3d leftDiff;
    //leftDiff << 0.0, ctl.prim11->get_slideLen()/2, 0.0;
    //Eigen::Vector3d endPosLeft;
    //endPosLeft = startPosLeft + leftDiff;
    //Eigen::Matrix3d endRotLeft;
    //endRotLeft = startRotLeft;
    //leftHandLinearTraj = new LinearTrajectory(startPosLeft, endPosLeft, startRotLeft, endRotLeft, nr_points_traj);

    // right gripper
    sva::PTransformd rightGripper;
    rightGripper = ctl.rh2Task->get_ef_pose() * X_0_mid.inv();
    Eigen::Vector3d startPosRight;
    startPosRight = rightGripper.translation();
    Eigen::Matrix3d startRotRight;
    startRotRight = rightGripper.rotation();
    Eigen::Vector3d rightDiff;
    rightDiff << 0.0, -ctl.prim11->get_slideLen(), 0.0;
    Eigen::Vector3d endPosRight;
    endPosRight = startPosRight + rightDiff;
    Eigen::Matrix3d endRotRight;
    endRotRight = startRotRight;
    rightHandLinearTraj = new LinearTrajectory(startPosRight, endPosRight, startRotRight, endRotRight, nr_points_traj);
}

Prim11Step * Prim11SpreadStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive11: Prim11SpreadStep: __update()." << std::endl;

    Eigen::Vector3d vzero = Eigen::Vector3d::Zero();
    Eigen::Matrix3d mide = Eigen::Matrix3d::Identity();
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> tt(vzero, vzero, vzero, mide); 
    if (cntRun >= nr_points_traj)
    {
        cntRun = 0;
        //delete leftHandLinearTraj;
        delete rightHandLinearTraj;
        return new Prim11CloseGripperStep;
    }
    else
    {
        //
        auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
        auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
        auto X_lf_rf = X_0_rf * (X_0_lf.inv());
        X_lf_rf.translation() = X_lf_rf.translation() / 2;
        auto X_0_mid = X_lf_rf * X_0_lf;

        //// left gripper
        //tt = leftHandLinearTraj->pop();
        //Eigen::Vector3d leftPos;
        //leftPos = std::get<0>(tt);
        //Eigen::Vector3d leftVel;
        //leftVel = std::get<1>(tt);
        ////ctl.lh2Task->positionTask->refVel(leftVel);
        //Eigen::Vector3d leftAccel;
        //leftAccel = std::get<2>(tt);
        ////ctl.lh2Task->positionTask->refAccel(leftAccel)
        //Eigen::Matrix3d leftRot;
        //leftRot = std::get<3>(tt);
        //ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot, leftPos) * X_0_mid);

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
        ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot, rightPos) * X_0_mid);
        //
        cntRun++;
        return this;
    }
}

/////////////////////////////////////////////////////////////
//  Primitive11 Close Gripper Step
/////////////////////////////////////////////////////////////

void Prim11CloseGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive11: Prim11CloseGripperStep: __init()." << std::endl;

    ctl.prim11->set_stepByStep(stepByStep_);
}

Prim11Step * Prim11CloseGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive11: Prim11CloseGripperStep: __update()." << std::endl;

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
            return new Prim11BackStep;
        }
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive11 Back Step
/////////////////////////////////////////////////////////////

void Prim11BackStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive11: Prim11BackStep: __init()." << std::endl;
    //
    auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
    auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
    auto X_lf_rf = X_0_rf * (X_0_lf.inv());
    X_lf_rf.translation() = X_lf_rf.translation() / 2;
    auto X_0_mid = X_lf_rf * X_0_lf;
    //
    ctl.prim11->set_stepByStep(stepByStep_);
    //// left gripper
    //sva::PTransformd leftGripper;
    //leftGripper = ctl.lh2Task->get_ef_pose() * X_0_mid.inv();
    //Eigen::Vector3d startPosLeft;
    //startPosLeft = leftGripper.translation();
    //Eigen::Matrix3d startRotLeft;
    //startRotLeft = leftGripper.rotation();
    //Eigen::Vector3d leftDiff;
    //leftDiff << 0.0, ctl.prim11->get_slideLen()/2, 0.0;
    //Eigen::Vector3d endPosLeft;
    //endPosLeft = startPosLeft + leftDiff;
    //Eigen::Matrix3d endRotLeft;
    //endRotLeft = startRotLeft;
    //leftHandLinearTraj = new LinearTrajectory(startPosLeft, endPosLeft, startRotLeft, endRotLeft, nr_points_traj);

    // right gripper
    sva::PTransformd rightGripper;
    rightGripper = ctl.rh2Task->get_ef_pose() * X_0_mid.inv();
    Eigen::Vector3d startPosRight;
    startPosRight = rightGripper.translation();
    Eigen::Matrix3d startRotRight;
    startRotRight = rightGripper.rotation();
    Eigen::Vector3d rightDiff;
    rightDiff << 0.0, ctl.prim11->get_slideLen(), 0.0;
    Eigen::Vector3d endPosRight;
    endPosRight = startPosRight + rightDiff;
    Eigen::Matrix3d endRotRight;
    endRotRight = startRotRight;
    rightHandLinearTraj = new LinearTrajectory(startPosRight, endPosRight, startRotRight, endRotRight, nr_points_traj);
}

Prim11Step * Prim11BackStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive11: Prim11BackStep: __update()." << std::endl;

    Eigen::Vector3d vzero = Eigen::Vector3d::Zero();
    Eigen::Matrix3d mide = Eigen::Matrix3d::Identity();
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> tt(vzero, vzero, vzero, mide); 
    if (cntRun >= nr_points_traj)
    {
        cntRun = 0;
        //delete leftHandLinearTraj;
        delete rightHandLinearTraj;
        return new Prim11InitPoseStep;
    }
    else
    {
        //
        auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
        auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
        auto X_lf_rf = X_0_rf * (X_0_lf.inv());
        X_lf_rf.translation() = X_lf_rf.translation() / 2;
        auto X_0_mid = X_lf_rf * X_0_lf;

        //// left gripper
        //tt = leftHandLinearTraj->pop();
        //Eigen::Vector3d leftPos;
        //leftPos = std::get<0>(tt);
        //Eigen::Vector3d leftVel;
        //leftVel = std::get<1>(tt);
        ////ctl.lh2Task->positionTask->refVel(leftVel);
        //Eigen::Vector3d leftAccel;
        //leftAccel = std::get<2>(tt);
        ////ctl.lh2Task->positionTask->refAccel(leftAccel)
        //Eigen::Matrix3d leftRot;
        //leftRot = std::get<3>(tt);
        //ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot, leftPos) * X_0_mid);

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
        ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot, rightPos) * X_0_mid);
        //
        cntRun++;
        return this;
    }
}

/////////////////////////////////////////////////////////////
//  Primitive11 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim11InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive11: Prim11SecondStep: __init()." << std::endl;
    ctl.prim11->set_stepByStep(stepByStep_);
}

Prim11Step * Prim11InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive11: Prim11SecondStep: __update()." << std::endl;

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
            return new Prim11EndStep;
        }
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive11 End Step
/////////////////////////////////////////////////////////////

void Prim11EndStep::__init(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive11: Prim11EndStep: init." << std::endl;
}

Prim11Step * Prim11EndStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive11: Prim11EndStep: update." << std::endl;

    return nullptr;
}

}
