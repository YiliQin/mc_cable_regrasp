#include "mc_cable_regrasp_primitive4_fsm.h"
#include "mc_cable_regrasp_primitive4.h"

namespace mc_control
{

Prim4Step::Prim4Step(const std::string & name)
    : name(name)
{
    //std::cout << "Prim4Step Constructed." << std::endl;
}

Prim4Step * Prim4Step::update(MCCableRegraspController & ctl)
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
//  Primitive4 Initialization Step
/////////////////////////////////////////////////////////////

void Prim4InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: InitSetp: __init()."<< std::endl;
    
    ctl.prim4->set_stepByStep(stepByStep_);
}

Prim4Step * Prim4InitStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive4: Prim4InitStep: __update()." << std::endl;

    return new Prim4OpenGripperStep;
}

/////////////////////////////////////////////////////////////
//  Primitive4 Open Gripper Step
/////////////////////////////////////////////////////////////

void Prim4OpenGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4OpenGripperStep: __init()." << std::endl;

    ctl.prim4->set_stepByStep(stepByStep_);
    // Open left gripper.
    auto gripper = ctl.grippers["l_gripper"].get();
    gripper->setTargetQ({0.5});
    // Close right gripper.        
    gripper = ctl.grippers["r_gripper"].get();
    gripper->setTargetQ({-0.7});
}

Prim4Step * Prim4OpenGripperStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive4: Prim4OpenGripperStep: __update()." << std::endl;

    // Wait.
    static int wait = 0;
    wait++;
    if (wait == 500)
    {
        wait = 0;
        return new Prim4RightHandFlipStep;
    }
    else
    { 
         return this;
    }
}

/////////////////////////////////////////////////////////////
//  Primitive4 Right Hand Flip Step
/////////////////////////////////////////////////////////////

void Prim4RightHandFlipStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4RightHandFlipStep: __init()." << std::endl;
 
    ctl.prim4->set_stepByStep(stepByStep_);
}

Prim4Step * Prim4RightHandFlipStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4RightHandFlipStep: __update()." << std::endl;

    // 
    auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
    auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
    auto X_lf_rf = X_0_rf * (X_0_lf.inv());
    X_lf_rf.translation() = X_lf_rf.translation() / 2;
    auto X_0_mid = X_lf_rf * X_0_lf;
    //
    sva::PTransformd rightGripper;
    rightGripper = ctl.rh2Task->get_ef_pose() * X_0_mid.inv();
    Eigen::Matrix3d rightRot;
    rightRot = rightGripper.rotation();
    Eigen::Vector3d rightTrans;
    rightTrans = rightGripper.translation();
    // Exchange position.
    Eigen::Vector3d exPos;
    exPos << 0.2, -0.3, 1.0;
    // Rotation
    Eigen::Matrix3d t;
    // rotz(90)*rotx(-90)*rotz(180)
    t << 0, 1, 0, 0, 0, -1, -1, 0, 0;
    ctl.rh2Task->set_ef_pose(sva::PTransformd(t.inverse(), exPos) * X_0_mid);

    return new Prim4ToCenterStep;
}

/////////////////////////////////////////////////////////////
//  Primitive4 To Center Step
/////////////////////////////////////////////////////////////

void Prim4ToCenterStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4ToCenterStep: __init()." << std::endl;

    ctl.prim4->set_stepByStep(stepByStep_);
}

Prim4Step * Prim4ToCenterStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4ToCenterStep: __update()." << std::endl;

    double diff;
    diff = ctl.rh2Task->eval().norm();
    if (diff <= 1e-2)
    { 
        //
        auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
        auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
        auto X_lf_rf = X_0_rf * (X_0_lf.inv());
        X_lf_rf.translation() = X_lf_rf.translation() / 2;
        auto X_0_mid = X_lf_rf * X_0_lf;
        //
        sva::PTransformd rightGripper;
        rightGripper = ctl.rh2Task->get_ef_pose() * X_0_mid.inv();
        Eigen::Matrix3d rightRot;
        rightRot = rightGripper.rotation();
        Eigen::Vector3d rightTrans;
        rightTrans = rightGripper.translation();
        // Exchange position.
        Eigen::Vector3d exPos;
        exPos << 0.25, -0.19, 1.0;
        // Rotation
        ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot, exPos) * X_0_mid);
        return new Prim4RegraspStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive4 Left Hand Regrasp Step
/////////////////////////////////////////////////////////////

void Prim4RegraspStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4RegraspStep: __init()." << std::endl;

    ctl.prim4->set_stepByStep(stepByStep_);
}

Prim4Step * Prim4RegraspStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4RegraspStep: __update()." << std::endl;

    double diff;
    diff = ctl.rh2Task->eval().norm();
    if (diff <= 1e-2)
    { 
        //
        auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
        auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
        auto X_lf_rf = X_0_rf * (X_0_lf.inv());
        X_lf_rf.translation() = X_lf_rf.translation() / 2;
        auto X_0_mid = X_lf_rf * X_0_lf;
        //
        sva::PTransformd leftGripper;
        leftGripper = ctl.lh2Task->get_ef_pose() * X_0_mid.inv();
        Eigen::Matrix3d leftRot;
        leftRot = leftGripper.rotation();
        Eigen::Vector3d leftTrans;
        leftTrans = leftGripper.translation();
        // Exchange position.
        Eigen::Vector3d exPos;
        exPos << 0.25, 0.19, 0.85;
        // Rotation
        Eigen::Matrix3d t;
        // rotz(-90)*roty(-90)
        t << 0, 1, 0, 0, 0, 1, 1, 0, 0;
        ctl.lh2Task->set_ef_pose(sva::PTransformd(t.inverse(), exPos) * X_0_mid);

        return new Prim4LeftHandLockStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive4 Left Hand Lock Step
/////////////////////////////////////////////////////////////

void Prim4LeftHandLockStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4LeftHandLockStep: __init()." << std::endl;

    ctl.prim4->set_stepByStep(stepByStep_);
}

Prim4Step * Prim4LeftHandLockStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4LeftHandLockStep: __update()." << std::endl;
  
    double diff;
    diff = ctl.lh2Task->eval().norm();
    if (diff <= 1e-2)
    { 
        static bool closed = false;
        if(closed == false)
        {
          closed = true;
          // Close/lock left gripper.
          auto gripper = ctl.grippers["l_gripper"].get();
          gripper->setTargetQ({0.0});
          // Close right gripper.        
          gripper = ctl.grippers["r_gripper"].get();
          gripper->setTargetQ({-0.7});
        }

        // Wait.
        static int wait = 0;
        wait++;
        if (wait == 500)
        {
            wait = 0;
            closed = false;
            return new Prim4BothFlipStep;
        }
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive4 Both Hands Flip Step
/////////////////////////////////////////////////////////////

void Prim4BothFlipStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4BothFlipStep: __init()." << std::endl;

    ctl.prim4->set_stepByStep(stepByStep_);
}

Prim4Step * Prim4BothFlipStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4BothFlipStep: __update()." << std::endl;

    //
    auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
    auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
    auto X_lf_rf = X_0_rf * (X_0_lf.inv());
    X_lf_rf.translation() = X_lf_rf.translation() / 2;
    auto X_0_mid = X_lf_rf * X_0_lf;
 
    // Left hand.
    sva::PTransformd leftGripper;
    leftGripper = ctl.lh2Task->get_ef_pose() * X_0_mid.inv();
    Eigen::Matrix3d leftRot;
    leftRot = leftGripper.rotation();
    Eigen::Vector3d leftTrans;
    leftTrans = leftGripper.translation();
    // Exchange position.
    Eigen::Vector3d exPosLeft;
    //exPosLeft << 0.25, 0.20, 0.90;
    exPosLeft << 0.25, 0.20, 1.0;
    // Rotation
    Eigen::Matrix3d tLeft;
    // rotz(-90)
    tLeft << 0, 1, 0, -1, 0, 0, 0, 0, 1;

    // Right hand.
    sva::PTransformd rightGripper;
    rightGripper = ctl.rh2Task->get_ef_pose() * X_0_mid.inv();
    Eigen::Matrix3d rightRot;
    rightRot = rightGripper.rotation();
    Eigen::Vector3d rightTrans;
    rightTrans = rightGripper.translation();
    // Exchange position.
    Eigen::Vector3d exPosRight;
    //exPosRight << 0.25, -0.20, 0.90;
    exPosRight << 0.25, -0.20, 1.0;
    // Rotation
    Eigen::Matrix3d tRight;
    // rotz(90)
    tRight << 0, -1, 0, 1, 0, 0, 0, 0, 1;

    // Both move.
    ctl.lh2Task->set_ef_pose(sva::PTransformd(tLeft.inverse(), exPosLeft) * X_0_mid);
    ctl.rh2Task->set_ef_pose(sva::PTransformd(tRight.inverse(), exPosRight) *  X_0_mid);
    
    return new Prim4InitPoseStep;
}

/////////////////////////////////////////////////////////////
//  Primitive4 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim4InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4SecondStep: __init()." << std::endl;

    ctl.prim4->set_stepByStep(stepByStep_);
}

Prim4Step * Prim4InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4SecondStep: __update()." << std::endl;

    double diffLeft;
    diffLeft = ctl.lh2Task->eval().norm();
    double  diffRight;
    diffRight = ctl.rh2Task->eval().norm();
    if ((diffLeft <= 1e-2) && (diffRight <= 1e-2))
    {
         return new Prim4EndStep;
    } 
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive4 End Step
/////////////////////////////////////////////////////////////

void Prim4EndStep::__init(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive4: Prim4EndStep: init." << std::endl;
}

Prim4Step * Prim4EndStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive4: Prim4EndStep: update." << std::endl;

    return nullptr;
}
}
