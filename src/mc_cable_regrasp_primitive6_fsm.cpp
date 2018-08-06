#include "mc_cable_regrasp_primitive6_fsm.h"
#include "mc_cable_regrasp_primitive6.h"

namespace mc_control
{

Prim6Step::Prim6Step(const std::string & name)
    : name(name)
{
    //std::cout << "Prim6Step Constructed." << std::endl;
}

Prim6Step * Prim6Step::update(MCCableRegraspController & ctl)
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
//  Primitive6 Initialization Step
/////////////////////////////////////////////////////////////

void Prim6InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive6: Prim6InitSetp: __init()."<< std::endl;

    ctl.prim6->set_stepByStep(stepByStep_);
}

Prim6Step * Prim6InitStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive6: Prim6InitStep: __update()." << std::endl;

    if (ctl.prim6ContinueS1 == true)
        return new Prim6ToInterPosStep;
    else
        return this;
}

/////////////////////////////////////////////////////////////
//  Primitive6 To Intermediate Posture Step
/////////////////////////////////////////////////////////////

void Prim6ToInterPosStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive6: Prim6ToInterPosStep: __init()." << std::endl;

    ctl.prim6->set_stepByStep(stepByStep_);
}

Prim6Step * Prim6ToInterPosStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive5: Prim6HangStep: __update()." << std::endl;

    // 
    auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
    auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
    auto X_lf_rf = X_0_rf * (X_0_lf.inv());
    X_lf_rf.translation() = X_lf_rf.translation() / 2;
    auto X_0_mid = X_lf_rf * X_0_lf;

    // Cheset task.
    Eigen::Matrix3d rot;
    rot << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::Vector3d t;     
    t << 0.0320, 0.0, 1.122;
    ctl.chestTask->set_ef_pose(sva::PTransformd(rot.inverse(), t) * X_0_mid);

    // Left gripper.
    Eigen::Matrix3d leftRot;
    // rotz(-90)
    leftRot << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    Eigen::Vector3d leftPos;
    leftPos << 0.20, ctl.prim6->get_distance()/2, 1.0;
    // Right gripper.
    Eigen::Matrix3d rightRot;
    // rotz(90)
    rightRot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    Eigen::Vector3d rightPos;
    rightPos << 0.20, -(ctl.prim6->get_distance()/2), 1.0;
    //
    ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot.inverse(), leftPos) * X_0_mid);
    ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot.inverse(), rightPos) * X_0_mid);

    return new Prim6ToPrePosStep;
    // for 20180731 test
    //return new Prim6InitPoseStep;
}

/////////////////////////////////////////////////////////////
//  Primitive6 To Preparing Installation Position Step
/////////////////////////////////////////////////////////////

void Prim6ToPrePosStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    std::cout << "Primitive6: Prim6OpenGripperStep: __init()." << std::endl;

    ctl.prim6->set_stepByStep(stepByStep_);
}

Prim6Step * Prim6ToPrePosStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive6: Prim6ToPrePoseStep: __update()." << std::endl;

    // 
    auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
    auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
    auto X_lf_rf = X_0_rf * (X_0_lf.inv());
    X_lf_rf.translation() = X_lf_rf.translation() / 2;
    auto X_0_mid = X_lf_rf * X_0_lf;

    //
    double diffLeft;
    diffLeft = ctl.lh2Task->eval().norm();
    double diffRight;
    diffRight = ctl.rh2Task->eval().norm();
    if ((diffLeft < 1e-2) && (diffRight < 1e-2) && (ctl.prim6ContinueS2 == true))
    {
        // Left gripper.
        Eigen::Matrix3d leftRot;
        // rotz(-90)
        leftRot << 0, 1, 0, -1, 0, 0, 0, 0, 1;
        Eigen::Vector3d leftOffset;
        leftOffset << -0.02 + 0.02, ctl.prim6->get_distance()/2, 0.15 + 0.10;  
        Eigen::Vector3d leftPos;
        //leftPos << 0.30, ctl.prim6->get_distance()/2, 1.2;
        leftPos = ctl.marker1_pos.translation() + leftOffset;
        // Right gripper.
        Eigen::Matrix3d rightRot;
        // rotz(90)
        rightRot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
        Eigen::Vector3d rightOffset;
        rightOffset << -0.02 + 0.02, -(ctl.prim6->get_distance()/2), 0.15 + 0.10;  
        Eigen::Vector3d rightPos;
        rightPos = ctl.marker1_pos.translation() + rightOffset;
        //
        ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot.inverse(), leftPos) * X_0_mid);
        ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot.inverse(), rightPos) * X_0_mid);

        return new Prim6InsStep;
        //return new Prim6InitPoseStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive6 Installation Step
/////////////////////////////////////////////////////////////

void Prim6InsStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive6: Prim6InsStep: __init()." << std::endl;

    ctl.prim6->set_stepByStep(stepByStep_);
}

Prim6Step * Prim6InsStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive6: Prim6ToPrePoseStep: __update()." << std::endl;

    double diffLeft;
    diffLeft = ctl.lh2Task->eval().norm();
    double diffRight;
    diffRight = ctl.rh2Task->eval().norm();
    if ((diffLeft < 1e-2) && (diffRight < 1e-2) && (ctl.prim6ContinueS3 == true))
    {
        // 
        auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
        auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
        auto X_lf_rf = X_0_rf * (X_0_lf.inv());
        X_lf_rf.translation() = X_lf_rf.translation() / 2;
        auto X_0_mid = X_lf_rf * X_0_lf;

        // Left gripper.
        Eigen::Matrix3d leftRot;
        // rotz(-90)
        leftRot << 0, 1, 0, -1, 0, 0, 0, 0, 1;
        Eigen::Vector3d leftOffset;
        leftOffset << -0.02 + 0.02, ctl.prim6->get_distance()/2, 0.15 + 0.00;  
        Eigen::Vector3d leftPos;
        //leftPos << 0.30, ctl.prim6->get_distance()/2, 1.1;
        leftPos = ctl.marker1_pos.translation() + leftOffset;
        // Right gripper.
        Eigen::Matrix3d rightRot;
        // rotz(90)
        rightRot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
        Eigen::Vector3d rightOffset;
        rightOffset << -0.02 + 0.02, -(ctl.prim6->get_distance()/2), 0.15 + 0.00;
        Eigen::Vector3d rightPos;
        //rightPos << 0.30, -(ctl.prim6->get_distance()/2), 1.1;
        rightPos = ctl.marker1_pos.translation() + rightOffset;
        //
        ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot.inverse(), leftPos) * X_0_mid);
        ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot.inverse(), rightPos) * X_0_mid);

        return new Prim6InitPoseStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive6 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim6InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive6: Prim6InitPoseStep: __init()." << std::endl;

    ctl.prim6->set_stepByStep(stepByStep_);
}

Prim6Step * Prim6InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive6: Prim6InitPoseStep: __update()." << std::endl;

    double diff;
    diff = ctl.rh2Task->eval().norm();
    if (diff < 1e-2)
    {
        return new Prim6EndStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive6 End Step
/////////////////////////////////////////////////////////////

void Prim6EndStep::__init(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive6: Prim6EndStep: init." << std::endl;
}

Prim6Step * Prim6EndStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive6: Prim6EndStep: update." << std::endl;

    return nullptr;
}

}
