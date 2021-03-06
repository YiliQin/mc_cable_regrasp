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

Prim6Step * Prim6InitStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive6: Prim6InitStep: __update()." << std::endl;

    return new Prim6ToInterPosStep;
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
    leftPos << 0.20, ctl.prim6->get_distance()/2, 1.0;
    // Right gripper.
    Eigen::Matrix3d rightRot;
    // rotz(90)
    rightRot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    Eigen::Vector3d rightPos;
    rightPos << 0.20, -(ctl.prim6->get_distance()/2), 1.0;
    //
    ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot.inverse(), leftPos));
    ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot.inverse(), rightPos));

    return new Prim6ToPrePosStep;
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

    double diffLeft;
    diffLeft = ctl.lh2Task->eval().norm();
    double diffRight;
    diffRight = ctl.rh2Task->eval().norm();
    if ((diffLeft < 1e-2) && (diffRight < 1e-2))
    {
        // Left gripper.
        Eigen::Matrix3d leftRot;
        // rotz(-90)
        leftRot << 0, 1, 0, -1, 0, 0, 0, 0, 1;
        Eigen::Vector3d leftPos;
        leftPos << 0.30, ctl.prim6->get_distance()/2, 1.2;
        // Right gripper.
        Eigen::Matrix3d rightRot;
        // rotz(90)
        rightRot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
        Eigen::Vector3d rightPos;
        rightPos << 0.30, -(ctl.prim6->get_distance()/2), 1.2;
        //
        ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot.inverse(), leftPos));
        ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot.inverse(), rightPos));

        return new Prim6InsStep;
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
    if ((diffLeft < 1e-2) && (diffRight < 1e-2))
    {
        // Left gripper.
        Eigen::Matrix3d leftRot;
        // rotz(-90)
        leftRot << 0, 1, 0, -1, 0, 0, 0, 0, 1;
        Eigen::Vector3d leftPos;
        leftPos << 0.30, ctl.prim6->get_distance()/2, 1.1;
        // Right gripper.
        Eigen::Matrix3d rightRot;
        // rotz(90)
        rightRot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
        Eigen::Vector3d rightPos;
        rightPos << 0.30, -(ctl.prim6->get_distance()/2), 1.1;
        //
        ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot.inverse(), leftPos));
        ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot.inverse(), rightPos));

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
