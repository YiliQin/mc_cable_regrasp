#include "mc_cable_regrasp_primitive13_fsm.h"
#include "mc_cable_regrasp_primitive13.h"

namespace mc_control
{

Prim13Step::Prim13Step(const std::string & name)
    : name(name)
{
    //std::cout << "Prim13Step Constructed." << std::endl;
}

Prim13Step * Prim13Step::update(MCCableRegraspController & ctl)
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
//  Primitive13 Initialization Step
/////////////////////////////////////////////////////////////

void Prim13InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive13: Prim13InitSetp: __init()."<< std::endl;

    ctl.prim13->set_stepByStep(stepByStep_);
}

Prim13Step * Prim13InitStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive13: Prim13InitStep: __update()." << std::endl;

    return new Prim13MidPointStep;
}

/////////////////////////////////////////////////////////////
//  Primitive13 To Middle Point Step
/////////////////////////////////////////////////////////////

void Prim13MidPointStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive13: Prim13MidPointStep: __init()." << std::endl;

    ctl.prim13->set_stepByStep(stepByStep_);
}

Prim13Step * Prim13MidPointStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive13: Prim13MidPointStep: __update()." << std::endl;

    //
    auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
    auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
    auto X_lf_rf = X_0_rf * (X_0_lf.inv());
    X_lf_rf.translation() = X_lf_rf.translation() / 2;
    auto X_0_mid = X_lf_rf * X_0_lf;

    // left gripper
    sva::PTransformd leftGripper;
    leftGripper = ctl.lh2Task->get_ef_pose() * X_0_mid.inv();
    Eigen::Vector3d leftPos;
    leftPos = leftGripper.translation();

    // Right gripper.
    Eigen::Matrix3d rightRot;
    // rotz(90)
    rightRot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    Eigen::Vector3d rightPos;
    rightPos << 0.05, -(ctl.prim13->get_dis_y() + 0.1), leftPos[2];
    //
    ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot.inverse(), rightPos) * X_0_mid);

    return new Prim13PreGraspStep;
}

/////////////////////////////////////////////////////////////
//  Primitive13 Prepare Grasp Step
/////////////////////////////////////////////////////////////

void Prim13PreGraspStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive13: Prim13PreGraspStep: __init()." << std::endl;

    ctl.prim13->set_stepByStep(stepByStep_);
}

Prim13Step * Prim13PreGraspStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive13: Prim13PreGraspStep: __update()." << std::endl;

    //double diffLeft;
    double diffRight;
    diffRight = ctl.rh2Task->eval().norm();
    if (diffRight < 1e-2)
    {
        auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
        auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
        auto X_lf_rf = X_0_rf * (X_0_lf.inv());
        X_lf_rf.translation() = X_lf_rf.translation() / 2;
        auto X_0_mid = X_lf_rf * X_0_lf;

        // left gripper
        sva::PTransformd leftGripper;
        leftGripper = ctl.lh2Task->get_ef_pose() * X_0_mid.inv();
        Eigen::Vector3d leftPos;
        leftPos = leftGripper.translation();

        // Right gripper.
        Eigen::Matrix3d rightRot;
        // rotz(90)
        rightRot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
        Eigen::Vector3d rightPos;
        //rightPos << 0.2, -(ctl.prim13->get_distance()), 1.1;
        //rightPos << leftPos[0], -(ctl.prim13->get_distance()), leftPos[2] + 0.2;
        //rightPos << leftPos[0] + ctl.regraspOffset[0], -(ctl.prim13->get_distance()), leftPos[2] + 0.2;
        rightPos << leftPos[0] + ctl.prim13->get_dis_x(), -(ctl.prim13->get_dis_y()), leftPos[2] + 0.2;
        //
        ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot.inverse(), rightPos) * X_0_mid);

        return new Prim13OpenGripperStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive13 Open Gripper Step
/////////////////////////////////////////////////////////////

void Prim13OpenGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive13: Prim13OpenGripperStep: __init()." << std::endl;

    ctl.prim13->set_stepByStep(stepByStep_);
}

Prim13Step * Prim13OpenGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive13: Prim13OpenGripperStep: __update()." << std::endl;

    //double diffLeft;
    double diffRight;
    diffRight = ctl.rh2Task->eval().norm();
    if (diffRight < 1e-2) 
    {
        static bool opened = false;
        if (opened == false)
        {
            opened = true;
            // Open right gripper.        
            auto gripper = ctl.grippers["r_gripper"].get();
            gripper->setTargetQ({0.7});
        }
        static int wait = 0;
        wait++;
        if (wait == 500)
        {
            wait = 0;
            opened = false;  
            return new Prim13GraspStep;
        }
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive13 Grasp Step
/////////////////////////////////////////////////////////////

void Prim13GraspStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive13: Prim13GraspStep: __init()." << std::endl;

    ctl.prim13->set_stepByStep(stepByStep_);
}

Prim13Step * Prim13GraspStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive13: Prim13GraspStep: __update()." << std::endl;
   
    //
    if (ctl.prim13Continue == true || ctl.igStop == true)
    {            
        ctl.prim13Continue = false;
        auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
        auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
        auto X_lf_rf = X_0_rf * (X_0_lf.inv());
        X_lf_rf.translation() = X_lf_rf.translation() / 2;
        auto X_0_mid = X_lf_rf * X_0_lf;

        // left gripper
        sva::PTransformd leftGripper;
        leftGripper = ctl.lh2Task->get_ef_pose() * X_0_mid.inv();
        Eigen::Vector3d leftPos;
        leftPos = leftGripper.translation();

        // Right gripper.
        Eigen::Matrix3d rightRot;
        // rotz(90)
        rightRot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
        Eigen::Vector3d rightPos;
        //rightPos << 0.2, -(ctl.prim13->get_distance()), 1.0;
        //rightPos << leftPos[0], -(ctl.prim13->get_distance()), leftPos[2];
        //rightPos << leftPos[0] + ctl.regraspOffset[0], -(ctl.prim13->get_distance()), leftPos[2] + (-0.03);
        rightPos << leftPos[0] + ctl.prim13->get_dis_x(), -(ctl.prim13->get_dis_y()), leftPos[2] + (-0.03);
        //
        ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot.inverse(), rightPos) * X_0_mid);
        return new Prim13CloseGripperStep;
    }
    else
        return this;
}

/////////////////////////////////////////////////////////////
//  Primitive13 Close Gripper Step
/////////////////////////////////////////////////////////////

void Prim13CloseGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive13: Prim13CloseGripperStep: __init()." << std::endl;

    ctl.prim13->set_stepByStep(stepByStep_);
}

Prim13Step * Prim13CloseGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive13: Prim13CloseGripperStep: __update()." << std::endl;

    double diffRight;
    diffRight = ctl.rh2Task->eval().norm();
    if (diffRight < 1e-2)
    {
        static bool closed = false;
        if(closed == false)
        {
            closed = true;
            // close right gripper
            auto gripper = ctl.grippers["r_gripper"].get();
            gripper->setTargetQ({-0.7});
            // loose left gripper
            gripper = ctl.grippers["l_gripper"].get();
            gripper->setTargetQ({-0.4});
        }
        static int wait = 0;
        if(wait++ == 500)
        {
            wait = 0;
            closed = false;
            return new Prim13InitPoseStep;
        }
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive13 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim13InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive13: Prim13InitPoseStep: __init()." << std::endl;

    ctl.prim13->set_stepByStep(stepByStep_);
}

Prim13Step * Prim13InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive13: Prim13InitPoseStep: __update()." << std::endl;

    double diffRight;
    diffRight = ctl.rh2Task->eval().norm();
    if (diffRight < 1e-2)
    {
        return new Prim13EndStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive13 End Step
/////////////////////////////////////////////////////////////

void Prim13EndStep::__init(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive13: Prim13EndStep: init." << std::endl;
}

Prim13Step * Prim13EndStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive13: Prim13EndStep: update." << std::endl;

    return nullptr;
}

}
