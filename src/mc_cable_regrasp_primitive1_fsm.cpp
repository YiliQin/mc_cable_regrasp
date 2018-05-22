#include "mc_cable_regrasp_primitive1_fsm.h"
#include "mc_cable_regrasp_primitive1.h"

namespace mc_control
{

Prim1Step::Prim1Step(const std::string & name)
    : name(name)
{
    //std::cout << "Prim1Step Constructed." << std::endl;
}

Prim1Step * Prim1Step::update(MCCableRegraspController & ctl)
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
//  Primitive1 Initialization Step
/////////////////////////////////////////////////////////////

void Prim1InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: InitSetp: __init()."<< std::endl;

    ctl.prim1->set_stepByStep(stepByStep_);
}

Prim1Step * Prim1InitStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1InitStep: __update()." << std::endl;
    //return this;
    ctl.neglectFctInp = ctl.neglectFctInp;

    //return this;
    return new Prim1OpenGripperStep;
}

/////////////////////////////////////////////////////////////
//  Primitive1 Open Gripper Step
/////////////////////////////////////////////////////////////

void Prim1OpenGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1OpenGripperStep: __init()." << std::endl;

    ctl.prim1->set_stepByStep(stepByStep_);
}

Prim1Step * Prim1OpenGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1OpenGripperStep: __update()." << std::endl;

    // Fxied left gripper.
    auto gripper = ctl.grippers["l_gripper"].get();
    gripper->setTargetQ({-0.5});
    // Loose right gripper.        
    gripper = ctl.grippers["r_gripper"].get();
    gripper->setTargetQ({0.0});

    // Wait.
    static int wait = 0;
    wait++;
    if (wait == 200)
    {
        wait = 0;
        return new Prim1SpreadStep;
    }
    else 
    {
        return this;
    }
}

/////////////////////////////////////////////////////////////
//  Primitive1 Spread Step
/////////////////////////////////////////////////////////////

void Prim1SpreadStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1SpreadStep: __init()." << std::endl;

    ctl.prim1->set_stepByStep(stepByStep_);
    // left gripper
    sva::PTransformd leftGripper;
    leftGripper = ctl.lh2Task->get_ef_pose();
    Eigen::Vector3d startPosLeft;
    startPosLeft = leftGripper.translation();
    Eigen::Matrix3d startRotLeft;
    startRotLeft = leftGripper.rotation();
    Eigen::Vector3d leftDiff;
    leftDiff << 0.0, ctl.prim1->get_slideLen()/2, 0.0;
    Eigen::Vector3d endPosLeft;
    endPosLeft = startPosLeft + leftDiff;
    Eigen::Matrix3d endRotLeft;
    endRotLeft = startRotLeft;
    leftHandLinearTraj = new Prim1Traj1(startPosLeft, endPosLeft, startRotLeft, endRotLeft, nr_points_traj);
    // right gripper
    sva::PTransformd rightGripper;
    rightGripper = ctl.rh2Task->get_ef_pose();
    Eigen::Vector3d startPosRight;
    startPosRight = rightGripper.translation();
    Eigen::Matrix3d startRotRight;
    startRotRight = rightGripper.rotation();
    Eigen::Vector3d rightDiff;
    rightDiff << 0.0, -(ctl.prim1->get_slideLen()/2), 0.0;
    Eigen::Vector3d endPosRight;
    endPosRight = startPosRight + rightDiff;
    Eigen::Matrix3d endRotRight;
    endRotRight = startRotRight;
    rightHandLinearTraj = new Prim1Traj1(startPosRight, endPosRight, startRotRight, endRotRight, nr_points_traj);
}

Prim1Step * Prim1SpreadStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1SpreadStep: __update()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    Eigen::Vector3d vzero = Eigen::Vector3d::Zero();
    Eigen::Matrix3d mide = Eigen::Matrix3d::Identity();
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> tt(vzero, vzero, vzero, mide); 
    if (cntRun >= nr_points_traj)
    {
        cntRun = 0;
        delete leftHandLinearTraj;
        delete rightHandLinearTraj;
        return new Prim1CloseGripperStep;
    }
    else
    {
        // left gripper
        tt = leftHandLinearTraj->pop();
        Eigen::Vector3d leftPos;
        leftPos = std::get<0>(tt);
        Eigen::Vector3d leftVel;
        leftVel = std::get<1>(tt);
        //ctl.lh2Task->positionTask->refvel(leftVel);
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
        //ctl.rh2Task->positionTask->refvel(rightVel);
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
//  Primitive1 Close Gripper Step
/////////////////////////////////////////////////////////////

void Prim1CloseGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1CloseGripperStep: __init()." << std::endl;

    ctl.prim1->set_stepByStep(stepByStep_);
}

Prim1Step * Prim1CloseGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1CloseGripperStep: __update()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    // Wait.
    double diffLeft;    
    diffLeft = ctl.lh2Task->eval().norm();
    double diffRight;
    diffRight = ctl.rh2Task->eval().norm();
    if ((diffLeft <= 1e-2) && (diffRight <= 1e-2))
    {    
        return new Prim1InitPoseStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive1 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim1InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1SecondStep: __init()." << std::endl;
    ctl.prim1->set_stepByStep(stepByStep_);
}

Prim1Step * Prim1InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1SecondStep: __update()." << std::endl;
    ctl.neglectFctInp = ctl.neglectFctInp;

    return new Prim1EndStep;
}

/////////////////////////////////////////////////////////////
//  Primitive1 End Step
/////////////////////////////////////////////////////////////

void Prim1EndStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1EndStep: init." << std::endl;
    ctl.neglectFctInp = ctl.neglectFctInp;
}

Prim1Step * Prim1EndStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive1: Prim1EndStep: update." << std::endl;
    ctl.neglectFctInp = ctl.neglectFctInp;

    //ctl.prim1->finish = true; 
    return nullptr;
}

}
