#include "mc_cable_regrasp_primitive4_fsm.h"
#include "mc_cable_regrasp_primitive4.h"

namespace mc_control
{

Prim4Step::Prim4Step(const std::string & name)
    : name(name)
{
    std::cout << "Prim4Step Constructed." << std::endl;
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
    //ctl.fsmtest = 1;
    
    ctl.prim4->stepByStep = stepByStep_;
}

Prim4Step * Prim4InitStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4InitStep: __update()." << std::endl;
    ctl.fsmtest = 1;

    //return this;
    return new Prim4OpenGripperStep;
}

/////////////////////////////////////////////////////////////
//  Primitive4 Open Gripper Step
/////////////////////////////////////////////////////////////

void Prim4OpenGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4OpenGripperStep: __init()." << std::endl;
    //ctl.fsmtest = 1;

    ctl.prim4->stepByStep = stepByStep_;
}

Prim4Step * Prim4OpenGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4OpenGripperStep: __update()." << std::endl;
    //ctl.fsmtest = 1;

    // Open left gripper.
    auto gripper = ctl.grippers["l_gripper"].get();
    gripper->setTargetQ({0.5});
    // Close right gripper.        
    gripper = ctl.grippers["r_gripper"].get();
    gripper->setTargetQ({-0.5});

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
    //ctl.fsmtest = 1;
 
    ctl.prim4->stepByStep = stepByStep_;
}

Prim4Step * Prim4RightHandFlipStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4RightHandFlipStep: __update()." << std::endl;
 
    sva::PTransformd rightGripper;
    rightGripper = ctl.rh2Task->get_ef_pose();
    Eigen::Matrix3d rightRot;
    rightRot = rightGripper.rotation();
    Eigen::Vector3d rightTrans;
    rightTrans = rightGripper.translation();
    // Exchange position.
    Eigen::Vector3d exPos;
    exPos << 0.2, -0.3, 1.0;
    // Rotation
    Eigen::Matrix3d t;
    // roty(-90)*rotx(90)*rotz(-90)
    t << -1, 0, 0, 0, 0, -1, 0, -1, 0;
    rightRot = rightRot * t;
    ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot, exPos));

    return new Prim4ToCenterStep;
}

/////////////////////////////////////////////////////////////
//  Primitive4 To Center Step
/////////////////////////////////////////////////////////////

void Prim4ToCenterStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4ToCenterStep: __init()." << std::endl;
    //ctl.fsmtest = 1;

    ctl.prim4->stepByStep = stepByStep_;
}

Prim4Step * Prim4ToCenterStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4ToCenterStep: __update()." << std::endl;

    double diff;
    diff = ctl.rh2Task->eval().norm();
    if (diff <= 1e-2)
    { 
        sva::PTransformd rightGripper;
        rightGripper = ctl.rh2Task->get_ef_pose();
        Eigen::Matrix3d rightRot;
        rightRot = rightGripper.rotation();
        Eigen::Vector3d rightTrans;
        rightTrans = rightGripper.translation();
        // Exchange position.
        Eigen::Vector3d exPos;
        exPos << 0.25, -0.19, 1.0;
        // Rotation
        ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot, exPos));
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
    //ctl.fsmtest = 1;

    ctl.prim4->stepByStep = stepByStep_;
}

Prim4Step * Prim4RegraspStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4RegraspStep: __update()." << std::endl;

    double diff;
    diff = ctl.rh2Task->eval().norm();
    if (diff <= 1e-2)
    { 
        sva::PTransformd leftGripper;
        leftGripper = ctl.lh2Task->get_ef_pose();
        Eigen::Matrix3d leftRot;
        leftRot = leftGripper.rotation();
        Eigen::Vector3d leftTrans;
        leftTrans = leftGripper.translation();
        // Exchange position.
        Eigen::Vector3d exPos;
        exPos << 0.25, 0.19, 0.85;
        // Rotation
        Eigen::Matrix3d t;
        // rotx(-90)
        t << 1, 0, 0, 0, 0, -1, 0, 1, 0;
        leftRot = leftRot * t;
        ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot, exPos));

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
    //ctl.fsmtest = 1;

    ctl.prim4->stepByStep = stepByStep_;
}

Prim4Step * Prim4LeftHandLockStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4LeftHandLockStep: __update()." << std::endl;
  
    double diff;
    diff = ctl.lh2Task->eval().norm();
    if (diff <= 1e-2)
    { 
        // Close/lock left gripper.
        auto gripper = ctl.grippers["l_gripper"].get();
        gripper->setTargetQ({0.1});
        // Close right gripper.        
        gripper = ctl.grippers["r_gripper"].get();
        gripper->setTargetQ({-0.5});

        // Wait.
        static int wait = 0;
        wait++;
        if (wait == 500)
        {
            wait = 0;
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
    //ctl.fsmtest = 1;

    ctl.prim4->stepByStep = stepByStep_;
}

Prim4Step * Prim4BothFlipStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4BothFlipStep: __update()." << std::endl;

    double diff;
    diff = ctl.lh2Task->eval().norm();
    if (diff <= 1e-2)
    { 
        // Left hand.
        sva::PTransformd leftGripper;
        leftGripper = ctl.lh2Task->get_ef_pose();
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
        // rotz(90)*rotx(-90)*roty(90)
        tLeft << -1, 0, 0, 0, 0, 1, 0, -1, 0;
        leftRot = leftRot * tLeft;

        // Right hand.
        sva::PTransformd rightGripper;
        rightGripper = ctl.rh2Task->get_ef_pose();
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
        // rotz(-90)*rotx(-90)*roty(90)
        tRight << -1, 0, 0, 0, 0, -1, 0, -1, 0;
        rightRot = rightRot * tRight;

        // Both move.
        ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot, exPosLeft));
        ctl.rh2Task->set_ef_pose(sva::PTransformd(rightRot, exPosRight));

        return new Prim4InitPoseStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive4 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim4InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4SecondStep: __init()." << std::endl;
    //ctl.fsmtest = 1;

    ctl.prim4->stepByStep = stepByStep_;
}

Prim4Step * Prim4InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4SecondStep: __update()." << std::endl;
    //ctl.fsmtest = 1;

    double  diff;
    diff = ctl.rh2Task->eval().norm();
    if (diff <= 1e-2)
    {
         return new Prim4EndStep;
    } 
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive4 End Step
/////////////////////////////////////////////////////////////

void Prim4EndStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4EndStep: init." << std::endl;
    ctl.fsmtest = 1;
}

Prim4Step * Prim4EndStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive4: Prim4EndStep: update." << std::endl;
    //ctl.fsmtest = 1;

    ctl.prim4->finish = true;
    return nullptr;
}
}
