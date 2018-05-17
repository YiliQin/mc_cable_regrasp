#include "mc_cable_regrasp_primitive3_fsm.h"
#include "mc_cable_regrasp_primitive3.h"

namespace mc_control
{

Prim3Step::Prim3Step(const std::string & name)
    : name(name)
{
    //std::cout << "Prim3Step Constructed." << std::endl;
}

Prim3Step * Prim3Step::update(MCCableRegraspController & ctl)
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
//  Primitive3 Initialization Step
/////////////////////////////////////////////////////////////

void Prim3InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: InitSetp: __init()."<< std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;
    
    ctl.prim3->set_stepByStep(stepByStep_);
}

Prim3Step * Prim3InitStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3InitStep: __update()." << std::endl;
    ctl.neglectFctInp = ctl.neglectFctInp;

    //return this;
    return new Prim3OpenGripperStep;
}

/////////////////////////////////////////////////////////////
//  Primitive3 Open Gripper Step
/////////////////////////////////////////////////////////////

void Prim3OpenGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3OpenGripperStep: __init()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    ctl.prim3->set_stepByStep(stepByStep_);
}

Prim3Step * Prim3OpenGripperStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3OpenGripperStep: __update()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    // Close left gripper.
    auto gripper = ctl.grippers["l_gripper"].get();
    gripper->setTargetQ({-0.5});
    // Open right gripper.        
    gripper = ctl.grippers["r_gripper"].get();
    gripper->setTargetQ({0.5});

    // Wait.
    static int wait = 0;
    wait++;
    if (wait == 200)
    {
        wait = 0;
        return new Prim3LeftHandFlipStep;
    }
    else
    { 
         return this;
    }
}

/////////////////////////////////////////////////////////////
//  Primitive3 Left Hand Flip Step
/////////////////////////////////////////////////////////////

void Prim3LeftHandFlipStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3LeftHandFlipStep: __init()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;
 
    ctl.prim3->set_stepByStep(stepByStep_);
}

Prim3Step * Prim3LeftHandFlipStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3LeftHandFlipStep: __update()." << std::endl;
 
    sva::PTransformd leftGripper;
    leftGripper = ctl.lh2Task->get_ef_pose();
    Eigen::Matrix3d leftRot;
    leftRot = leftGripper.rotation();
    Eigen::Vector3d leftTrans;
    leftTrans = leftGripper.translation();
    // Exchange position.
    Eigen::Vector3d exPos;
    exPos << 0.2, 0.3, 1.0;
    // Rotation
    Eigen::Matrix3d t;
    // rotx(-90)*rotz(90)
    t << 0, -1, 0, 0, 0, 1, -1, 0, 0;
    ctl.lh2Task->set_ef_pose(sva::PTransformd(t.inverse(), exPos));

    return new Prim3ToCenterStep;
}

/////////////////////////////////////////////////////////////
//  Primitive3 To Center Step
/////////////////////////////////////////////////////////////

void Prim3ToCenterStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3ToCenterStep: __init()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    ctl.prim3->set_stepByStep(stepByStep_);
}

Prim3Step * Prim3ToCenterStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3ToCenterStep: __update()." << std::endl;

    double diff;
    diff = ctl.lh2Task->eval().norm();
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
        exPos << 0.25, 0.19, 1.0;
        // Rotation
        ctl.lh2Task->set_ef_pose(sva::PTransformd(leftRot, exPos));
        return new Prim3RegraspStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive3 Left Hand Regrasp Step
/////////////////////////////////////////////////////////////

void Prim3RegraspStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3RegraspStep: __init()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    ctl.prim3->set_stepByStep(stepByStep_);
}

Prim3Step * Prim3RegraspStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3RegraspStep: __update()." << std::endl;

    double diff;
    diff = ctl.lh2Task->eval().norm();
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
        exPos << 0.25, -0.19, 0.85;
        // Rotation
        Eigen::Matrix3d t;
        // rotz(90)*roty(-90)
        t << 0, -1, 0, 0, 0, -1, 1, 0, 0;
        ctl.rh2Task->set_ef_pose(sva::PTransformd(t.inverse(), exPos));

        return new Prim3RightHandLockStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive3 Right Hand Lock Step
/////////////////////////////////////////////////////////////

void Prim3RightHandLockStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3RightHandLockStep: __init()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    ctl.prim3->set_stepByStep(stepByStep_);
}

Prim3Step * Prim3RightHandLockStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3RightHandLockStep: __update()." << std::endl;
  
    double diff;
    diff = ctl.rh2Task->eval().norm();
    if (diff <= 1e-2)
    { 
        // Close left gripper.
        auto gripper = ctl.grippers["l_gripper"].get();
        gripper->setTargetQ({-0.5});
        // Close/lock right gripper.        
        gripper = ctl.grippers["r_gripper"].get();
        gripper->setTargetQ({0.1});

        // Wait.
        static int wait = 0;
        wait++;
        if (wait == 500)
        {
            wait = 0;
            return new Prim3BothFlipStep;
        }
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive3 Both Hands Flip Step
/////////////////////////////////////////////////////////////

void Prim3BothFlipStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3BothFlipStep: __init()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    ctl.prim3->set_stepByStep(stepByStep_);
}

Prim3Step * Prim3BothFlipStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3BothFlipStep: __update()." << std::endl;

    double diff;
    diff = ctl.rh2Task->eval().norm();
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
        // rotz(-90)
        tLeft << 0, 1, 0, -1, 0, 0, 0, 0, 1;

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
        // rotz(90)
        tRight << 0, -1, 0, 1, 0, 0, 0, 0, 1;

        // Both move.
        ctl.lh2Task->set_ef_pose(sva::PTransformd(tLeft.inverse(), exPosLeft));
        ctl.rh2Task->set_ef_pose(sva::PTransformd(tRight.inverse(), exPosRight));

        return new Prim3InitPoseStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive3 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim3InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3SecondStep: __init()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    ctl.prim3->set_stepByStep(stepByStep_);
}

Prim3Step * Prim3InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3SecondStep: __update()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    double  diff;
    diff = ctl.lh2Task->eval().norm();
    if (diff <= 1e-2)
    {
         return new Prim3EndStep;
    } 
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive3 End Step
/////////////////////////////////////////////////////////////

void Prim3EndStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3EndStep: init." << std::endl;
    ctl.neglectFctInp = ctl.neglectFctInp;
}

Prim3Step * Prim3EndStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive3: Prim3EndStep: update." << std::endl;
    ctl.neglectFctInp = ctl.neglectFctInp;

    //ctl.prim3->finish = true;
    return nullptr;
}
}
