#include "mc_cable_regrasp_primitive18_fsm.h"
#include "mc_cable_regrasp_primitive18.h"

namespace mc_control
{

Prim18Step::Prim18Step(const std::string & name)
    : name(name)
{
    //std::cout << "Prim18Step Constructed." << std::endl;
}

Prim18Step * Prim18Step::update(MCCableRegraspController & ctl)
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
//  Primitive18 Initialization Step
/////////////////////////////////////////////////////////////

void Prim18InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive18: InitSetp: __init()."<< std::endl;

    ctl.prim18->set_stepByStep(stepByStep_);
}

Prim18Step * Prim18InitStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive18: Prim18InitStep: __update()." << std::endl;

    return new Prim18OpenRightStep;
}

/////////////////////////////////////////////////////////////
//  Primitive18 Open Right Gripper Step
/////////////////////////////////////////////////////////////

void Prim18OpenRightStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive18: Prim18OpenRightStep: __init()." << std::endl;

    ctl.prim18->set_stepByStep(stepByStep_);
    // Fix left gripper.
    auto gripper = ctl.grippers["l_gripper"].get();
    gripper->setTargetQ({-0.7});
    // Loose right gripper.        
    gripper = ctl.grippers["r_gripper"].get();
    gripper->setTargetQ({0.5});

}

Prim18Step * Prim18OpenRightStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive18: Prim18OpenRightStep: __update()." << std::endl;

    // Wait.
    static int wait = 0;
    wait++;
    if (wait == 500)
    {
        wait = 0;
        return new Prim18ReleRightStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive18 Release Right Gripper Step
/////////////////////////////////////////////////////////////

void Prim18ReleRightStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive18: Prim18ReleRightStep: __init()." << std::endl;
    ctl.prim18->set_stepByStep(stepByStep_);
}

Prim18Step * Prim18ReleRightStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive18: Prim18ReleRightStep: __update()." << std::endl;

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

    return new Prim18CloseRightStep;
}

/////////////////////////////////////////////////////////////
//  Primitive18 Close Right Gripper Step
/////////////////////////////////////////////////////////////

void Prim18CloseRightStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive18: Prim18CloseRightStep: __init()." << std::endl;

    ctl.prim18->set_stepByStep(stepByStep_);        

}

Prim18Step * Prim18CloseRightStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive18: Prim18CloseRightStep: __update()." << std::endl;

    double diffRight;
    diffRight = ctl.rh2Task->eval().norm();
    if (diffRight <= 1e-2)
    {
        static bool gripper_changed = false;
        if (gripper_changed == false)
        {
            gripper_changed = true;
            // Close left gripper.
            auto gripper = ctl.grippers["l_gripper"].get();
            gripper->setTargetQ({-0.7});
            // Fix right gripper.        
            gripper = ctl.grippers["r_gripper"].get();
            gripper->setTargetQ({-0.7});
        }
        // Wait.
        static int wait = 0;
        wait++;
        if (wait == 500)
        {
            wait = 0;
            gripper_changed = false;
            return new Prim18RightBackStep;
        }
    }

    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive18 Right Gripper Back Step
/////////////////////////////////////////////////////////////

void Prim18RightBackStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive18: Prim18RightBackStep: __init()." << std::endl;
    ctl.prim18->set_stepByStep(stepByStep_);
}

Prim18Step * Prim18RightBackStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive18: Prim18RightBackStep: __update()." << std::endl;

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

        // close left gripper
        auto gripper = ctl.grippers["l_gripper"].get();
        gripper->setTargetQ({-0.7});
        // close right gripper        
        gripper = ctl.grippers["r_gripper"].get();
        gripper->setTargetQ({-0.7});

        // left gripper
        Eigen::Matrix3d endRotRight; 
        //endRotRight << 0.9637, 0.0877, -0.2521, -0.1328, 0.9768, -0.1680, 0.2315, 0.1954  ,0.9530;
        endRotRight << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        Eigen::Vector3d endPosRight;
        endPosRight << -0.10, -0.3475, 0.7319; 

        ctl.rh2Task->set_ef_pose(sva::PTransformd(endRotRight, endPosRight) * X_0_mid);

        return new Prim18InitPoseStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive18 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim18InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive18: Prim18SecondStep: __init()." << std::endl;
    ctl.prim18->set_stepByStep(stepByStep_);
}

Prim18Step * Prim18InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive18: Prim18SecondStep: __update()." << std::endl;

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
            return new Prim18EndStep;
        }
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive18 End Step
/////////////////////////////////////////////////////////////

void Prim18EndStep::__init(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive18: Prim18EndStep: init." << std::endl;
}

Prim18Step * Prim18EndStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive18: Prim18EndStep: update." << std::endl;

    return nullptr;
}

}
