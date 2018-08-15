#include "mc_cable_regrasp_primitive17_fsm.h"
#include "mc_cable_regrasp_primitive17.h"

namespace mc_control
{

Prim17Step::Prim17Step(const std::string & name)
    : name(name)
{
    //std::cout << "Prim17Step Constructed." << std::endl;
}

Prim17Step * Prim17Step::update(MCCableRegraspController & ctl)
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
//  Primitive17 Initialization Step
/////////////////////////////////////////////////////////////

void Prim17InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive17: InitSetp: __init()."<< std::endl;

    ctl.prim17->set_stepByStep(stepByStep_);
}

Prim17Step * Prim17InitStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive17: Prim17InitStep: __update()." << std::endl;

    return new Prim17OpenLeftStep;
}

/////////////////////////////////////////////////////////////
//  Primitive17 Open Left Gripper Step
/////////////////////////////////////////////////////////////

void Prim17OpenLeftStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive17: Prim17OpenLeftStep: __init()." << std::endl;

    ctl.prim17->set_stepByStep(stepByStep_);        

    // Loose left gripper.
    auto gripper = ctl.grippers["l_gripper"].get();
    gripper->setTargetQ({0.5});
    // Fix right gripper.        
    gripper = ctl.grippers["r_gripper"].get();
    gripper->setTargetQ({-0.7});
}

Prim17Step * Prim17OpenLeftStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive17: Prim17OpenLeftStep: __update()." << std::endl;

    // Wait.
    static int wait = 0;
    wait++;
    if (wait == 500)
    {
        wait = 0;
        return new Prim17ReleLeftStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive17 Release Left Gripper Step
/////////////////////////////////////////////////////////////

void Prim17ReleLeftStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive17: Prim17ReleLeftStep: __init()." << std::endl;
    ctl.prim17->set_stepByStep(stepByStep_);
}

Prim17Step * Prim17ReleLeftStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive17: Prim17ReleLeftStep: __update()." << std::endl;

    //
    auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
    auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
    auto X_lf_rf = X_0_rf * (X_0_lf.inv());
    X_lf_rf.translation() = X_lf_rf.translation() / 2;
    auto X_0_mid = X_lf_rf * X_0_lf;

    // left gripper
    sva::PTransformd leftGripper;
    leftGripper = ctl.lh2Task->get_ef_pose() * X_0_mid.inv();
    Eigen::Vector3d startPosLeft;
    startPosLeft = leftGripper.translation();
    Eigen::Matrix3d startRotLeft;
    startRotLeft = leftGripper.rotation();

    Eigen::Vector3d leftDiff;
    leftDiff << 0.0, 0.0, 0.15;
    Eigen::Vector3d endPosLeft;
    endPosLeft = startPosLeft + leftDiff;
    Eigen::Matrix3d endRotLeft;
    endRotLeft = startRotLeft;

    ctl.lh2Task->set_ef_pose(sva::PTransformd(endRotLeft, endPosLeft) * X_0_mid);

    return new Prim17CloseLeftStep;
}

/////////////////////////////////////////////////////////////
//  Primitive17 Close Left Gripper Step
/////////////////////////////////////////////////////////////

void Prim17CloseLeftStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive17: Prim17CloseLeftStep: __init()." << std::endl;

    ctl.prim17->set_stepByStep(stepByStep_);        

}

Prim17Step * Prim17CloseLeftStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive17: Prim17CloseLeftStep: __update()." << std::endl;

    double diffLeft;
    diffLeft = ctl.lh2Task->eval().norm();
    if (diffLeft <= 1e-2)
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
            return new Prim17LeftBackStep;
        }
    }

    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive17 Left Gripper Back Step
/////////////////////////////////////////////////////////////

void Prim17LeftBackStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive17: Prim17LeftBackStep: __init()." << std::endl;
    ctl.prim17->set_stepByStep(stepByStep_);
}

Prim17Step * Prim17LeftBackStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive17: Prim17LeftBackStep: __update()." << std::endl;

    // Wait.
    double diffLeft;
    diffLeft = ctl.lh2Task->eval().norm();
    if (diffLeft <= 1e-2)
    {    
        //
        auto X_0_lf = ctl.robot().surface("LFullSole").X_0_s(ctl.robot());
        auto X_0_rf = ctl.robot().surface("RFullSole").X_0_s(ctl.robot());
        auto X_lf_rf = X_0_rf * (X_0_lf.inv());
        X_lf_rf.translation() = X_lf_rf.translation() / 2;
        auto X_0_mid = X_lf_rf * X_0_lf;

        // left gripper
        Eigen::Matrix3d endRotLeft; 
        //endRotLeft << 0.9637, 0.0877, -0.2521, -0.1328, 0.9768, -0.1680, 0.2315, 0.1954  ,0.9530;
        endRotLeft << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        Eigen::Vector3d endPosLeft;
        endPosLeft << -0.10, 0.3475, 0.7319; 

        ctl.lh2Task->set_ef_pose(sva::PTransformd(endRotLeft, endPosLeft) * X_0_mid);

        return new Prim17InitPoseStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive17 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim17InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive17: Prim17SecondStep: __init()." << std::endl;
    ctl.prim17->set_stepByStep(stepByStep_);
}

Prim17Step * Prim17InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive17: Prim17SecondStep: __update()." << std::endl;

    // Wait.
    double diffLeft;
    diffLeft = ctl.lh2Task->eval().norm();
    if (diffLeft <= 1e-2)
    {    
        static int wait = 0;
        wait++;
        if (wait == 500) 
        {
            wait = 0;
            return new Prim17EndStep;
        }
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive17 End Step
/////////////////////////////////////////////////////////////

void Prim17EndStep::__init(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive17: Prim17EndStep: init." << std::endl;
}

Prim17Step * Prim17EndStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive17: Prim17EndStep: update." << std::endl;

    return nullptr;
}

}
