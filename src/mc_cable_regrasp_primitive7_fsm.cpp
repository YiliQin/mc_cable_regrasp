#include "mc_cable_regrasp_primitive7_fsm.h"
#include "mc_cable_regrasp_primitive7.h"

namespace mc_control
{

Prim7Step::Prim7Step(const std::string & name)
    : name(name)
{
    //std::cout << "Prim7Step Constructed." << std::endl;
}

Prim7Step * Prim7Step::update(MCCableRegraspController & ctl)
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
//  Primitive7 Initialization Step
/////////////////////////////////////////////////////////////

void Prim7InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive7: InitSetp: __init()."<< std::endl;

    ctl.prim7->set_stepByStep(stepByStep_);
}

Prim7Step * Prim7InitStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive7: Prim7InitStep: __update()." << std::endl;
    if (ctl.prim7Continue == true)
    {    
        ctl.prim7Continue = false;
        return new Prim7OpenGripperStep;
    }
    else
        return this;
}

/////////////////////////////////////////////////////////////
//  Primitive7 Open Two Gripper Step
/////////////////////////////////////////////////////////////

void Prim7OpenGripperStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive7: Prim7OpenGripperStep: __init()." << std::endl;

    ctl.prim7->set_stepByStep(stepByStep_);        

    // Loose left gripper.
    auto gripper = ctl.grippers["l_gripper"].get();
    gripper->setTargetQ({0.5});
    // Fix right gripper.        
    gripper = ctl.grippers["r_gripper"].get();
    gripper->setTargetQ({0.5});
}

Prim7Step * Prim7OpenGripperStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive7: Prim7OpenGripperStep: __update()." << std::endl;

    // Wait.
    static int wait = 0;
    wait++;
    if (wait == 500)
    {
        wait = 0;
        return new Prim7ReleaseStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive7 Release Gripper Step
/////////////////////////////////////////////////////////////

void Prim7ReleaseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive7: Prim7ReleaseStep: __init()." << std::endl;
    ctl.prim7->set_stepByStep(stepByStep_);
}

Prim7Step * Prim7ReleaseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive7: Prim7ReleaseStep: __update()." << std::endl;

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
    //t << 0.0320, 0.0, 1.122;
    //t << -0.0320, 0.0, 1.0;
    t << -0.05, 0.0, 1.122;
    ctl.chestTask->set_ef_pose(sva::PTransformd(rot.inverse(), t) * X_0_mid);

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

    return new Prim7CloseStep;
}

/////////////////////////////////////////////////////////////
//  Primitive7 Close Gripper Step
/////////////////////////////////////////////////////////////

void Prim7CloseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive7: Prim7CloseStep: __init()." << std::endl;

    ctl.prim7->set_stepByStep(stepByStep_);        

}

Prim7Step * Prim7CloseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive7: Prim7CloseStep: __update()." << std::endl;

    double diffLeft, diffRight;
    diffLeft = ctl.lh2Task->eval().norm();
    diffRight = ctl.rh2Task->eval().norm();
    if ((diffLeft <= 1e-2) && (diffRight <= 1e-2))
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
            return new Prim7BackStep;
        }
    }

    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive7 Gripper Back Step
/////////////////////////////////////////////////////////////

void Prim7BackStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive7: Prim7BackStep: __init()." << std::endl;
    ctl.prim7->set_stepByStep(stepByStep_);
}

Prim7Step * Prim7BackStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive7: Prim7BackStep: __update()." << std::endl;

    // Wait.
    double diffLeft, diffRight;
    diffLeft = ctl.lh2Task->eval().norm();
    diffRight = ctl.rh2Task->eval().norm();

    if ((diffLeft <= 1e-2) && (diffRight <= 1e-2))
    {    
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
        //t << -0.0320, 0.0, 1.0;
        //t << -0.05, 0.0, 1.122;
        ctl.chestTask->set_ef_pose(sva::PTransformd(rot.inverse(), t) * X_0_mid);

        // left gripper
        Eigen::Matrix3d endRotLeft; 
        endRotLeft << 0.825986, -0.330466, 0.45666, 0.242954, 0.939727, 0.240597, -0.508645, -0.0877827, 0.85649;
        Eigen::Vector3d endPosLeft;
        endPosLeft << -0.00615974, 0.378751, 0.846779;

        ctl.lh2Task->set_ef_pose(sva::PTransformd(endRotLeft, endPosLeft) * X_0_mid);

        // right gripper
        Eigen::Matrix3d endRotRight; 
        endRotRight << 0.82598, 0.329603, 0.457295, -0.242991, 0.940186, -0.238757, -0.508637, 0.0860903, 0.856666;
        Eigen::Vector3d endPosRight;
        endPosRight << -0.00616116, -0.378347, 0.846025; 

        ctl.rh2Task->set_ef_pose(sva::PTransformd(endRotRight, endPosRight) * X_0_mid);

        return new Prim7InitPoseStep;
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive7 Initial Pose Step
/////////////////////////////////////////////////////////////

void Prim7InitPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive7: Prim7SecondStep: __init()." << std::endl;
    ctl.prim7->set_stepByStep(stepByStep_);
}

Prim7Step * Prim7InitPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Primitive7: Prim7SecondStep: __update()." << std::endl;

    // Wait.
    double diffLeft, diffRight;
    diffLeft = ctl.lh2Task->eval().norm();
    diffRight = ctl.rh2Task->eval().norm();
    if ((diffLeft <= 1e-2) && (diffRight <= 1e-2))
    {    
        static int wait = 0;
        wait++;
        if (wait == 500) 
        {
            wait = 0;
            return new Prim7EndStep;
        }
    }
    return this;
}

/////////////////////////////////////////////////////////////
//  Primitive7 End Step
/////////////////////////////////////////////////////////////

void Prim7EndStep::__init(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive7: Prim7EndStep: init." << std::endl;
}

Prim7Step * Prim7EndStep::__update(MCCableRegraspController &)
{
    // For test.
    //std::cout << "Primitive7: Prim7EndStep: update." << std::endl;

    return nullptr;
}

}
