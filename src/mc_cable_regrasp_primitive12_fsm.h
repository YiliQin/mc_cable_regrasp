#pragma once

#include <iostream>
#include <Eigen/Core>

#include "mc_cable_regrasp_controller.h"
#include "mc_cable_regrasp_linear_trajectory.h"

namespace mc_control
{

struct Prim12Step
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        Prim12Step(const std::string & name);
        virtual ~Prim12Step() {};
        Prim12Step * update(MCCableRegraspController & ctl);
    protected:
        virtual void __init(MCCableRegraspController & ctl) = 0;
        virtual Prim12Step * __update(MCCableRegraspController & ctl) = 0;
    public:
        bool first_call = true;
        std::string name;
};

#define CREATE_STEP(NAME, DESC, MEMBERS)\
struct NAME : public Prim12Step\
{\
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW\
    NAME() : Prim12Step(DESC) {}\
    virtual void __init(MCCableRegraspController & ctl) override;\
    virtual Prim12Step * __update(MCCableRegraspController & ctl) override;\
    MEMBERS\
};

CREATE_STEP(Prim12InitStep, "Primitive12 Initialization Step",
                bool stepByStep_ = true;
                )

CREATE_STEP(Prim12OpenGripperStep, "Primitive12 Open Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim12SpreadStep, "Primitive12 Spread Step",
                bool stepByStep_ = true;
                LinearTrajectory * leftHandLinearTraj;
                LinearTrajectory * rightHandLinearTraj;
                int nr_points_traj = 100;
                int cntRun = 0;
                )

CREATE_STEP(Prim12CloseGripperStep, "Primitive12 Close Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim12BackStep, "Primitive12 Back Step",
                bool stepByStep_ = true;
                LinearTrajectory * leftHandLinearTraj;
                LinearTrajectory * rightHandLinearTraj;
                int nr_points_traj = 100;
                int cntRun = 0;
                )

CREATE_STEP(Prim12OpenRightStep, "Primitive12 Open Right Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim12ReleRightStep, "Primitive12 Release Right Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim12RightBackStep, "Primitive12 Right Gripper Back Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim12InitPoseStep, "Primitive12 Initial Pose Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim12EndStep, "Primitive12 End Step",
                )

#undef CREATE_STEP

}
