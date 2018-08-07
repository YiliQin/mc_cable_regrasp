#pragma once

#include <iostream>
#include <Eigen/Core>

#include "mc_cable_regrasp_controller.h"
#include "mc_cable_regrasp_linear_trajectory.h"

namespace mc_control
{

struct Prim11Step
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        Prim11Step(const std::string & name);
        virtual ~Prim11Step() {};
        Prim11Step * update(MCCableRegraspController & ctl);
    protected:
        virtual void __init(MCCableRegraspController & ctl) = 0;
        virtual Prim11Step * __update(MCCableRegraspController & ctl) = 0;
    public:
        bool first_call = true;
        std::string name;
};

#define CREATE_STEP(NAME, DESC, MEMBERS)\
struct NAME : public Prim11Step\
{\
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW\
    NAME() : Prim11Step(DESC) {}\
    virtual void __init(MCCableRegraspController & ctl) override;\
    virtual Prim11Step * __update(MCCableRegraspController & ctl) override;\
    MEMBERS\
};

CREATE_STEP(Prim11InitStep, "Primitive11 Initialization Step",
                bool stepByStep_ = true;
                )

CREATE_STEP(Prim11OpenGripperStep, "Primitive11 Open Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim11SpreadStep, "Primitive11 Spread Step",
                bool stepByStep_ = true;
                LinearTrajectory * leftHandLinearTraj;
                LinearTrajectory * rightHandLinearTraj;
                int nr_points_traj = 100;
                int cntRun = 0;
                )

CREATE_STEP(Prim11CloseGripperStep, "Primitive11 Close Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim11BackStep, "Primitive11 Back Step",
                bool stepByStep_ = true;
                LinearTrajectory * leftHandLinearTraj;
                LinearTrajectory * rightHandLinearTraj;
                int nr_points_traj = 100;
                int cntRun = 0;
                )

CREATE_STEP(Prim11InitPoseStep, "Primitive11 Initial Pose Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim11EndStep, "Primitive11 End Step",
                )

#undef CREATE_STEP

}
