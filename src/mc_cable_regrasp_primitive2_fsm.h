#pragma once

#include <iostream>
#include <Eigen/Core>

#include "mc_cable_regrasp_controller.h"
#include "mc_cable_regrasp_linear_trajectory.h"

namespace mc_control
{

struct Prim2Step
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        Prim2Step(const std::string & name);
        virtual ~Prim2Step() {};
        Prim2Step * update(MCCableRegraspController & ctl);
    protected:
        virtual void __init(MCCableRegraspController & ctl) = 0;
        virtual Prim2Step * __update(MCCableRegraspController & ctl) = 0;
    public:
        bool first_call = true;
        std::string name;
};

#define CREATE_STEP(NAME, DESC, MEMBERS)\
struct NAME : public Prim2Step\
{\
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW\
    NAME() : Prim2Step(DESC) {}\
    virtual void __init(MCCableRegraspController & ctl) override;\
    virtual Prim2Step * __update(MCCableRegraspController & ctl) override;\
    MEMBERS\
};

CREATE_STEP(Prim2InitStep, "Primitive2 Initialization Step",
                bool stepByStep_ = true;
                )

CREATE_STEP(Prim2OpenGripperStep, "Primitive2 Open Gripper Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim2SpreadStep, "Primitive2 Spread Step",
                bool stepByStep_ = true;
                LinearTrajectory * leftHandLinearTraj;
                LinearTrajectory * rightHandLinearTraj;
                int nr_points_traj = 100;
                int cntRun = 0;
                )

CREATE_STEP(Prim2CloseGripperStep, "Primitive2 Close Gripper Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim2InitPoseStep, "Primitive2 Initial Pose Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim2EndStep, "Primitive2 End Step",
                )

#undef CREATE_STEP

}
