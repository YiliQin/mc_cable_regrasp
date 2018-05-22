#pragma once

#include <iostream>
#include <Eigen/Core>

#include "mc_cable_regrasp_controller.h"
#include "mc_cable_regrasp_primitive1_traj1.h"

namespace mc_control
{

struct Prim1Step
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        Prim1Step(const std::string & name);
        virtual ~Prim1Step() {};
        Prim1Step * update(MCCableRegraspController & ctl);
    protected:
        virtual void __init(MCCableRegraspController & ctl) = 0;
        virtual Prim1Step * __update(MCCableRegraspController & ctl) = 0;
    public:
        bool first_call = true;
        std::string name;
};

#define CREATE_STEP(NAME, DESC, MEMBERS)\
struct NAME : public Prim1Step\
{\
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW\
    NAME() : Prim1Step(DESC) {}\
    virtual void __init(MCCableRegraspController & ctl) override;\
    virtual Prim1Step * __update(MCCableRegraspController & ctl) override;\
    MEMBERS\
};

CREATE_STEP(Prim1InitStep, "Primitive1 Initialization Step",
                bool stepByStep_ = true;
                )

CREATE_STEP(Prim1OpenGripperStep, "Primitive1 Open Gripper Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim1SpreadStep, "Primitive1 Spread Step",
                bool stepByStep_ = true;
                Prim1Traj1 * leftHandLinearTraj;
                Prim1Traj1 * rightHandLinearTraj;
                int nr_points_traj = 100;
                int cntRun = 0;
                )

CREATE_STEP(Prim1CloseGripperStep, "Primitive1 Close Gripper Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim1InitPoseStep, "Primitive1 Initial Pose Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim1EndStep, "Primitive1 End Step",
                )

#undef CREATE_STEP

}
