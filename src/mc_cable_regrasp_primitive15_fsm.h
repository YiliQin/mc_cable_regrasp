#pragma once

#include <iostream>
#include <Eigen/Core>

#include "mc_cable_regrasp_controller.h"
#include "mc_cable_regrasp_linear_trajectory.h"

namespace mc_control
{

struct Prim15Step
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        Prim15Step(const std::string & name);
        virtual ~Prim15Step() {};
        Prim15Step * update(MCCableRegraspController & ctl);
    protected:
        virtual void __init(MCCableRegraspController & ctl) = 0;
        virtual Prim15Step * __update(MCCableRegraspController & ctl) = 0;
    public:
        bool first_call = true;
        std::string name;
};

#define CREATE_STEP(NAME, DESC, MEMBERS)\
struct NAME : public Prim15Step\
{\
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW\
    NAME() : Prim15Step(DESC) {}\
    virtual void __init(MCCableRegraspController & ctl) override;\
    virtual Prim15Step * __update(MCCableRegraspController & ctl) override;\
    MEMBERS\
};

CREATE_STEP(Prim15InitStep, "Primitive15 Initialization Step",
                bool stepByStep_ = true;
                )

CREATE_STEP(Prim15BaseMoveStep, "Primitive15 Base Move Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim15ResponseStep, "Primitive15 Move Response Step",
                bool stepByStep_ = true;
                int nr_points_traj = 100;
                int cntRun = 0;
                )

CREATE_STEP(Prim15InitPoseStep, "Primitive15 Initial Pose Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim15EndStep, "Primitive15 End Step",
                )

#undef CREATE_STEP

}
