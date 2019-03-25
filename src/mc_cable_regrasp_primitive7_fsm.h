#pragma once

#include <iostream>
#include <Eigen/Core>

#include "mc_cable_regrasp_controller.h"
#include "mc_cable_regrasp_linear_trajectory.h"

namespace mc_control
{

struct Prim7Step
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        Prim7Step(const std::string & name);
        virtual ~Prim7Step() {};
        Prim7Step * update(MCCableRegraspController & ctl);
    protected:
        virtual void __init(MCCableRegraspController & ctl) = 0;
        virtual Prim7Step * __update(MCCableRegraspController & ctl) = 0;
    public:
        bool first_call = true;
        std::string name;
};

#define CREATE_STEP(NAME, DESC, MEMBERS)\
struct NAME : public Prim7Step\
{\
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW\
    NAME() : Prim7Step(DESC) {}\
    virtual void __init(MCCableRegraspController & ctl) override;\
    virtual Prim7Step * __update(MCCableRegraspController & ctl) override;\
    MEMBERS\
};

CREATE_STEP(Prim7InitStep, "Primitive7 Initialization Step",
                bool stepByStep_ = true;
                )

CREATE_STEP(Prim7OpenGripperStep, "Primitive7 Open Two Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim7ReleaseStep, "Primitive7 Release Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim7CloseStep, "Primitive7 Close Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim7BackStep, "Primitive7 Gripper Back Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim7InitPoseStep, "Primitive7 Initial Pose Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim7EndStep, "Primitive7 End Step",
                )

#undef CREATE_STEP

}
