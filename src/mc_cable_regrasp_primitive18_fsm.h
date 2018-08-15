#pragma once

#include <iostream>
#include <Eigen/Core>

#include "mc_cable_regrasp_controller.h"
#include "mc_cable_regrasp_linear_trajectory.h"

namespace mc_control
{

struct Prim18Step
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        Prim18Step(const std::string & name);
        virtual ~Prim18Step() {};
        Prim18Step * update(MCCableRegraspController & ctl);
    protected:
        virtual void __init(MCCableRegraspController & ctl) = 0;
        virtual Prim18Step * __update(MCCableRegraspController & ctl) = 0;
    public:
        bool first_call = true;
        std::string name;
};

#define CREATE_STEP(NAME, DESC, MEMBERS)\
struct NAME : public Prim18Step\
{\
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW\
    NAME() : Prim18Step(DESC) {}\
    virtual void __init(MCCableRegraspController & ctl) override;\
    virtual Prim18Step * __update(MCCableRegraspController & ctl) override;\
    MEMBERS\
};

CREATE_STEP(Prim18InitStep, "Primitive18 Initialization Step",
                bool stepByStep_ = true;
                )

CREATE_STEP(Prim18OpenRightStep, "Primitive18 Open Right Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim18ReleRightStep, "Primitive18 Release Right Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim18CloseRightStep, "Primitive18 Close Right Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim18RightBackStep, "Primitive18 Right Gripper Back Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim18InitPoseStep, "Primitive18 Initial Pose Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim18EndStep, "Primitive18 End Step",
                )

#undef CREATE_STEP

}
