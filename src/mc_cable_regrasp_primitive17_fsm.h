#pragma once

#include <iostream>
#include <Eigen/Core>

#include "mc_cable_regrasp_controller.h"
#include "mc_cable_regrasp_linear_trajectory.h"

namespace mc_control
{

struct Prim17Step
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        Prim17Step(const std::string & name);
        virtual ~Prim17Step() {};
        Prim17Step * update(MCCableRegraspController & ctl);
    protected:
        virtual void __init(MCCableRegraspController & ctl) = 0;
        virtual Prim17Step * __update(MCCableRegraspController & ctl) = 0;
    public:
        bool first_call = true;
        std::string name;
};

#define CREATE_STEP(NAME, DESC, MEMBERS)\
struct NAME : public Prim17Step\
{\
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW\
    NAME() : Prim17Step(DESC) {}\
    virtual void __init(MCCableRegraspController & ctl) override;\
    virtual Prim17Step * __update(MCCableRegraspController & ctl) override;\
    MEMBERS\
};

CREATE_STEP(Prim17InitStep, "Primitive17 Initialization Step",
                bool stepByStep_ = true;
                )

CREATE_STEP(Prim17OpenLeftStep, "Primitive17 Open Left Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim17ReleLeftStep, "Primitive17 Release Left Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim17CloseLeftStep, "Primitive17 Close Left Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim17LeftBackStep, "Primitive17 Left Gripper Back Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim17InitPoseStep, "Primitive17 Initial Pose Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim17EndStep, "Primitive17 End Step",
                )

#undef CREATE_STEP

}
