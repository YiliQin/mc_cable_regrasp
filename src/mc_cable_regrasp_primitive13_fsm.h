#pragma once

#include <iostream>
#include <Eigen/Core>

#include "mc_cable_regrasp_controller.h"

namespace mc_control
{

struct Prim13Step
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        Prim13Step(const std::string & name);
        virtual ~Prim13Step() {};
        Prim13Step * update(MCCableRegraspController & ctl);
    protected:
        virtual void __init(MCCableRegraspController & ctl) = 0;
        virtual Prim13Step * __update(MCCableRegraspController & ctl) = 0;
    public:
        bool first_call = true;
        std::string name;
};

#define CREATE_STEP(NAME, DESC, MEMBERS)\
struct NAME : public Prim13Step\
{\
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW\
    NAME() : Prim13Step(DESC) {}\
    virtual void __init(MCCableRegraspController & ctl) override;\
    virtual Prim13Step * __update(MCCableRegraspController & ctl) override;\
    MEMBERS\
};

CREATE_STEP(Prim13InitStep, "Primitive13 Initialization Step",
                bool stepByStep_ = true;
                )

CREATE_STEP(Prim13PreGraspStep, "Primitive13 Prepare Grasp Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim13OpenGripperStep, "Primitive13 Open Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim13GraspStep, "Primitive13 Grasp Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim13CloseGripperStep, "Primitive13 Close Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim13InitPoseStep, "Primitive13 Initial Pose Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim13EndStep, "Primitive13 End Step",
                )

#undef CREATE_STEP

}
