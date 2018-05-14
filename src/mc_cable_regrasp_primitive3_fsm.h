#pragma once

#include <iostream>
#include <Eigen/Core>

#include "mc_cable_regrasp_controller.h"

namespace mc_control
{

struct Prim3Step
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        Prim3Step(const std::string & name);
        virtual ~Prim3Step() {};
        Prim3Step * update(MCCableRegraspController & ctl);
    protected:
        virtual void __init(MCCableRegraspController & ctl) = 0;
        virtual Prim3Step * __update(MCCableRegraspController & ctl) = 0;
    public:
        bool first_call = true;
        std::string name;
};

#define CREATE_STEP(NAME, DESC, MEMBERS)\
struct NAME : public Prim3Step\
{\
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW\
    NAME() : Prim3Step(DESC) {}\
    virtual void __init(MCCableRegraspController & ctl) override;\
    virtual Prim3Step * __update(MCCableRegraspController & ctl) override;\
    MEMBERS\
};

CREATE_STEP(Prim3InitStep, "Primitive3 Initialization Step",
                bool stepByStep_ = true;
                )

CREATE_STEP(Prim3OpenGripperStep, "Primitive3 Open Gripper Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim3LeftHandFlipStep, "Primitive3 Left Hand Flip Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim3RegraspStep, "Primitive3 Left Hand Regrasp Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim3ToCenterStep, "Primitive3 To Center Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim3RightHandLockStep, "Primitive3 Right Hand Lock Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim3BothFlipStep, "Primitive3 Both Hands Flip Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim3InitPoseStep, "Primitive3 Initial Pose Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim3EndStep, "Primitive3 End Step",
                )

#undef CREATE_STEP

}
