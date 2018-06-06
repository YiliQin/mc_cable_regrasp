#pragma once

#include <iostream>
#include <Eigen/Core>

#include "mc_cable_regrasp_controller.h"

namespace mc_control
{

struct Prim4Step
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        Prim4Step(const std::string & name);
        virtual ~Prim4Step() {};
        Prim4Step * update(MCCableRegraspController & ctl);
    protected:
        virtual void __init(MCCableRegraspController & ctl) = 0;
        virtual Prim4Step * __update(MCCableRegraspController & ctl) = 0;
    public:
        bool first_call = true;
        std::string name;
};

#define CREATE_STEP(NAME, DESC, MEMBERS)\
struct NAME : public Prim4Step\
{\
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW\
    NAME() : Prim4Step(DESC) {}\
    virtual void __init(MCCableRegraspController & ctl) override;\
    virtual Prim4Step * __update(MCCableRegraspController & ctl) override;\
    MEMBERS\
};

CREATE_STEP(Prim4InitStep, "Primitive4 Initialization Step",
                bool stepByStep_ = true;
                )

CREATE_STEP(Prim4OpenGripperStep, "Primitive4 Open Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim4RightHandFlipStep, "Primitive4 Right Hand Flip Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim4RegraspStep, "Primitive4 Left Hand Regrasp Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim4ToCenterStep, "Primitive4 To Center Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim4LeftHandLockStep, "Primitive4 Left Hand Lock Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim4BothFlipStep, "Primitive4 Both Hands Flip Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim4InitPoseStep, "Primitive4 Initial Pose Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim4EndStep, "Primitive4 End Step",
                )

#undef CREATE_STEP

}
