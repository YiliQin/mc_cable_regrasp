#pragma once

#include <iostream>
#include <Eigen/Core>

#include "mc_cable_regrasp_controller.h"

namespace mc_control
{

struct Prim14Step
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        Prim14Step(const std::string & name);
        virtual ~Prim14Step() {};
        Prim14Step * update(MCCableRegraspController & ctl);
    protected:
        virtual void __init(MCCableRegraspController & ctl) = 0;
        virtual Prim14Step * __update(MCCableRegraspController & ctl) = 0;
    public:
        bool first_call = true;
        std::string name;
};

#define CREATE_STEP(NAME, DESC, MEMBERS)\
struct NAME : public Prim14Step\
{\
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW\
    NAME() : Prim14Step(DESC) {}\
    virtual void __init(MCCableRegraspController & ctl) override;\
    virtual Prim14Step * __update(MCCableRegraspController & ctl) override;\
    MEMBERS\
};

CREATE_STEP(Prim14InitStep, "Primitive14 Initialization Step",
                bool stepByStep_ = true;
                )

CREATE_STEP(Prim14MidPointStep, "Primitive14 To Middle Point Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim14PreGraspStep, "Primitive14 Prepare Grasp Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim14OpenGripperStep, "Primitive14 Open Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim14GraspStep, "Primitive14 Grasp Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim14CloseGripperStep, "Primitive14 Close Gripper Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim14InitPoseStep, "Primitive14 Initial Pose Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim14EndStep, "Primitive14 End Step",
                )

#undef CREATE_STEP

}
