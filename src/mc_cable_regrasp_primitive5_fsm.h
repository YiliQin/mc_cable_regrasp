#pragma once

#include <iostream>
#include <Eigen/Core>

#include "mc_cable_regrasp_controller.h"

namespace mc_control
{

struct Prim5Step
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        Prim5Step(const std::string & name);
        virtual ~Prim5Step() {};
        Prim5Step * update(MCCableRegraspController & ctl);
    protected:
        virtual void __init(MCCableRegraspController & ctl) = 0;
        virtual Prim5Step * __update(MCCableRegraspController & ctl) = 0;
    public:
        bool first_call = true;
        std::string name;
};

#define CREATE_STEP(NAME, DESC, MEMBERS)\
struct NAME : public Prim5Step\
{\
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW\
    NAME() : Prim5Step(DESC) {}\
    virtual void __init(MCCableRegraspController & ctl) override;\
    virtual Prim5Step * __update(MCCableRegraspController & ctl) override;\
    MEMBERS\
};

CREATE_STEP(Prim5InitStep, "Primitive5 Initialization Step",
                bool stepByStep_ = true;
                )

CREATE_STEP(Prim5PreGraspStep, "Primitive5 Prepare Grasp Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim5GraspStep, "Primitive5 Grasp Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim5HangStep, "Primitive5 Hang Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim5InitPoseStep, "Primitive5 Initial Pose Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim5EndStep, "Primitive5 End Step",
                )

#undef CREATE_STEP

}
