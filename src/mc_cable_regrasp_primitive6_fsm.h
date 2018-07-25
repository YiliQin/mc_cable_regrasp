#pragma once

#include <iostream>
#include <Eigen/Core>

#include "mc_cable_regrasp_controller.h"

namespace mc_control
{

struct Prim6Step
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        Prim6Step(const std::string & name);
        virtual ~Prim6Step() {};
        Prim6Step * update(MCCableRegraspController & ctl);
    protected:
        virtual void __init(MCCableRegraspController & ctl) = 0;
        virtual Prim6Step * __update(MCCableRegraspController & ctl) = 0;
    public:
        bool first_call = true;
        std::string name;
};

#define CREATE_STEP(NAME, DESC, MEMBERS)\
struct NAME : public Prim6Step\
{\
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW\
    NAME() : Prim6Step(DESC) {}\
    virtual void __init(MCCableRegraspController & ctl) override;\
    virtual Prim6Step * __update(MCCableRegraspController & ctl) override;\
    MEMBERS\
};

CREATE_STEP(Prim6InitStep, "Primitive6 Initialization Step",
                bool stepByStep_ = true;
                )

CREATE_STEP(Prim6ToInterPosStep, "Primitive6 To Intermediate Posture Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim6ToPrePosStep, "Primitive6 To Preparing Installation Position Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim6InsStep, "Primitive6 Installation Step",
                bool stepByStep_ = true;
                int cntRun = 0;
                )

CREATE_STEP(Prim6InitPoseStep, "Primitive6 Initial Pose Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(Prim6EndStep, "Primitive6 End Step",
                )

#undef CREATE_STEP

}
