#pragma once
#include <iostream>
#include "mc_cable_regrasp_controller.h"

namespace mc_control
{

struct GlobalTestStep
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        GlobalTestStep(const std::string & name);
        virtual ~GlobalTestStep() {};
        GlobalTestStep * update(MCCableRegraspController & ctl);
    protected:
        virtual void __init(MCCableRegraspController & ctl) = 0;
        virtual GlobalTestStep * __update(MCCableRegraspController & ctl) = 0;
    public:
        bool first_call = true;
        std::string name;
};

#define CREATE_STEP(NAME, DESC, MEMBERS)\
struct NAME : public GlobalTestStep\
{\
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW\
    NAME() : GlobalTestStep(DESC) {}\
    virtual void __init(MCCableRegraspController & ctl) override;\
    virtual GlobalTestStep * __update(MCCableRegraspController & ctl) override;\
    MEMBERS\
};

CREATE_STEP(InitStep, "Initialization Step",
                bool stepByStep_ = true;
                )

CREATE_STEP(InitialPoseStep, "To Initial Pose Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(PlannerRunStep, "Planner Run Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(QueueReadStep, "Queue Read Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(ResExeStep, "Execute Planner Results Step",
                bool stepByStep_ = true;
                int cnt = 0;
                )

CREATE_STEP(EndStep, "End Step",
                )

#undef CREATE_STEP

}
