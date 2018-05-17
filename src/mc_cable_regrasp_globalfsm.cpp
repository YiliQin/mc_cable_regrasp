#include "mc_cable_regrasp_globalfsm.h"
#include "mc_cable_regrasp_primitive1.h"
#include "mc_cable_regrasp_primitive2.h"
#include "mc_cable_regrasp_primitive3.h"
#include "mc_cable_regrasp_primitive4.h"
#include "mc_cable_regrasp_primitive5.h"

namespace mc_control
{

GlobalTestStep::GlobalTestStep(const std::string & name)
    : name(name)
{

}

GlobalTestStep * GlobalTestStep::update(MCCableRegraspController & ctl)
{
    if(first_call)
    {
        __init(ctl);
        first_call = false;
        return this;
    }

    return __update(ctl);
}

/////////////////////////////////////////////////////////////
//  Initialization Step
////////////////////////////////////////////////////////////

void InitStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Global: InitSetp: __init()." << std::endl;
    ctl.stepByStep = stepByStep_;
}

GlobalTestStep * InitStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Global: InitStep: __update()." << std::endl;
    
    //return this;

    ctl.neglectFctInp = ctl.neglectFctInp;
    return new InitialPoseStep;
}

/////////////////////////////////////////////////////////////
//  To Initial Pose Step
////////////////////////////////////////////////////////////

void InitialPoseStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Global: InitialPoseStep: __init()." << std::endl;
    ctl.stepByStep = stepByStep_;
}

GlobalTestStep * InitialPoseStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Global: InitialPoseStep: __update()." << std::endl;
    //return this;
    ctl.neglectFctInp = ctl.neglectFctInp;
    
    return new PlannerRunStep;
}

/////////////////////////////////////////////////////////////
//  Planner Run Step
////////////////////////////////////////////////////////////

void PlannerRunStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Global: PlannerRunStep: __init()." << std::endl;
    ctl.stepByStep = stepByStep_;
}

GlobalTestStep * PlannerRunStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Global: PlannerRunStep: __update()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    PrimInfo primInfo;

    // Test data 1
    primInfo.name = "Primitive5";
    primInfo.primNum = 5;
    primInfo.parNum = 1;
    primInfo.par1 = 0.6;
    primInfo.par2 = 0.0;
    ctl.quePrim.push(primInfo);     

    // Test data 2
    primInfo.name = "Primitive1";
    primInfo.primNum = 1;
    primInfo.parNum = 1;
    primInfo.par1 = 0.3;
    primInfo.par2 = 0.0;
    ctl.quePrim.push(primInfo);     

    // Test data 3
    primInfo.name = "Primitive4";
    primInfo.primNum = 4;
    primInfo.parNum = 0;
    primInfo.par1 = -0.0;
    primInfo.par2 = 0.0;
    ctl.quePrim.push(primInfo);     

    // Test data 4
    primInfo.name = "Primitive2";
    primInfo.primNum = 2;
    primInfo.parNum = 1;
    primInfo.par1 = 0.3;
    primInfo.par2 = 0.0;
    ctl.quePrim.push(primInfo);     

    // Test data 5
    primInfo.name = "Primitive3";
    primInfo.primNum = 3;
    primInfo.parNum = 0;
    primInfo.par1 = -0.0;
    primInfo.par2 = 0.0;
    ctl.quePrim.push(primInfo); 

    // Test data 6
    primInfo.name = "Primitive1";
    primInfo.primNum = 1;
    primInfo.parNum = 1;
    primInfo.par1 = 0.3;
    primInfo.par2 = 0.0;
    ctl.quePrim.push(primInfo);     

    // Test data 7
    primInfo.name = "Primitive4";
    primInfo.primNum = 4;
    primInfo.parNum = 0;
    primInfo.par1 = -0.0;
    primInfo.par2 = 0.0;
    ctl.quePrim.push(primInfo);     

    // Test data 8
    primInfo.name = "Primitive2";
    primInfo.primNum = 2;
    primInfo.parNum = 1;
    primInfo.par1 = 0.3;
    primInfo.par2 = 0.0;
    ctl.quePrim.push(primInfo);     

    // Test data 9
    primInfo.name = "Primitive3";
    primInfo.primNum = 3;
    primInfo.parNum = 0;
    primInfo.par1 = -0.0;
    primInfo.par2 = 0.0;
    ctl.quePrim.push(primInfo);     

    LOG_SUCCESS("The size number of the queue:" << ctl.quePrim.size()); 
    //return this;
    return new QueueReadStep;
}

/////////////////////////////////////////////////////////////
//  Queue Read Step
////////////////////////////////////////////////////////////

void QueueReadStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Global: PlannerRunStep: __init()." << std::endl;

    ctl.stepByStep = stepByStep_;
}

GlobalTestStep * QueueReadStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Global: PlannerRunStep: __update()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    if (ctl.quePrim.empty() == false)
    {
        LOG_SUCCESS("Read out primitive:" << ctl.quePrim.front().name << "," << ctl.quePrim.front().primNum)
        ctl.primName = ctl.quePrim.front().name;
        ctl.primType = ctl.quePrim.front().primNum;
        ctl.primParNum = ctl.quePrim.front().parNum;
        ctl.primPar1 = ctl.quePrim.front().par1;
        ctl.primPar2 = ctl.quePrim.front().par2;
        ctl.quePrim.pop();
        LOG_SUCCESS("Run pop(), left primitive:" << ctl.quePrim.size());
        return new ResExeStep;
    }
    else 
    {
        return new EndStep;
    }
}

/////////////////////////////////////////////////////////////
//  Execute Planner Results Step
////////////////////////////////////////////////////////////

void ResExeStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Global: ResExeStep: __init()." << std::endl;

    switch (ctl.primType)
    {
        case 1:
            LOG_SUCCESS("Execuating Primitive1.");
            ctl.prim1->reset();
            ctl.prim1->slideLen = ctl.primPar1;
            break;
        case 2:
            LOG_SUCCESS("Execuating Primitive2.");
            ctl.prim2->reset();
            ctl.prim2->slideLen = ctl.primPar1;
            break;
        case 3:
            LOG_SUCCESS("Execuating Primitive3.");
            ctl.prim3->reset();
            break;
        case 4:
            LOG_SUCCESS("Execuating Primitive4.");
            ctl.prim4->reset();
            break;
        case 5:
            LOG_SUCCESS("Execuating Primitive5.");
            ctl.prim5->reset();
            ctl.prim5->disBetHands = ctl.primPar1;
            break;
        default:
            break;
    }
}

GlobalTestStep * ResExeStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Global: ResExeStep: __update()." << std::endl;
    //ctl.neglectFctInp = ctl.neglectFctInp;

    switch (ctl.primType)
    {
        case 1:
            ctl.prim1->prim1_fsm_run(ctl);
            if (ctl.prim1->finish == true)
            {
                ctl.primType = 0;
                ctl.prim1->finish = false;
                if (ctl.quePrim.empty() == false)
                    return new QueueReadStep;
                else
                    return new EndStep;
            }
            break;
        case 2:
            ctl.prim2->prim2_fsm_run(ctl);
            if (ctl.prim2->finish == true)
            {
                ctl.primType = 0;
                ctl.prim2->finish = false;
                if (ctl.quePrim.empty() == false)
                    return new QueueReadStep;
                else
                    return new EndStep;
            }
            break;
        case 3:
            ctl.prim3->prim3_fsm_run(ctl);
            if (ctl.prim3->finish == true)
            {
                ctl.primType = 0;
                ctl.prim3->finish = false;
                if (ctl.quePrim.empty() == false)
                    return new QueueReadStep;
                else
                    return new EndStep;
            }
            break;
        case 4:
            ctl.prim4->prim4_fsm_run(ctl);
            if (ctl.prim4->finish == true)
            {
                ctl.primType = 0;
                ctl.prim4->finish = false;
                if (ctl.quePrim.empty() == false)
                    return new QueueReadStep;
                else
                    return new EndStep;
            }
            break;
        case 5:
            ctl.prim5->prim5_fsm_run(ctl);
            if (ctl.prim5->finish == true)
            {
                ctl.primType = 0;
                ctl.prim5->finish = false;
                if (ctl.quePrim.empty() == false)
                    return new QueueReadStep;
                else
                    return new EndStep;
            }
            break;
        default:
            return new EndStep;
            break;
    }

    return this;
}

/////////////////////////////////////////////////////////////
//  End Step
////////////////////////////////////////////////////////////

void EndStep::__init(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Global: EndStep: __init()." << std::endl;
    ctl.neglectFctInp = ctl.neglectFctInp;
}

GlobalTestStep * EndStep::__update(MCCableRegraspController & ctl)
{
    // For test.
    //std::cout << "Global: EndStep: update" << std::endl;
    ctl.neglectFctInp = ctl.neglectFctInp;

    return nullptr;
}

}
