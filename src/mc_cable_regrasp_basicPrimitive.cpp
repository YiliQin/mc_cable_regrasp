#include <iostream>

#include "mc_cable_regrasp_basicPrimitive.h"

namespace mc_control
{

BasicPrimitive::BasicPrimitive(int primitiveID, std::string primitiveDes)
    : primitiveID(primitiveID), primitiveDes(primitiveDes)
{
}

BasicPrimitive::~BasicPrimitive()
{
}

void BasicPrimitive::reset()
{
    primitiveID = 0;
    primitiveDes = "Please Input Description.";
}

void BasicPrimitive::prim_config(double par1, double par2, MCCableRegraspController & ctl)
{
    par1 = par1;
    par2 = par2;
    ctl.neglectFctInp = ctl.neglectFctInp;
}

void BasicPrimitive::prim_fsm_run(MCCableRegraspController & ctl)
{
    ctl.neglectFctInp = ctl.neglectFctInp;
}

int BasicPrimitive::getID()
{
    return primitiveID;
}

std::string BasicPrimitive::getDes()
{
    return primitiveDes;
}

bool BasicPrimitive::get_finish()
{
    return finish;
}

void BasicPrimitive::idle()
{
    finish = false;
}

void BasicPrimitive::set_stepByStep(bool stepByStep_)
{
    stepByStep = stepByStep_;
}

}
