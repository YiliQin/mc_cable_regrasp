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

}
