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

int BasicPrimitive::getID()
{
    return primitiveID;
}

std::string BasicPrimitive::getDes()
{
    return primitiveDes;
}

void BasicPrimitive::printDes()
{
    std::cout << primitiveID << std::endl;
    std::cout << primitiveDes << std::endl;
}

}
