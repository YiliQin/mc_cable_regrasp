#pragma once

#include <Eigen/Core>
#include <string>

namespace mc_control
{

struct BasicPrimitive
{
    public:
        BasicPrimitive() {};
        BasicPrimitive(int primitiveID, std::string primitiveDes);
        ~BasicPrimitive();

        int getID();
        std::string getDes();
        
        // For test.
        void printDes();

    //protected:
        // ID and description 
        int primitiveID = 0;
        std::string primitiveDes = "Please Input Description.";
};


}
