#include "mc_cable_regrasp_linear_trajectory.h"

namespace mc_control
{

LinearTrajectory::LinearTrajectory()
{
}

LinearTrajectory::LinearTrajectory(const Eigen::Vector3d spos, const Eigen::Vector3d epos, const Eigen::Matrix3d srot, const Eigen::Matrix3d erot, std::size_t nr_points)
    : startPos(spos), endPos(epos), startRot(srot), endRot(erot), nr_points(nr_points)
{
    gen_traj();
}

LinearTrajectory::~LinearTrajectory()
{
}

void LinearTrajectory::gen_traj()
{
    double diffy;
    diffy = (endPos[1] - startPos[1])/(double)nr_points;
    // position of y direction
    Eigen::Vector3d pos = startPos;
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();
    for (std::size_t i = 0; i < nr_points; i++)
    {
        // positon
        pos[0] = pos[0];
        pos[1] = pos[1] + diffy;  
        pos[2] = pos[2];
        // velocity
        vel[0] = 0.0;
        vel[1] = 0.1;
        vel[2] = 0.0;
        // acceleration
        acc[0] = 0.0;
        acc[1] = 0.0;
        acc[2] = 0.0;
        // rotation
        rot = startRot;

        queue.push(std::make_tuple(pos, vel, acc, rot));
    }
    //// for test
    //std::cout << "The size of trajectory queue: " << queue.size() << std::endl;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> LinearTrajectory::pop()
{
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> quad;
    if (queue.empty())
    {
        Eigen::Vector3d zero = Eigen::Vector3d::Zero();
        quad = std::make_tuple(endPos, zero, zero, endRot);
    }
    else 
    {
        quad = queue.front(); 
        queue.pop();
    }

    return quad;
}

}
