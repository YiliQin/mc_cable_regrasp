#pragma once

#include <iostream>
#include <queue>
#include <Eigen/Core>
#include <utility>

namespace mc_control
{

struct Prim1Traj1
{
    public:
        Prim1Traj1();
        Prim1Traj1(const Eigen::Vector3d spos, const Eigen::Vector3d epos, const Eigen::Matrix3d srot, const Eigen::Matrix3d erot, std::size_t nr_points);
        ~Prim1Traj1();
        // 
        void gen_traj(); 
        std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> pop();
    private:
        // parameters for trajectory 
        Eigen::Vector3d startPos;
        Eigen::Vector3d endPos;
        Eigen::Matrix3d startRot;
        Eigen::Matrix3d endRot;
        std::size_t nr_points;
        // queue
        std::queue<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>> queue;
};

}
