#pragma once

#include <queue>

#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/CoMTask.h>

// ROS
#include <mc_rtc/ros.h>
#include <ros/ros.h>
#include <whycon_lshape/WhyConLShapeMsg.h>
#include <mutex>
#include <thread>

namespace tf2_ros
{
  class TransformBroadcaster;
}

#include <mc_control/api.h>

namespace mc_control
{

struct GlobalTestStep;
struct InitStep;
struct Primitive1;
struct Primitive2;
struct Primitive3;
struct Primitive4;
struct Primitive5;
struct Primitive6;
struct Primitive11;
struct Primitive12;
struct Primitive13;
struct Primitive14;

struct PrimInfo
{
    std::string name = "None";
    int primNum = 0;

    int parNum = 0;
    double par1 = 0.0;
    double par2 = 0.0;
};

struct MC_CONTROL_DLLAPI MCCableRegraspController : public MCController
{
    public:
        MCCableRegraspController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt, const mc_rtc::Configuration & config);
        
        // User function.
        void global_fsm_run();
        // Virtual function.
        virtual bool run() override;
        virtual void reset(const ControllerResetData & reset_data) override;
        virtual bool read_msg(std::string & msg) override;
        virtual bool read_write_msg(std::string & msg, std::string & out) override;

    public:
        /** Keep the configuration available */
        mc_rtc::Configuration config_;

        // FLAGS: set before controller usage
        bool FLAG_SIMULATION_VREP = true;
        // for test
        bool cmdContinue = false;
        bool prim6ContinueS1 = false; 
        bool prim6ContinueS2 = false;
        bool prim6ContinueS3 = false;
        // True for the initial reset
        bool initial_reset = true;

        // Primitive.
        GlobalTestStep * step = nullptr;
        Primitive1 * prim1 = nullptr;
        Primitive2 * prim2 = nullptr;
        Primitive3 * prim3 = nullptr;
        Primitive4 * prim4 = nullptr;
        Primitive5 * prim5 = nullptr;
        Primitive6 * prim6 = nullptr;
        Primitive11 * prim11 = nullptr;
        Primitive12 * prim12 = nullptr;
        Primitive13 * prim13 = nullptr;
        Primitive14 * prim14 = nullptr;

        std::queue<PrimInfo> quePrim;
        
        // For global FSM.
        bool stepByStep = false;
        bool paused = false;
        
        // Primitive information
        std::string primName = "";
        int primType = 0;
        int primParNum = 0;
        double primPar1 = 0.0;
        double primPar2 = 0.0;
        
        // ROS Whycon vision
        void ros_spinner();
        void lShapeCallback(const whycon_lshape::WhyConLShapeMsg & msg);
        std::shared_ptr<ros::NodeHandle> m_nh_;
        std::thread m_ros_spinner_;
        ros::Subscriber l_shape_sub_;

        struct LShape
        {
        bool initialized = true;
        sva::PTransformd pos;
        sva::PTransformd world_pos;
        void update(const sva::PTransformd & in, const sva::PTransformd & X_0_head)
        {
          pos = in;
          world_pos = pos * X_0_head;
          last_update_ = 0;
        }
        void tick(double dt) { last_update_ += dt; initialized = last_update_ < 0.5; }
        private:
        double last_update_ = 0;
        };
        std::unordered_map<std::string, LShape> lshapes;

        std::thread lshapes_simulation_th_;
        mutable std::mutex lshapes_mut_;
        bool active_ = true;

        // camera
        std::string camera_body;
        std::string camera_pan_joint;
        std::string camera_tilt_joint;
        // marker
        sva::PTransformd marker1_pos;
        sva::PTransformd marker2_pos;

        // Task.
        std::shared_ptr<mc_tasks::EndEffectorTask> lh2Task;
        std::shared_ptr<mc_tasks::EndEffectorTask> rh2Task;
        std::shared_ptr<mc_tasks::EndEffectorTask> chestTask;
        std::shared_ptr<mc_tasks::CoMTask> comTask;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_caster;

        // Visual sservoing task
        const sva::PTransformd & getCameraPose() const;
        const sva::PTransformd & X_camera_marker(const std::string & name) const;
        const sva::PTransformd & X_0_marker(const std::string & name) const;
        // 
        unsigned int seq;

        // temp
        std::map<std::string, sva::ForceVecd> wrenches;
        std::map<std::string, sva::ForceVecd> wrenches_cleared;

        struct WrenchData
        {
            Eigen::Vector3d com;
            sva::ForceVecd offset;
            sva::ForceVecd unidentified_offset;
            double gravityForce;
            double length;
        };
        WrenchData wrench_data;

        // Constraints.
        mc_solver::CollisionsConstraint barCollisionConstraint;
};

}

//SIMPLE_CONTROLLER_CONSTRUCTOR("CableRegrasp", mc_control::MCCableRegraspController)
