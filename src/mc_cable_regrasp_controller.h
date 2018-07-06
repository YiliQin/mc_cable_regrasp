#pragma once

#include <queue>

#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/CoMTask.h>


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
        MCCableRegraspController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);
        
        // User function.
        void global_fsm_run();
        // Virtual function.
        virtual bool run() override;
        virtual void reset(const ControllerResetData & reset_data) override;
        virtual bool read_msg(std::string & msg) override;
        virtual bool read_write_msg(std::string & msg, std::string & out) override;

    public:
        // for test
        bool cmdContinue = false;

        // Primitive.
        GlobalTestStep * step = nullptr;
        Primitive1 * prim1 = nullptr;
        Primitive2 * prim2 = nullptr;
        Primitive3 * prim3 = nullptr;
        Primitive4 * prim4 = nullptr;
        Primitive5 * prim5 = nullptr;
        std::queue<PrimInfo> quePrim;
        
        // FLAG - For global FSM.
        bool stepByStep = false;
        bool paused = false;
        
        // Primitive information
        std::string primName = "";
        int primType = 0;
        int primParNum = 0;
        double primPar1 = 0.0;
        double primPar2 = 0.0;

        // Task.
        std::shared_ptr<mc_tasks::EndEffectorTask> lh2Task;
        std::shared_ptr<mc_tasks::EndEffectorTask> rh2Task;
        std::shared_ptr<mc_tasks::EndEffectorTask> chestTask;
        std::shared_ptr<mc_tasks::CoMTask> comTask;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_caster;
        // 
        unsigned int seq;
        // Constraints.
        mc_solver::CollisionsConstraint barCollisionConstraint;
};

}

//SIMPLE_CONTROLLER_CONSTRUCTOR("CableRegrasp", mc_control::MCCableRegraspController)
