#include "mc_cable_regrasp_controller.h"
#include "mc_cable_regrasp_globalfsm.h"
#include "mc_cable_regrasp_primitive1.h"
#include "mc_cable_regrasp_primitive2.h"
#include "mc_cable_regrasp_primitive3.h"
#include "mc_cable_regrasp_primitive4.h"
#include "mc_cable_regrasp_primitive5.h"
#include "mc_cable_regrasp_primitive6.h"

#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>
#include <mc_rbdyn/RobotLoader.h>

#ifdef MC_RTC_HAS_ROS
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#endif

namespace
{
    inline std::stringstream& operator>>(std::stringstream & ss, Eigen::Matrix3d & m)
    {
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
        ss >> m(i,j);
        }
    }
    return ss;
    }
    inline std::stringstream& operator>>(std::stringstream & ss, Eigen::Vector3d & v)
    {
        for(int i = 0; i < 3; ++i)
        {
          ss >> v(i);
        }
        return ss;
    }
}

namespace mc_control
{

MCCableRegraspController::MCCableRegraspController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController({robot_module,
        mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("bar")),
        mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("wall-holder")),
        mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("ground"))},
        dt),
        tf_caster(0), seq(0), barCollisionConstraint(robots(), 0, 1, solver().dt())
{
    // modify the model position and orientation error in Choreonoid
    //robots().robot(1).posW({sva::RotZ(M_PI)*sva::RotX(M_PI/2), {-0.27, 0., 2*0.54 + 0.01}});
    //robots().robot(2).posW({sva::RotZ(M_PI), {0.45, 0., 0.75}});

    qpsolver->addConstraintSet(contactConstraint);
    qpsolver->addConstraintSet(dynamicsConstraint);
    if(robot().name() == "hrp2_drc")
    {
        /* Set more restrictive auto collision constraints */
        selfCollisionConstraint.reset();
        selfCollisionConstraint.addCollisions(solver(), {
            mc_rbdyn::Collision("LARM_LINK3", "BODY", 0.1, 0.05, 0.),
            mc_rbdyn::Collision("LARM_LINK4", "BODY", 0.1, 0.05, 0.),
            mc_rbdyn::Collision("LARM_LINK5", "BODY", 0.1, 0.05, 0.),
            mc_rbdyn::Collision("RARM_LINK3", "BODY", 0.1, 0.05, 0.),
            mc_rbdyn::Collision("RARM_LINK4", "BODY", 0.1, 0.05, 0.),
            mc_rbdyn::Collision("RARM_LINK5", "BODY", 0.1, 0.05, 0.),
            mc_rbdyn::Collision("RARM_LINK3", "CHEST_LINK0", 0.1, 0.05, 0.),
            mc_rbdyn::Collision("RARM_LINK4", "CHEST_LINK0", 0.1, 0.05, 0.),
            mc_rbdyn::Collision("RARM_LINK5", "CHEST_LINK0", 0.1, 0.05, 0.),
            mc_rbdyn::Collision("RARM_LINK4", "CHEST_LINK1", 0.1, 0.05, 0.),
            mc_rbdyn::Collision("RARM_LINK5", "CHEST_LINK1", 0.1, 0.05, 0.),
            mc_rbdyn::Collision("LARM_LINK3", "CHEST_LINK0", 0.1, 0.05, 0.),
            mc_rbdyn::Collision("LARM_LINK4", "CHEST_LINK0", 0.1, 0.05, 0.),
            mc_rbdyn::Collision("LARM_LINK5", "CHEST_LINK0", 0.1, 0.05, 0.),
            mc_rbdyn::Collision("LARM_LINK4", "CHEST_LINK1", 0.1, 0.05, 0.),
            mc_rbdyn::Collision("LARM_LINK5", "CHEST_LINK1", 0.1, 0.05, 0.)
        });
    }
    qpsolver->addConstraintSet(selfCollisionConstraint);
    barCollisionConstraint.addCollisions(solver(), {
        //mc_rbdyn::Collision("LARM_LINK6", "bar", 0.1, 0.05, 0.),
        //mc_rbdyn::Collision("RARM_LINK6", "bar", 0.1, 0.05, 0.)
    });
    solver().addConstraintSet(barCollisionConstraint);
    qpsolver->addTask(postureTask.get());
    qpsolver->setContacts({
        mc_rbdyn::Contact(robots(), 0, 3, "LFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), 0, 3, "RFullSole", "AllGround"),
        //mc_rbdyn::Contact(robots(), "Butthock", "AllGround")
    });

    lh2Task.reset(new mc_tasks::EndEffectorTask("LARM_LINK6", robots(), robots().robotIndex(), 2.0, 1e5));
    rh2Task.reset(new mc_tasks::EndEffectorTask("RARM_LINK6", robots(), robots().robotIndex(), 2.0, 1e5));
    chestTask.reset(new mc_tasks::EndEffectorTask("CHEST_LINK1", robots(), robots().robotIndex(), 1.0, 1e6));
    solver().addTask(lh2Task);
    solver().addTask(rh2Task);
    solver().addTask(chestTask);

    comTask.reset(new mc_tasks::CoMTask(robots(), robots().robotIndex()));
    solver().addTask(comTask);

    #ifdef MC_RTC_HAS_ROS
    if(mc_rtc::ROSBridge::get_node_handle())
    {
    tf_caster.reset(new tf2_ros::TransformBroadcaster());
    }
    #endif

    LOG_SUCCESS("MCCableRegraspController init done")
}

void MCCableRegraspController::reset(const ControllerResetData & reset_data)
{
    MCController::reset(reset_data);
    qpsolver->setContacts({
        mc_rbdyn::Contact(robots(), 0, 3, "LFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), 0, 3, "RFullSole", "AllGround"),
        //mc_rbdyn::Contact(robots(), "Butthock", "AllGround")
    });

    lh2Task->reset();
    rh2Task->reset();
    chestTask->reset();
    comTask->reset();

    // Start the global FSM.
    step = new InitStep();
    // Create primitive object.
    prim1 = new Primitive1(1, "Primivive 1", *this); 
    prim2 = new Primitive2(2, "Primivive 2", *this); 
    prim3 = new Primitive3(3, "Primivive 3", *this); 
    prim4 = new Primitive4(4, "Primivive 4", *this); 
    prim5 = new Primitive5(5, "Primivive 5", *this); 
    prim6 = new Primitive6(6, "Primivive 6", *this); 
}

bool MCCableRegraspController::run()
{
    bool ret = MCController::run();
    if(ret)
    {
        #ifdef MC_RTC_HAS_ROS
        if(tf_caster)
        {
            geometry_msgs::TransformStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "BODY";
            msg.header.seq = ++seq;
            msg.child_frame_id = "bci_left_hand_target";
            sva::PTransformd X = lh2Task->get_ef_pose();
            Eigen::Vector4d q = Eigen::Quaterniond(X.rotation()).inverse().coeffs();
            const Eigen::Vector3d & t = X.translation();

            msg.transform.translation.x = t.x();
            msg.transform.translation.y = t.y();
            msg.transform.translation.z = t.z();

            msg.transform.rotation.w = q.w();
            msg.transform.rotation.x = q.x();
            msg.transform.rotation.y = q.y();
            msg.transform.rotation.z = q.z();
            tf_caster->sendTransform(msg);
        }
        if(tf_caster)
        {
            geometry_msgs::TransformStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "BODY";
            msg.header.seq = ++seq;
            msg.child_frame_id = "bci_right_hand_target";
            sva::PTransformd X = rh2Task->get_ef_pose();
            Eigen::Vector4d q = Eigen::Quaterniond(X.rotation()).inverse().coeffs();
            const Eigen::Vector3d & t = X.translation();

            msg.transform.translation.x = t.x();
            msg.transform.translation.y = t.y();
            msg.transform.translation.z = t.z();

            msg.transform.rotation.w = q.w();
            msg.transform.rotation.x = q.x();
            msg.transform.rotation.y = q.y();
            msg.transform.rotation.z = q.z();
            tf_caster->sendTransform(msg);
        }
        #endif

    // Put run() here.
    global_fsm_run();
    }
    return ret;
}

/////////////////////////////////////////////////////////////////////////////
//  Global System
/////////////////////////////////////////////////////////////////////////////

void MCCableRegraspController::global_fsm_run()
{
    if (!paused)
    {
        if (step)
        {
            auto nstep = step->update(*this);
            if (nstep != step)
            {
                if (stepByStep == true)
                {
                
                }
                else if (stepByStep == false)
                {
                    LOG_SUCCESS("Completed: " << step->name);
                }
                delete step; 
                step = nstep;
                //// Control the FSM step.
                //paused = stepByStep;
                if (step == nullptr)
                {
                    LOG_SUCCESS("Completed: Global FSM");
                }
            }
        } 
    }
}

/////////////////////////////////////////////////////////////////////////////
//  System Interface
/////////////////////////////////////////////////////////////////////////////

bool MCCableRegraspController::read_msg(std::string & msg)
{
    if(msg.size() == 0)
    {
        return false;
    }
    std::stringstream ss;
    ss << msg;
    std::string token;
    ss >> token;

    if(token == "SetEFPose")
    {
        std::string ef;
        ss >> ef;
        Eigen::Matrix3d ori;
        ss >> ori;
        Eigen::Vector3d t;
        ss >> t;
    if(ef == "lh2")
    {
        lh2Task->set_ef_pose(sva::PTransformd(ori.inverse(), t));
    }
    else if(ef == "rh2")
    {
        rh2Task->set_ef_pose(sva::PTransformd(ori.inverse(), t));
    }
    else
    {
        LOG_ERROR("Do not know how to set ef pose for: " << ef)
    }
    }
    else if(token == "SetPanTilt")
    {
        double pan = 0; double tilt = 0;
        ss >> pan >> tilt;
        set_joint_pos("HEAD_JOINT0", pan);
        set_joint_pos("HEAD_JOINT1", tilt); 
    }
    else if(token == "Continue")
    {
        cmdContinue = true;
        return true;
    }
    else if(token == "OpenRightGripper")
    {
        auto gripper = grippers["r_gripper"].get();
        gripper->setTargetQ({0.5});
        return true;  
    }
    else if(token == "CloseRightGripper")
    {
        auto gripper = grippers["r_gripper"].get();
        gripper->setTargetQ({-0.7});
        return true;
    }
    else if(token == "OpenLeftGripper")
    {
        auto gripper = grippers["l_gripper"].get();
        gripper->setTargetQ({0.5});
        return true;  
    }
    else if(token == "CloseLeftGripper")
    {
        auto gripper = grippers["l_gripper"].get();
        gripper->setTargetQ({-0.7});
        return true;
    }
    else
    {
        LOG_ERROR("BCISelfInteract controller cannot handle this token: " << token)
        return false;
    }
    return true;
}

bool MCCableRegraspController::read_write_msg(std::string & msg, std::string & out)
{
    if(msg.size() == 0)
    {
        return false;
    }
    std::stringstream ss;
    ss << msg;
    std::string token;
    ss >> token;
    if(token == "Read")
    {
        sva::PTransformd out_X = sva::PTransformd::Identity();
        Eigen::Matrix4d out_m = Eigen::Matrix4d::Identity();
        std::string signal;
        ss >> signal;
        if(signal == "gaze")
        {
            out_X = robot().mbc().bodyPosW[robot().bodyIndexByName("HEAD_LINK1")];
        }
        else if(signal == "waist")
        {
            out_X = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];
        }
        else if(signal == "lh2")
        {
            out_X = lh2Task->get_ef_pose();
        }
        else if(signal == "rh2")
        {
            out_X = rh2Task->get_ef_pose();
        }
        else
        {
            LOG_ERROR("Do not know how to read signal: " << signal)
        }
        Eigen::Matrix3d ori = Eigen::Quaterniond(out_X.rotation()).inverse().toRotationMatrix();
        for(int i = 0; i < 3; ++i)
        {
            for(int j = 0; j < 3; ++j)
            {
                out_m(i,j) = ori(i,j);
            }
            out_m(i, 3) = out_X.translation()(i);
        }
        std::stringstream ss_out;
        ss_out << out_m;
        out = ss_out.str();
    }
    else
    {
        LOG_ERROR("BCISelfInteract controller cannot handle this token: " << token)
        return false;
    }
    return true;
}

}

SIMPLE_CONTROLLER_CONSTRUCTOR("CableRegrasp", mc_control::MCCableRegraspController)
