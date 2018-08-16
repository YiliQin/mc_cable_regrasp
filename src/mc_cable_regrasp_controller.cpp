#include "mc_cable_regrasp_controller.h"
#include "mc_cable_regrasp_globalfsm.h"
#include "mc_cable_regrasp_primitive1.h"
#include "mc_cable_regrasp_primitive2.h"
#include "mc_cable_regrasp_primitive3.h"
#include "mc_cable_regrasp_primitive4.h"
#include "mc_cable_regrasp_primitive5.h"
#include "mc_cable_regrasp_primitive6.h"
#include "mc_cable_regrasp_primitive7.h"
#include "mc_cable_regrasp_primitive11.h"
#include "mc_cable_regrasp_primitive12.h"
#include "mc_cable_regrasp_primitive13.h"
#include "mc_cable_regrasp_primitive14.h"
#include "mc_cable_regrasp_primitive15.h"
#include "mc_cable_regrasp_primitive17.h"
#include "mc_cable_regrasp_primitive18.h"

#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>
#include <mc_rbdyn/RobotLoader.h>

#ifdef MC_RTC_HAS_ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
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

MCCableRegraspController::MCCableRegraspController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt, const mc_rtc::Configuration & config)
: MCController({robot_module,
        mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("bar")),
        mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("wall-holder")),
        mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("ground"))},
        dt),
        config_(config),
        m_nh_(mc_rtc::ROSBridge::get_node_handle()),
        tf_caster(0), seq(0), barCollisionConstraint(robots(), 0, 1, solver().dt())
{
    // modify the model position and orientation error in Choreonoid
    //robots().robot(1).posW({sva::RotZ(M_PI)*sva::RotX(M_PI/2), {-0.27, 0., 2*0.54 + 0.01}});
    //robots().robot(2).posW({sva::RotZ(M_PI), {0.45, 0., 0.75}});

    // for simulation
    FLAG_SIMULATION_VREP = config("Simulation");
    camera_body = static_cast<std::string>(config("camera_body"));

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
            mc_rbdyn::Collision("LARM_LINK5", "CHEST_LINK1", 0.1, 0.05, 0.),
            ////
            //mc_rbdyn::Collision("LARM_LINK6", "LLEG_LINK0", 0.1, 0.02, 0.),
            //mc_rbdyn::Collision("RARM_LINK6", "RLEG_LINK0", 0.1, 0.02, 0.),
            //mc_rbdyn::Collision("LARM_LINK6", "LLEG_LINK1", 0.1, 0.02, 0.),
            //mc_rbdyn::Collision("RARM_LINK6", "RLEG_LINK1", 0.1, 0.02, 0.),
            //mc_rbdyn::Collision("LARM_LINK6", "LLEG_LINK2", 0.1, 0.02, 0.),
            //mc_rbdyn::Collision("RARM_LINK6", "RLEG_LINK2", 0.1, 0.02, 0.)
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

    // Start the global FSM.
    step = new InitStep();
    // Create primitive object.
    prim1 = new Primitive1(1, "Primivive 1", *this); 
    prim2 = new Primitive2(2, "Primivive 2", *this); 
    prim3 = new Primitive3(3, "Primivive 3", *this); 
    prim4 = new Primitive4(4, "Primivive 4", *this); 
    prim5 = new Primitive5(5, "Primivive 5", *this); 
    prim6 = new Primitive6(6, "Primivive 6", *this); 
    prim7 = new Primitive7(7, "Primivive 7", *this); 
    prim11 = new Primitive11(11, "Primitive 11", *this);
    prim12 = new Primitive12(12, "Primitive 12", *this);
    prim13 = new Primitive13(13, "Primitive 13", *this);
    prim14 = new Primitive14(14, "Primitive 14", *this);
    prim15 = new Primitive15(15, "Primitive 15", *this);
    prim17 = new Primitive17(17, "Primitive 17", *this);
    prim18 = new Primitive18(18, "Primitive 18", *this);

    // system offset
    markerOffset << 0.0, 0.0, -0.17;
    gripperOffset << 0.0, 0.0, 0.20;
    compenOffset << 0.06, 0.0, 0.02;

    //
    #ifdef MC_RTC_HAS_ROS
    if(mc_rtc::ROSBridge::get_node_handle())
    {
    tf_caster.reset(new tf2_ros::TransformBroadcaster());
    }
    #endif

    LOG_SUCCESS("MCCableRegraspController init done")
}

void MCCableRegraspController::ros_spinner()
{
  ros::Rate rt(30);
  while(ros::ok() && active_)
  {
    ros::spinOnce();
    rt.sleep();
  }
}

void MCCableRegraspController::lShapeCallback(const whycon_lshape::WhyConLShapeMsg & msg)
{
  std::lock_guard<std::mutex> lock(lshapes_mut_);
  Eigen::Matrix3d camera_R_image;
  camera_R_image = sva::RotY(-M_PI/2)*sva::RotZ(M_PI/2);
  const sva::PTransformd& X_0_head = getCameraPose();
  for(const auto & s : msg.shapes)
  {
    Eigen::Vector3d pos = { s.pose.position.x,
                            s.pose.position.y,
                            s.pose.position.z };
    Eigen::Quaterniond q = { s.pose.orientation.w,
                             s.pose.orientation.x,
                             s.pose.orientation.y,
                             s.pose.orientation.z };
    if(!lshapes.count(s.name))
    {
      std::string name = s.name;
      gui()->addElement({"LShapes markers"},
                        mc_rtc::gui::Transform(s.name,
                                             [this,name]() { return X_0_marker(name); },
                                             [](const sva::PTransformd&){}));
    }
    lshapes[s.name].update({q.toRotationMatrix() * camera_R_image.transpose(), camera_R_image * pos}, X_0_head);
  }
}

const sva::PTransformd & MCCableRegraspController::getCameraPose() const
{
  return robot().bodyPosW(camera_body);
}

const sva::PTransformd & MCCableRegraspController::X_camera_marker(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(lshapes_mut_);
  return lshapes.at(name).pos;
}

const sva::PTransformd & MCCableRegraspController::X_0_marker(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(lshapes_mut_);
  return lshapes.at(name).world_pos;
}

void MCCableRegraspController::reset(const ControllerResetData & reset_data)
{    
    // 
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

    // for simulation
    if(initial_reset)
    {
      initial_reset = false;
      if(FLAG_SIMULATION_VREP)
      {
        auto shapes_c = config_("simulation")("markers");
        for(auto k : shapes_c.keys())
        {
          lshapes[k].update(shapes_c(k)("pos"), sva::PTransformd::Identity());
          gui()->addElement({"LShapes markers"},
                            mc_rtc::gui::Transform(k,
                                                   [this,k](){ return X_0_marker(k); },
                                                   [](const sva::PTransformd&){}));
        }
        lshapes_simulation_th_ = std::thread([this,shapes_c]()
        {
          ros::Rate rt(30);
          while(ros::ok() && active_)
          {
            {
              std::lock_guard<std::mutex> lock(lshapes_mut_);
              const auto & X_0_camera = getCameraPose();
              for(auto k : shapes_c.keys())
              {
                sva::PTransformd X_camera_marker;
                if(shapes_c(k).has("relative"))
                {
                  const auto & X_0_s = robot().surface(shapes_c(k)("relative")).X_0_s(robot());
                  sva::PTransformd X_s_marker = shapes_c(k)("pos");
                  auto X_0_marker = X_s_marker * X_0_s;
                  X_camera_marker = X_0_marker * (X_0_camera.inv());
                }
                else
                {
                  sva::PTransformd X_0_marker = shapes_c(k)("pos");
                  X_camera_marker = X_0_marker * (X_0_camera.inv());
                }
                lshapes[k].update(X_camera_marker, X_0_camera);
              }
            }
            rt.sleep();
          }
        });
      }
      else
      {
        if(!m_nh_)
        {
          LOG_ERROR_AND_THROW(std::runtime_error, "This controller does not work withtout ROS")
        }
        m_ros_spinner_ = std::thread{[this](){ this->ros_spinner(); }};
        l_shape_sub_ = m_nh_->subscribe("/whycon_lshape/whycon_lshape", 1000, &MCCableRegraspController::lShapeCallback, this);
      }
    } // initial reset

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

    // for simulation
    if(!FLAG_SIMULATION_VREP)
    {
        for(auto & ls : lshapes)
        {
        ls.second.tick(timeStep);
        }
    }
    if(FLAG_SIMULATION_VREP)
    {
    // transform from Vrep force sensor reference system to solver force sensor reference system
    wrenches["LeftHandForceSensor"] = this->robot().forceSensor("LeftHandForceSensor").wrench();
    wrenches["RightHandForceSensor"] = this->robot().forceSensor("RightHandForceSensor").wrench();
    sva::ForceVecd wrench_inter = wrenches.at("LeftHandForceSensor");
    wrenches.at("LeftHandForceSensor").couple() << wrench_inter.couple()[2],wrench_inter.couple()[1],-wrench_inter.couple()[0];
    wrenches.at("LeftHandForceSensor").force() << wrench_inter.force()[2],wrench_inter.force()[1],-wrench_inter.force()[0];
    wrench_inter = wrenches.at("RightHandForceSensor");
    wrenches.at("RightHandForceSensor").couple() << wrench_inter.couple()[2],wrench_inter.couple()[1],-wrench_inter.couple()[0];
    wrenches.at("RightHandForceSensor").force() << wrench_inter.force()[2],wrench_inter.force()[1],-wrench_inter.force()[0];
    }
    else
    {
    wrenches["LeftHandForceSensor"] = this->robot().forceSensor("LeftHandForceSensor").removeGravity(this->robot());
    wrenches["RightHandForceSensor"] = this->robot().forceSensor("RightHandForceSensor").removeGravity(this->robot());
    }

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
    auto X_0_lf = robot().surface("LFullSole").X_0_s(robot());
    auto X_0_rf = robot().surface("RFullSole").X_0_s(robot());
    auto X_lf_rf = X_0_rf * (X_0_lf.inv());
    X_lf_rf.translation() = X_lf_rf.translation() / 2;
    auto X_0_mid = X_lf_rf * X_0_lf;
    //
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
    else if (token == "Prim6ContinueS1")
    {
        prim6ContinueS1 = true;
        return true;
    }
    else if (token == "Prim6ContinueS2")
    {
        prim6ContinueS2 = true; 
        return true;
    }
    else if (token == "Prim6ContinueS3")
    {
        prim6ContinueS3 = true;
        return true;
    }
    else if(token == "Prim15Continue")
    {
        prim15Continue = true;
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
    else if (token == "GetEfPose")
    {
        // left gripper
        sva::PTransformd leftGripper;
        leftGripper = lh2Task->get_ef_pose();
        Eigen::Vector3d startPosLeft;
        startPosLeft = leftGripper.translation();
        Eigen::Matrix3d startRotLeft;
        startRotLeft = leftGripper.rotation();
        LOG_SUCCESS("left rotation:");
        LOG_SUCCESS(startRotLeft);
        LOG_SUCCESS("left translation:");
        LOG_SUCCESS(startPosLeft);

        // right gripper
        sva::PTransformd rightGripper;
        rightGripper = rh2Task->get_ef_pose();
        Eigen::Vector3d startPosRight;
        startPosRight = rightGripper.translation();
        Eigen::Matrix3d startRotRight;
        startRotRight = rightGripper.rotation();

        LOG_SUCCESS("right rotation:");
        LOG_SUCCESS(startRotRight);
        LOG_SUCCESS("right translation:");
        LOG_SUCCESS(startPosRight);
    }
    else if (token == "GetMarkerPos")
    {
        //// for experiment
        //marker1Pos = lshapes["wall_0"].world_pos;
        //marker2Pos = lshapes["rail"].world_pos;

        // for simulation
        marker1Pos = lshapes["wall_0"].world_pos * X_0_mid.inv();
        marker2Pos = lshapes["rail"].world_pos * X_0_mid.inv();

        // get current marker position
        Eigen::Vector3d zeroVec;
        zeroVec << 0.0, 0.0, 0.0;
        if (marker1Pos.translation() != zeroVec)
        {
            curMarkerPos = marker1Pos;
        }
        else if (marker2Pos.translation() != zeroVec)
        {
            curMarkerPos = marker2Pos;
        }
        else
        {
            // bug
            curMarkerPos = curMarkerPos;
        }

        // print out message
        LOG_SUCCESS("Marker_5cm position:");
        LOG_SUCCESS(marker1Pos.translation());
        LOG_SUCCESS("Marker_8cm position:");
        LOG_SUCCESS(marker2Pos.translation());
        LOG_SUCCESS("Current marker position:");
        LOG_SUCCESS(curMarkerPos.translation());
    }
    else if (token == "MoveGripper")
    {
        double x, y, z;
        ss >> x >> y >> z;
        Eigen::Vector3d diff;
        diff << x, y, z;

        // left gripper
        sva::PTransformd leftGripper;
        leftGripper = lh2Task->get_ef_pose() * X_0_mid.inv();
        Eigen::Vector3d startPosLeft;
        startPosLeft = leftGripper.translation();
        Eigen::Vector3d endPosLeft; 
        endPosLeft = startPosLeft + diff;
        // right gripper
        sva::PTransformd rightGripper;
        rightGripper = rh2Task->get_ef_pose() * X_0_mid.inv();
        Eigen::Vector3d startPosRight;
        startPosRight = rightGripper.translation();
        Eigen::Vector3d endPosRight;
        endPosRight = startPosRight + diff;

        lh2Task->set_ef_pose(sva::PTransformd(leftGripper.rotation(), endPosLeft) * X_0_mid);
        rh2Task->set_ef_pose(sva::PTransformd(rightGripper.rotation(), endPosRight) * X_0_mid);
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

CONTROLLER_CONSTRUCTOR("CableRegrasp", mc_control::MCCableRegraspController)
