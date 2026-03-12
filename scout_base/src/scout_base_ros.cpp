/*
 * scout_base_ros.cpp
 *
 * Created on: Oct 15, 2021 14:35
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "scout_base/scout_base_ros.hpp"

#include "scout_base/scout_messenger.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace westonrobot {

ScoutBaseRos::ScoutBaseRos(std::string node_name)
    : rclcpp::Node(node_name), keep_running_(false) {
  this->declare_parameter("port_name", rclcpp::ParameterValue("can0"));

  this->declare_parameter("odom_frame", rclcpp::ParameterValue("odom"));
  this->declare_parameter("base_frame", rclcpp::ParameterValue("base_link"));
  this->declare_parameter("odom_topic_name", rclcpp::ParameterValue("odom"));

  this->declare_parameter("status_topic_name",
                          rclcpp::ParameterValue("/scout_status"));
  this->declare_parameter("motion_cmd_topic_name",
                          rclcpp::ParameterValue("/cmd_vel"));
  this->declare_parameter("light_cmd_topic_name",
                          rclcpp::ParameterValue("/light_control"));

  this->declare_parameter("publish_tf", rclcpp::ParameterValue(true));
  this->declare_parameter("use_stamped_cmd_vel", rclcpp::ParameterValue(false));

  this->declare_parameter("is_scout_mini", rclcpp::ParameterValue(false));
  this->declare_parameter("is_omni_wheel", rclcpp::ParameterValue(false));

  this->declare_parameter("simulated_robot", rclcpp::ParameterValue(false));
  this->declare_parameter("control_rate", rclcpp::ParameterValue(50));

  LoadParameters();
}

void ScoutBaseRos::LoadParameters() {
  this->get_parameter_or<std::string>("port_name", port_name_, "can0");

  this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
  this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
  this->get_parameter_or<std::string>("odom_topic_name", odom_topic_name_,
                                      "odom");

  this->get_parameter_or<std::string>("status_topic_name", status_topic_name_,
                                      "/scout_status");
  this->get_parameter_or<std::string>("motion_cmd_topic_name",
                                      motion_cmd_topic_name_, "/cmd_vel");
  this->get_parameter_or<std::string>("light_cmd_topic_name",
                                      light_cmd_topic_name_, "/light_control");

  this->get_parameter_or<bool>("publish_tf", publish_tf_, true);
  this->get_parameter_or<bool>("use_stamped_cmd_vel", use_stamped_cmd_vel_,
                               false);

  this->get_parameter_or<bool>("is_scout_mini", is_scout_mini_, false);
  this->get_parameter_or<bool>("is_omni_wheel", is_omni_wheel_, false);

  this->get_parameter_or<bool>("simulated_robot", simulated_robot_, false);
  this->get_parameter_or<int>("control_rate", sim_control_rate_, 50);

  std::cout << "Loading parameters: " << std::endl;
  std::cout << "- port name: " << port_name_ << std::endl;
  std::cout << "- odom frame name: " << odom_frame_ << std::endl;
  std::cout << "- base frame name: " << base_frame_ << std::endl;
  std::cout << "- odom topic name: " << odom_topic_name_ << std::endl;

  std::cout << "- status topic name: " << status_topic_name_ << std::endl;
  std::cout << "- motion_cmd topic name: " << motion_cmd_topic_name_
            << std::endl;
  std::cout << "- light_cmd topic name: " << light_cmd_topic_name_ << std::endl;

  std::cout << "- publish_tf: " << std::boolalpha << publish_tf_ << std::endl;

  std::cout << "- is scout mini: " << std::boolalpha << is_scout_mini_
            << std::endl;
  std::cout << "- is omni wheel: " << std::boolalpha << is_omni_wheel_
            << std::endl;

  std::cout << "- simulated robot: " << std::boolalpha << simulated_robot_
            << std::endl;
  std::cout << "- sim control rate: " << sim_control_rate_ << std::endl;
  std::cout << "----------------------------" << std::endl;
}

bool ScoutBaseRos::Initialize() {
  if (is_scout_mini_) {
    std::cout << "Robot base: Scout Mini" << std::endl;
  } else {
    std::cout << "Robot base: Scout" << std::endl;
  }

  ProtocolDetector detector;
  if (detector.Connect(port_name_)) {
    auto proto = detector.DetectProtocolVersion(5);
    if (proto == ProtocolVersion::AGX_V1) {
      std::cout << "Detected protocol: AGX_V1" << std::endl;
      if (!is_omni_wheel_) {
        is_omni_ = false;
        robot_ = std::make_shared<ScoutRobot>(ProtocolVersion::AGX_V1,
                                              is_scout_mini_);
        if (is_scout_mini_) {
          std::cout << "Creating interface for Scout Mini with AGX_V1 Protocol"
                    << std::endl;
        } else {
          std::cout << "Creating interface for Scout with AGX_V1 Protocol"
                    << std::endl;
        }
      } else {
        is_omni_ = true;
        omni_robot_ = std::unique_ptr<ScoutMiniOmniRobot>(
            new ScoutMiniOmniRobot(ProtocolVersion::AGX_V1));
        std::cout
            << "Creating interface for Scout Mini Omni with AGX_V1 Protocol"
            << std::endl;
      }
    } else if (proto == ProtocolVersion::AGX_V2) {
      std::cout << "Detected protocol: AGX_V2" << std::endl;
      if (!is_omni_wheel_) {
        is_omni_ = false;
        robot_ = std::make_shared<ScoutRobot>(ProtocolVersion::AGX_V2,
                                              is_scout_mini_);
        std::cout << "Creating interface for Scout with AGX_V2 Protocol"
                  << std::endl;
      } else {
        is_omni_ = true;
        omni_robot_ = std::unique_ptr<ScoutMiniOmniRobot>(
            new ScoutMiniOmniRobot(ProtocolVersion::AGX_V2));
        std::cout
            << "Creating interface for Scout Mini Omni with AGX_V2 Protocol"
            << std::endl;
      }
    } else {
      std::cout << "Detected protocol: UNKONWN" << std::endl;
      return false;
    }
  } else {
    return false;
  }

  return true;
}

void ScoutBaseRos::Stop() { keep_running_ = false; }

void ScoutBaseRos::Run() {

  auto run_messenger = [&](auto robot, auto connect_fn) {
    using RobotType = typename decltype(robot)::element_type;

    auto make_and_run = [&](auto messenger) {
      messenger->SetFrames(odom_frame_, base_frame_);
      messenger->SetDataTopicNames(odom_topic_name_, status_topic_name_);
      messenger->SetCmdTopicNames(motion_cmd_topic_name_,
                                  light_cmd_topic_name_);
      if (simulated_robot_)
        messenger->SetSimulationMode(sim_control_rate_);
      messenger->SetPublishTf(publish_tf_);

      if (!connect_fn())
        return;

      // publish robot state at 50Hz while listening to twist commands
      messenger->SetupSubscription();
      keep_running_ = true;
      rclcpp::Rate rate(50, this->get_clock());
      while (keep_running_) {
        messenger->PublishStateToROS();
        rclcpp::spin_some(shared_from_this());
        rate.sleep();
        //this->get_clock()->sleep_for(rclcpp::Duration(0, 20000000));
      }
    };

    if (use_stamped_cmd_vel_) {
      make_and_run(std::make_unique<
                   ScoutMessenger<RobotType, geometry_msgs::msg::TwistStamped>>(
          robot, this));
    } else {
      make_and_run(std::make_unique<
                   ScoutMessenger<RobotType, geometry_msgs::msg::Twist>>(robot,
                                                                         this));
    }
  };

  auto make_connect_fn = [&](auto robot) {
    return [&, robot]() -> bool {
      if (port_name_.find("can") == std::string::npos) {
        std::cout << "Please check the specified port name is a CAN port"
                  << std::endl;
        return false;
      }
      if (robot->Connect(port_name_)) {
        robot->EnableCommandedMode();
        std::cout << "Using CAN bus to talk with the robot" << std::endl;
        return true;
      }
      std::cout << "Failed to connect to the robot CAN bus" << std::endl;
      return false;
    };
  };

  // instantiate a ROS messenger
  if (!is_omni_) {
    run_messenger(robot_, make_connect_fn(robot_));
  } else {
    run_messenger(omni_robot_, make_connect_fn(omni_robot_));
  }
}
} // namespace westonrobot
