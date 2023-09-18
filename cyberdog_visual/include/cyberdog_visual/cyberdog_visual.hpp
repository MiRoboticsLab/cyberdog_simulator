// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <urdf/model.h>
#include <vector>
#include <stack>
#include <lcm/lcm-cpp.hpp>
#include "leg_control_data_lcmt.hpp"
#include "state_estimator_lcmt.hpp"
#include "simulator_lcmt.hpp"
#include "localization_lcmt.hpp"

namespace cyberdog
{

  class CyberDogVisual : public rclcpp::Node
  {
  public:
    /**
     * @brief Construct a new CyberDogVisual object
     * 
     */
    CyberDogVisual();

    /**
     * @brief Destroy the CyberDogVisual object
     * 
     */
    virtual ~CyberDogVisual();

  private:
    /**
     * @brief update function for sending lcm messages
     * 
     */
    void UpdateTimerCallback();

    /**
     * @brief Handle Odom lcm messages
     * 
     * @param rbuf : buffer receive lcm messages
     * @param chan : the channel to subscribe the lcm message
     * @param msg  : the Odom message
     */
    void HandleOdomMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const state_estimator_lcmt *msg);

    /**
     * @brief Handle global to robot lcm messages
     * 
     * @param rbuf : buffer receive lcm messages 
     * @param chan : the channel to subscribe the lcm message
     * @param msg  : the global to robot message
     */
    void HandleGlobalOdomMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const localization_lcmt *msg);
    
    /**
     * @brief Handle joint states messages
     * 
     * @param rbuf : buffer receive lcm messages 
     * @param chan : the channel to subscribe the lcm message
     * @param msg  : the joint states message
     */
    void HandleJointMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const leg_control_data_lcmt *msg);

    /**
     * @brief Read parameters 
     * 
     */
    void ReadParameters();

    /**
     * @brief Get the name of joint 
     * 
     * @param urdf_model : the urdf of robot
     * @return std::vector<std::string> : name list of each joints
     */
    std::vector<std::string> GetJointName(const urdf::Model &urdf_model);

    rclcpp::TimerBase::SharedPtr  update_timer_;
     std::shared_ptr<tf2_ros::TransformBroadcaster>  br_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
		sensor_msgs::msg::JointState js_;

    lcm::LCM odom_lcm_;
    lcm::LCM odom_global_lcm_;
    lcm::LCM joint_lcm_;

    std::string joint_state_topic_;
    std::string robot_description_;
    std::string root_link_;
    std::string urdf_string;
    double publish_frequency_;

    bool use_state_estimator_;
  };
}