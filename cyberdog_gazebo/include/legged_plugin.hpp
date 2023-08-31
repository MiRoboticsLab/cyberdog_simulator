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

#include <functional>
#include <memory>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <lcm/lcm-cpp.hpp>

#include "legged_simparam.hpp"
#include "actuator.hpp"
#include "lcmhandler.hpp"

#include <cyberdog_msg/msg/apply_force.hpp>

namespace gazebo
{

  struct _contact_force //Holds contact force data from foot contact sensors 
  {
    Eigen::Vector3d force; 
    std::string parent_name;
  };

  struct _apply_force //Holds apply force command from apply_force message
  {
    std::string name; 
    double time; 
    ignition::math::Vector3d force; 
    ignition::math::Vector3d rel_pos;
  };

  class LeggedPlugin : public ModelPlugin
  {
  public:
    /**
     * @brief Called once when gazebo start up.
     *        For more detail, visit https://classic.gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin
     * 
     * @param _parent A model is a collections of links, joints, and plugins
     * @param _sdf A pointer to the SDF element for the plugin in the world file
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  
    /**
     * @brief Called by the world update start event
     * 
     */
    void OnUpdate();

    Eigen::Vector3d forceToBody(_contact_force &_contact_force, physics::ModelPtr _model);

  private:
    /**
     * @brief update joint states from gazebo
     * 
     */
    void GetJointStates();

    /**
     * @brief send sharedmemory data to control program
     * 
     */
    void SendSMData();
    
    /**
     * @brief Set the joint command of robot in gazebo
     * 
     */
    void SetJointCom();

    /**
     * @brief Handle gamepad command lcm messages 
     * 
     * @param rbuf the lcm buffer to receive message
     * @param chan the channel to subscribe the lcm message
     * @param msg the gamepad command message
     */

    void HandleCommand(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const gamepad_lcmt *msg);

    /**
     * @brief Get the Contact Force from foot contact sensor
     * 
     */
    void GetContactForce4();

    /**
     * @brief Handle ApplyForce topic message 
     * 
     * @param msg ApplyForce topic message
     */
    void ForceHandler(const cyberdog_msg::msg::ApplyForce::SharedPtr msg);

    /**
     * @brief Apply force to the links of robot with the command from ApplyForce topic message 
     * 
     */
    void ApplyForce();
    
    // Pointer to the model
    physics::ModelPtr model_;

    // Pointer to the update event connection
    event::ConnectionPtr update_connection_;

    // gazebo sensors
    gazebo::sensors::Sensor_V sensors_;

    // gazebo sensors attached to the current robot
    gazebo::sensors::Sensor_V sensors_attached_to_robot_;

    // imu sensor
    gazebo::sensors::ImuSensorPtr imu_sensor_;

    // contact sensor
    gazebo::sensors::ContactSensorPtr contact_sensor_fl_;
    gazebo::sensors::ContactSensorPtr contact_sensor_fr_;
    gazebo::sensors::ContactSensorPtr contact_sensor_hl_;
    gazebo::sensors::ContactSensorPtr contact_sensor_hr_;

    // Gazebo joint names vector
    std::vector<std::string> joint_names_;

    // Gazebo joint map
    std::map<std::string, gazebo::physics::JointPtr> joint_map_;

    // ApplyForce topic subscription
    std::shared_ptr<GazeboNode> force_node_;
    rclcpp::Subscription<cyberdog_msg::msg::ApplyForce>::SharedPtr for_sub_;    
    _apply_force apply_force_;

    // Shared memory message
    SimulatorToRobotMessage simToRobot;

    // Lcm message of simulator state 
    simulator_lcmt lcm_sim_handler_;

    SimParam*     simparam_     =   nullptr;
    LCMHandler*   lcmhandler_   =   nullptr;
    NodeExc*      node_executor_ =   nullptr;
    
    std::vector<double> q_;
    std::vector<double> dq_;
    std::vector<double> tau_;
    std::vector<double> contact_;
    std::vector<double> q_ctrl_;
    std::vector<double> dq_ctrl_;
    std::vector<double> tau_ctrl_;
    Actuator motor_;

    int foot_counter_;
    int frequency_counter_;

    Eigen::Quaterniond q_body_;
    uint kleg_map[4] = {1, 0, 3, 2};
    
    bool use_currentloop_response_;
    bool use_TNcurve_motormodel_;
    bool use_torque_response_;
    bool use_force_contact_sensor_;
  };
}
