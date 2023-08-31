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
#ifndef FOOT_CONTACT_PLUGIN_HPP
#define FOOT_CONTACT_PLUGIN_HPP

#include <string>

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <lcm/lcm-cpp.hpp>

#include "gazebo_foot_contact.hpp"


namespace gazebo
{
  class FootContactPlugin : public SensorPlugin
  {
    
    public:
    /**
     * @brief Construct a new Foot Contact Plugin object
     * 
     */
    FootContactPlugin();

    /**
     * @brief Destroy the Foot Contact Plugin object
     * 
     */
    virtual ~FootContactPlugin();

    /**
     * @brief Load the sensor plugin.
     * 
     * @param _sensor Pointer to the sensor that loaded this plugin.
     * @param _sdf SDF element that describes the plugin.
    */
    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private: 
    /**
     * @brief Callback that receives the contact sensor's update signal.
     * 
     */
    virtual void OnUpdate();

    // Pointer to the contact sensor
    sensors::ContactSensorPtr parentSensor;

    // Connection that maintains a link between the contact sensor's
    // updated signal and the OnUpdate callback.
    event::ConnectionPtr updateConnection;
  };
}
#endif
