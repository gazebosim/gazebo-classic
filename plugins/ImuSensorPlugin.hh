/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef _GAZEBO_IMU_SENSOR_PLUGIN_HH_
#define _GAZEBO_IMU_SENSOR_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/Events.hh"

namespace gazebo
{
  /// \brief An base class plugin for custom imu sensor processing.
  class GAZEBO_VISIBLE ImuSensorPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: ImuSensorPlugin();

    /// \brief Destructor
    public: virtual ~ImuSensorPlugin();

    /// \brief Load the plugin.
    /// \param[in] _parent Pointer to the parent sensor.
    /// \param[in] _sdf SDF element for the plugin.
    public: virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update callback. Overload this function in a child class.
    /// \param[in] _sensor pointer to this sensor
    protected: virtual void OnUpdate(sensors::ImuSensorPtr _sensor);

    /// \brief The parent sensor
    protected: sensors::ImuSensorPtr parentSensor;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr connection;

    /// \brief Pointer to the world
    private: physics::WorldPtr world;

    /// \brief Pointer to the parent link
    private: physics::LinkPtr link;
  };
}
#endif
