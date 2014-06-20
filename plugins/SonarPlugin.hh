/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _GAZEBO_SONAR_PLUGIN_HH_
#define _GAZEBO_SONAR_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/SonarSensor.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  /// \brief A sonar sensor plugin
  class SonarPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: SonarPlugin();

    /// \brief Destructor
    public: virtual ~SonarPlugin();

    /// \brief Load the plugin
    /// \param[in] _parent Pointer to the parent sensor.
    /// \param[in] _sdf SDF element for the plugin.
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update callback. Overload this function in a child class.
    /// \param[in] _msg The sonar ping message.
    protected: virtual void OnUpdate(msgs::SonarStamped _msg);

    /// \brief The parent sensor
    protected: sensors::SonarSensorPtr parentSensor;

    /// \brief The connection tied to SonarSensor's update event
    private: event::ConnectionPtr connection;
  };
}
#endif
