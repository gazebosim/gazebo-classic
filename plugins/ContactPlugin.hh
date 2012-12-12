/*
 * Copyright 2012 Open Source Robotics Foundation
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
/*
 * Desc: Contact Plugin
 * Author: Nate Koenig mod by John Hsu
 */

#ifndef GAZEBO_CONTACT_PLUGIN_HH
#define GAZEBO_CONTACT_PLUGIN_HH

#include "common/Plugin.hh"
#include "sensors/SensorTypes.hh"
#include "sensors/ContactSensor.hh"
#include "gazebo.hh"

namespace gazebo
{
  /// \brief A Bumper controller
  class ContactPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: ContactPlugin();

    /// \brief Destructor
    public: virtual ~ContactPlugin();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Pointer to parent
    protected: physics::WorldPtr world;

    /// \brief The parent sensor
    protected: sensors::ContactSensorPtr parentSensor;
  };
}

#endif

