/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS_CAMERATRIGGERPLUGIN_HH_
#define GAZEBO_PLUGINS_CAMERATRIGGERPLUGIN_HH_

#include <memory>
#include <string>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  // forward declaration
  class CameraTriggerPluginPrivate;

  /// \brief A plugin to demonstrate manual trigger of camera sensor updates
  /// Press 'c' on the keyboard to trigger a render update and receive an image
  /// via the camera topic.
  class GAZEBO_VISIBLE CameraTriggerPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: CameraTriggerPlugin();

    /// \brief Destructor
    public: virtual ~CameraTriggerPlugin();

    // Documentation inherited
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Pointer to the private data
    std::unique_ptr<CameraTriggerPluginPrivate> dataPtr;
  };
}
#endif
