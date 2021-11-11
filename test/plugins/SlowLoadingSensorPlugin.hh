/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef GAZEBO_TEST_PLUGINS_SLOWLOADINGSENSORPLUGIN_HH_
#define GAZEBO_TEST_PLUGINS_SLOWLOADINGSENSORPLUGIN_HH_

#include <memory>

#include <gazebo/common/Plugin.hh>

namespace gazebo
{
  class SlowLoadingSensorPluginPrivate;

  /// \brief A sensor plugin that sleeps a specified amount of time
  /// during Load.
  ///
  /// The plugin requires the following parameter:
  /// <load_seconds>     Number of seconds to sleep during load
  class GAZEBO_VISIBLE SlowLoadingSensorPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: SlowLoadingSensorPlugin();

    /// \brief Destructor.
    public: virtual ~SlowLoadingSensorPlugin();

    /// \brief Load the plugin.
    /// \param[in] _sensor Pointer to sensor
    /// \param[in] _sdf Pointer to the SDF configuration.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Pointer to private data.
    private: std::unique_ptr<SlowLoadingSensorPluginPrivate> dataPtr;
  };
}

#endif
