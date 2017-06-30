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
#ifndef GAZEBO_WHEEL_SLIP_PLUGIN_HH_
#define GAZEBO_WHEEL_SLIP_PLUGIN_HH_

#include <memory>

#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  // Forward declare private data class
  class WheelSlipPluginPrivate;

  /// \brief A plugin that updates the ODE wheel slip parameter
  ///
  /** \verbatim
    <plugin filename="libWheelSlipPlugin.so" name="wheel_slip">
      <!-- Names of wheel links -->
      <link>wheel_front_left</link>
      <link>wheel_front_right</link>
      <link>wheel_rear_left</link>
      <link>wheel_rear_right</link>
    </plugin>
   \endverbatim */
  class GAZEBO_VISIBLE WheelSlipPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: WheelSlipPlugin();

    /// \brief Destructor.
    public: virtual ~WheelSlipPlugin();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the plugin. This is updated every iteration of
    /// simulation.
    private: void Update();

    /// \brief Private data pointer.
    private: std::unique_ptr<WheelSlipPluginPrivate> dataPtr;
  };
}
#endif
