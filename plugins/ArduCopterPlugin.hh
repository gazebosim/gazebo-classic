/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_PLUGINS_ARDUCOPTERPLUGIN_HH_
#define GAZEBO_PLUGINS_ARDUCOPTERPLUGIN_HH_

#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  // Forward declare private data class
  class ArduCopterPluginPrivate;

  /// \brief Interface ArduCopter from ardupilot stack
  /// modeled after SITL/SIM_*
  ///
  /// The plugin requires the following parameters:
  /// <controller_ip>       controller is on this ip
  /// <state_port>          This plugin publishes states on this port
  /// <command_port>        This plugin receives states on this port
  ///
  /// The following parameters are optional:
  /// <param.xml>           use params stored at this absolute file path
  ///
  class GAZEBO_VISIBLE ArduCopterPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: ArduCopterPlugin();

    /// \brief Destructor.
    public: ~ArduCopterPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the control surfaces controllers.
    /// \param[in] _info Update information provided by the server.
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Update PID Joint controllers.
    /// \param[in] _dt time step size since last update.
    private: void ApplyMotorForces(const double _dt);

    /// \brief Reset PID Joint controllers.
    private: void ResetPIDs();

    /// \brief Receive motor commands from ArduCopter
    private: void ReceiveMotorCommand();

    /// \brief Send state to ArduCopter
    private: void SendState() const;

    /// \brief Private data pointer.
    private: std::unique_ptr<ArduCopterPluginPrivate> dataPtr;
  };
}
#endif
