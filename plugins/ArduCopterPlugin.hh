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
  /// <rotor>       rotor description block
  ///    id         attribute rotor id
  ///    <vel_p_gain>       velocity pid p gain
  ///    <vel_i_gain>       velocity pid i gain
  ///    <vel_d_gain>       velocity pid d gain
  ///    <vel_i_max>        velocity pid max integral correction
  ///    <vel_i_min>        velocity pid min integral correction
  ///    <vel_cmd_max>      velocity pid max command torque
  ///    <vel_cmd_min>      velocity pid min command torque
  ///    <jointName>        rotor motor joint, torque applied here
  ///    <turningDirection> turning direction, 'cw' or 'ccw'
  ///    <rotorVelocitySlowdownSim> experimental, not needed
  /// <imuName>     scoped name for the imu sensor
  /// <connectionTimeoutMaxCount> timeout before giving up on
  ///                             controller synchronization
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
    private: void OnUpdate();

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
