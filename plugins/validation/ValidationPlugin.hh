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

#ifndef _GAZEBO_VALIDATION_PLUGIN_
#define _GAZEBO_VALIDATION_PLUGIN_

#include <memory>
#include <mutex>
#include <string>
#include <ignition/transport/Node.hh>
#include <sdf/sdf.hh>
#include "gazebo/common/Time.hh"
#include "gazebo/common/Timer.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"
#include "State.hh"

namespace gazebo
{
  /// \brief Labels for states.
  static std::string kReadyState = "gazebo_ready";
  static std::string kSetState   = "gazebo_set";
  static std::string kGoState    = "gazebo_go";
  static std::string kEndState   = "gazebo_end";

  /// \brief State that handles the "ready" state.
  class ReadyState : public State
  {
    // Use class constructor from base class.
    using State::State;

    // Documentation inherited.
    public: virtual void DoInitialize();

    // Documentation inherited
    public: virtual void DoFeedback();
  };

  /// \brief State that handles the "set" state.
  class SetState : public State
  {
    // Use class constructor from base class.
    using State::State;

    // Documentation inherited.
    public: virtual void DoInitialize();
  };

  /// \brief State that handles the "go" state.
  class GoState : public State
  {
    // Use class constructor from base class.
    using State::State;

    public: virtual void DoInitialize();

    // Documentation inherited.
    public: virtual void DoFeedback();
  };

  /// \brief State that handles the "end" state.
  class EndState : public State
  {
    // Use class constructor from base class.
    using State::State;

    public: virtual void DoInitialize();
  };

  /// Example SDF:
  ///       <plugin name="actuator_plugin" filename="libActuatorPlugin.so">
  ///        <actuator>
  ///          <name>actuator_0</name> <!-- optional -->
  ///          <joint>JOINT_0</joint> <!-- name of joint to actuate -->
  ///          <index>0</index> <!-- needed for multi-DOF joints -->
  ///          <type>electric_motor</type> <!-- motor model type -->
  ///          <power>20</power> <!-- parameters for motor model -->
  ///          <max_velocity>6</max_velocity>
  ///          <max_torque>10.0</max_torque>
  ///        </actuator>
  ///      </plugin>
  ///    </model>
  ///
  /// Required fields:
  /// - name
  /// - joint
  /// - index (can be 0 in most cases)
  /// - type: current options are electric_motor, velocity_limiter or null
  /// Required for motor model electric_motor:
  /// - power
  /// - max_velocity
  /// - max_torque
  /// Required for motor model velocity_limiter:
  /// - max_velocity
  /// - max_torque
  /// \brief ToDo
  class GAZEBO_VISIBLE ValidationPlugin : public ModelPlugin
  {
    /// \brief Class constructor.
    public: ValidationPlugin();

    /// Documentation inherited
    public: void Load(physics::ModelPtr _parent,
                      sdf::ElementPtr _sdf);

    /// \brief Callback on world update event.
    private: void WorldUpdateCallback();

    /// \brief ToDo.
    private: bool LoadModelParams();

    /// \brief ToDo.
    public: void ChangeState(State &_newState);

    /// \brief ToDo.
    public: bool MoreRuns() const;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief State machine states.
    public: std::unique_ptr<ReadyState> readyState;
    public: std::unique_ptr<SetState> setState;
    public: std::unique_ptr<GoState> goState;
    public: std::unique_ptr<EndState> endState;

    /// \brief Pointer to the current state.
    public: State *currentState;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ValidationPlugin)
}

#endif
