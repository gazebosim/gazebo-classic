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

namespace gazebo
{
  /// \brief Labels for states.
  static std::string kInitState  = "init_state";
  static std::string kSetupState = "setup_state";
  static std::string kRunState   = "run_state";
  static std::string kEvalState  = "eval_state";
  static std::string kStopState  = "stop_state";

  // Forward declarations.
  class ValidationPlugin;

  namespace msgs
  {
    class GzString;
  }

  /// \brief State pattern used for the state machine.
  class State
  {
    /// \brief Class constructor.
    /// \param[in] _name Name of the state.
    /// \param[in] _plugin Pointer to the model plugin.
    public: State(const std::string &_name,
                  ValidationPlugin &_plugin);

    /// \brief Update the state.
    public: void Update();

    /// \brief Called once before changing to a different state.
    public: void Teardown();

    /// \brief Equal to operator.
    /// \param[in] _state The state to compare against.
    /// \return true if the state has the same name.
    public: bool operator ==(const State &_state) const;

    /// \brief Not equal to operator
    /// \param[in] _state The state to compare against
    /// \return true if the state doesn't have the same name.
    public: bool operator !=(const State &_v) const;

    /// \brief Return the current feedback value.
    protected: std::string Feedback() const;

    /// \brief ToDo.
    private: virtual void DoInitialize();

    /// \brief ToDo.
    private: virtual void DoUpdate();

    /// \brief ToDo.
    private: virtual void DoTeardown();

    /// \brief ToDo.
    private: virtual void DoFeedback();

    /// \brief Initialize the state. Called once after a pause duration after
    /// entering state.
    private: void Initialize();

    /// \brief Callback that updates the feedback member variable.
    private: void OnFeedback(const msgs::GzString &_msg);

     /// \brief Publish the current state.
    private: void PublishState();

    /// \brief Name of the state.
    public: const std::string name;

    /// \brief Pointer to the validation plugin.
    protected: ValidationPlugin &plugin;

    /// \brief Timer to measure time in the current state.
    protected: common::Timer timer;

    /// \brief Has initialized
    protected: bool initialized = false;

    /// \brief Last feedback message.
    private: std::string feedback;

    /// \brief Mutex to protect the feedback message.
    private: mutable std::mutex mutex;

    /// \brief Last time the we published the state.
    private: common::Time lastPublication;

    /// \brief Elapsed time between state publications.
    private: common::Time publicationInterval;

    /// \brief Transport node.
    protected: ignition::transport::Node node;
  };

  /// \brief State that handles the "init" state.
  class InitState : public State
  {
    // Use class constructor from base class.
    using State::State;

    // Documentation inherited.
    public: virtual void DoInitialize();

    // Documentation inherited
    public: virtual void DoFeedback();
  };

  /// \brief State that handles the "setup" state.
  class SetupState : public State
  {
    // Use class constructor from base class.
    using State::State;

    // Documentation inherited
    public: virtual void DoFeedback();
  };

  /// \brief State that handles the "run" state.
  class RunState : public State
  {
    // Use class constructor from base class.
    using State::State;

    // Documentation inherited.
    public: virtual void DoFeedback();
  };

  /// \brief State that handles the "eval" state.
  class EvalState : public State
  {
    // Use class constructor from base class.
    using State::State;

    // Documentation inherited
    public: virtual void DoFeedback();
  };

  /// \brief State that handles the "stop" state.
  class StopState : public State
  {
    // Use class constructor from base class.
    using State::State;

    // Documentation inherited.
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
    public: void SetState(State &_newState);

    /// \brief ToDo.
    public: bool MoreRuns() const;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief "init" state.
    public: std::unique_ptr<InitState> initState;

    /// \brief "setup" state.
    public: std::unique_ptr<SetupState> setupState;

    /// \brief "run" state.
    public: std::unique_ptr<RunState> runState;

    /// \brief "eval" state.
    public: std::unique_ptr<EvalState> evalState;

    /// \brief "stop" state.
    public: std::unique_ptr<StopState> stopState;

    /// \brief Pointer to the current game state.
    public: State *currentState;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ValidationPlugin)
}

#endif
