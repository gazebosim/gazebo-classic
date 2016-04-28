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

#ifndef _GAZEBO_PLUGINS_VALIDATION_PLUGIN_
#define _GAZEBO_PLUGINS_VALIDATION_PLUGIN_

#include <memory>
#include <mutex>
#include <string>
#include <sdf/sdf.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"
#include "State.hh"

namespace gazebo
{
  /// \brief Labels for states.
  static std::string kReadyState = "gazebo_ready";
  static std::string kSetState   = "gazebo_set";
  static std::string kGoState    = "gazebo_go";
  static std::string kEndState   = "gazebo_end";

  class ValidationPlugin;

  /// \brief A generic state for the validation plugin.
  class GAZEBO_VISIBLE GazeboState : public State
  {
    // Use class constructor from base class.
    public: GazeboState(const std::string &_name,
                        ValidationPlugin &_plugin);

    /// \brief ToDo.
    protected: ValidationPlugin &plugin;
  };

  /// \brief State that handles the "ready" state.
  class GAZEBO_VISIBLE GazeboReadyState : public GazeboState
  {
    // Use class constructor from base class.
    using GazeboState::GazeboState;

    // Documentation inherited
    public: virtual void DoFeedback();
  };

  /// \brief State that handles the "set" state.
  class GazeboSetState : public GazeboState
  {
    // Use class constructor from base class.
    using GazeboState::GazeboState;

    // Documentation inherited.
    public: virtual void DoInitialize();
  };

  /// \brief State that handles the "go" state.
  class GazeboGoState : public GazeboState
  {
    // Use class constructor from base class.
    using GazeboState::GazeboState;

    public: virtual void DoInitialize();

    // Documentation inherited.
    public: virtual void DoFeedback();
  };

  /// \brief State that handles the "end" state.
  class GazeboEndState : public GazeboState
  {
    // Use class constructor from base class.
    using GazeboState::GazeboState;

    public: virtual void DoInitialize();
  };

  /// \brief ToDo
  /// The plugin accepts the following parameters:
  /// <lower_limit>
  ///   <name>a_joint</name>
  ///   <min>1.0</min>
  ///   <max>2.0</max>
  ///   <step>0.25</step>
  /// </lower_limit>
  /// <upper_limit>
  ///   <name>a_joint</name>
  ///   <min>3.0</min>
  ///   <max>4.0</max>
  ///   <step>0.25</step>
  /// </upper_limit>
  class GAZEBO_VISIBLE ValidationPlugin : public ModelPlugin
  {
    /// \brief Class constructor.
    public: ValidationPlugin();

    /// Documentation inherited
    public: void Load(physics::ModelPtr _model,
                      sdf::ElementPtr _sdf);

    /// \brief ToDo.
    public: bool LoadModelParams();

    /// \brief Callback on world update event.
    private: void WorldUpdateCallback();

    /// \brief ToDo.
    public: void ChangeState(State &_newState);

    /// \brief ToDo.
    public: bool MoreRuns() const;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief State machine states.
    public: std::unique_ptr<GazeboReadyState> readyState;
    public: std::unique_ptr<GazeboSetState>   setState;
    public: std::unique_ptr<GazeboGoState>    goState;
    public: std::unique_ptr<GazeboEndState>   endState;

    /// \brief Pointer to the current state.
    public: State *currentState;

    /// \brief Pointer to the model;
    private: physics::ModelPtr model;

    /// \brief Lower joint limits to test.
    private: std::map<std::string, std::vector<double>> lowerLimitParams;

    /// \brief Upper joint limits to test.
    private: std::map<std::string, std::vector<double>> upperLimitParams;

    /// \brief ToDo.
    //private: std::string currentParam;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ValidationPlugin)
}

#endif
