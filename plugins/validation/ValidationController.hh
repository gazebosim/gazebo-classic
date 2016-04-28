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

#ifndef _GAZEBO_PLUGINS_VALIDATION_CONTROLLER_
#define _GAZEBO_PLUGINS_VALIDATION_CONTROLLER_

#include <memory>
#include <mutex>
#include <string>
#include "State.hh"

namespace gazebo
{
  /// \brief Labels for states.
  static std::string kControllerReadyState     = "controller_ready";
  static std::string kControllerInitCondsState = "controller_init_conds";
  static std::string kControllerRunningState   = "controller_running";
  static std::string kControllerEndState       = "controller_end";

  class ValidationController;

  /// \brief A generic state for the validation controller.
  class GAZEBO_VISIBLE ControllerState : public State
  {
    // Use class constructor from base class.
    public: ControllerState(const std::string &_name,
                            ValidationController &_plugin);

    /// \brief ToDo.
    protected: ValidationController &controller;
  };

  /// \brief State that handles the "ready" state.
  class ControllerReadyState : public ControllerState
  {
    // Use class constructor from base class.
    using ControllerState::ControllerState;

    // Documentation inherited
    public: virtual void DoFeedback();
  };

  /// \brief State that handles the "initConds" state.
  class ControllerInitCondsState : public ControllerState
  {
    // Use class constructor from base class.
    using ControllerState::ControllerState;

    // Documentation inherited.
    public: virtual void DoInitialize();

    // Documentation inherited.
    public: virtual void DoUpdate();
  };

  /// \brief State that handles the "running" state.
  class ControllerRunningState : public ControllerState
  {
    // Use class constructor from base class.
    using ControllerState::ControllerState;

    // Documentation inherited.
    public: virtual void DoInitialize();

    // Documentation inherited.
    public: virtual void DoUpdate();
  };

  /// \brief State that handles the "running" state.
  class ControllerEndState : public ControllerState
  {
    // Use class constructor from base class.
    using ControllerState::ControllerState;

    // Documentation inherited.
    public: virtual void DoInitialize();

    // Documentation inherited
    public: virtual void DoFeedback();
  };

  /// \brief ToDo
  class GAZEBO_VISIBLE ValidationController
  {
    /// \brief Class constructor.
    public: ValidationController();

    /// \brief Class destructor.
    public: ~ValidationController();

    /// \brief Callback on world update event.
    public: void Start();

    /// \brief ToDo.
    public: void ChangeState(State &_newState);

    /// ToDo
    public: virtual void UpdateRun() = 0;

    /// ToDo
    public: virtual bool Running() const = 0;

    /// ToDo
    public: virtual void InitializeInitConds() {};

    /// ToDo
    public: virtual void UpdateInitConds() {};

    /// ToDo
    public: virtual bool Initializing() const {return false;};

    /// ToDo
    public: virtual void InitializeRun() {};

    /// \brief State machine states.
    public: std::unique_ptr<ControllerReadyState>     readyState;
    public: std::unique_ptr<ControllerInitCondsState> initCondsState;
    public: std::unique_ptr<ControllerRunningState>   runningState;
    public: std::unique_ptr<ControllerEndState>       endState;

    /// \brief Pointer to the current state.
    public: State *currentState;
  };
}

#endif
