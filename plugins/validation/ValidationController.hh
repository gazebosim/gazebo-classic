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

#include <mutex>
#include <string>
#include <ignition/transport/Node.hh>
#include "gazebo/common/Time.hh"
#include "gazebo/common/Timer.hh"
#include "State.hh"

namespace gazebo
{
  /// \brief Labels for states.
  static std::string kControllerReadyState     = "controller_ready";
  static std::string kControllerInitCondsState = "controller_init_conds";
  static std::string kControllerRunningState   = "controller_running";
  static std::string kControllerEndState       = "controller_end";

  /// \brief State that handles the "ready" state.
  template <class T>
  class ControllerReadyState : public State<T>
  {
    // Use class constructor from base class.
    using State<T>::State;

    // Documentation inherited.
    public: virtual void DoInitialize()
    {
      std::cout << "ReadyState::Initialize()" << std::endl;
    }

    // Documentation inherited
    public: virtual void DoFeedback()
    {
      //if (this->Feedback() == "gazebo_go")
      //  this->fsm.ChangeState(*this->fsm.initCondsState);

      //std::cout << "ReadyState::DoOnState()" << std::endl;
    }
  };

  /// \brief State that handles the "initConds" state.
  template <typename T>
  class ControllerInitCondsState : public State<T>
  {
    // Use class constructor from base class.
    using State<T>::State;

    // Documentation inherited.
    public: virtual void DoInitialize()
    {
      // Send initial conditions.
      std::cout << "InitCondsState::DoInitialize()" << std::endl;
      //std::cout << "Initial conditions" << std::endl;
    }

    // Documentation inherited.
    public: virtual void DoUpdate()
    {
      // Check if the initial conditions are satisfied.

      //std::cout << "InitCondsState::DoUpdate()" << std::endl;

      if (this->timer.GetElapsed() >= gazebo::common::Time(2.0))
        this->fsm.ChangeState(*this->fsm.runningState);
    }
  };

  /// \brief State that handles the "running" state.
  template <typename T>
  class ControllerRunningState : public State<T>
  {
    // Use class constructor from base class.
    using State<T>::State;

    // Documentation inherited.
    public: virtual void DoInitialize()
    {
      std::cout << "RunningState::Initialize()" << std::endl;
    }

    // Documentation inherited.
    public: virtual void DoUpdate()
    {
      // Send the next command.

      //std::cout << "RunningState::DoUpdate()" << std::endl;

      // Check if we are done with the run
      if (this->timer.GetElapsed() >= gazebo::common::Time(5.0))
        this->fsm.ChangeState(*this->fsm.endState);
    }
  };

  /// \brief State that handles the "running" state.
  template <typename T>
  class ControllerEndState : public State<T>
  {
    // Use class constructor from base class.
    using State<T>::State;

    // Documentation inherited.
    public: virtual void DoInitialize()
    {
      std::cout << "EndState::Initialize()" << std::endl;
    }

    // Documentation inherited
    public: virtual void DoOnState()
    {
      // Check if Gazebo is ready for another run.
      if (this->Feedback() == "gazebo_ready")
        this->fsm.ChangeState(*this->fsm.readyState);

      //std::cout << "EndState::DoOnState()" << std::endl;
    }
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
    public: void ChangeState(State<ValidationController> &_newState);

    /// \brief State machine states.
    public: std::unique_ptr<ControllerReadyState<ValidationController>>     readyState;
    public: std::unique_ptr<ControllerInitCondsState<ValidationController>> initCondsState;
    public: std::unique_ptr<ControllerRunningState<ValidationController>>   runningState;
    public: std::unique_ptr<ControllerEndState<ValidationController>>       endState;

    /// \brief Pointer to the current state.
    public: State<ValidationController> *currentState;
  };
}

#endif
