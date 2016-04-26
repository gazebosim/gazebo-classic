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
#include <ignition/transport/Node.hh>
#include <sdf/sdf.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"
#include "State.hh"
#include "ValidationController.hh"

namespace gazebo
{
  /// \brief Labels for states.
  static std::string kReadyState = "gazebo_ready";
  static std::string kSetState   = "gazebo_set";
  static std::string kGoState    = "gazebo_go";
  static std::string kEndState   = "gazebo_end";

  /// \brief State that handles the "ready" state.
  template <typename T>
  class GazeboReadyState : public State<T>
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
      // Is the controller ready?
      if (this->Feedback() == "controller_ready")
        this->fsm.ChangeState(*this->fsm.setState);

      //std::cout << "ReadyState::DoFeedback()" << std::endl;
    }
  };

  /// \brief State that handles the "set" state.
  template <typename T>
  class GazeboSetState : public State<T>
  {
    // Use class constructor from base class.
    using State<T>::State;

    // Documentation inherited.
    public: virtual void DoInitialize()
    {
      // Load the parameters.

      // Start the run.
      this->fsm.ChangeState(*this->fsm.goState);

      std::cout << "SetState::DoInitialize()" << std::endl;
    }
  };

  /// \brief State that handles the "go" state.
  template <typename T>
  class GazeboGoState : public State<T>
  {
    // Use class constructor from base class.
    using State<T>::State;

    public: virtual void DoInitialize()
    {
      std::cout << "GoState::DoInitialize()" << std::endl;
    }

    // Documentation inherited.
    public: virtual void DoFeedback()
    {
      if (this->Feedback() == "controller_end")
      {
        if (this->fsm.MoreRuns())
        {
          // Go for the next run.
          this->fsm.ChangeState(*this->fsm.readyState);
        }
        else
        {
          this->fsm.ChangeState(*this->fsm.endState);
        }
      }

      // std::cout << "RunState::DoFeedback()" << std::endl;
    }
  };

  /// \brief State that handles the "end" state.
  template <typename T>
  class GazeboEndState : public State<T>
  {
    // Use class constructor from base class.
    using State<T>::State;

    public: virtual void DoInitialize()
    {
      std::cout << "EndState::DoInitialize()" << std::endl;
    }
  };

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
    public: void ChangeState(State<ValidationPlugin> &_newState);

    /// \brief ToDo.
    public: bool MoreRuns() const;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief State machine states.
    public: std::unique_ptr<GazeboReadyState<ValidationPlugin>> readyState;
    public: std::unique_ptr<GazeboSetState<ValidationPlugin>>   setState;
    public: std::unique_ptr<GazeboGoState<ValidationPlugin>>    goState;
    public: std::unique_ptr<GazeboEndState<ValidationPlugin>>   endState;

    /// \brief Pointer to the current state.
    public: State<ValidationPlugin> *currentState;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ValidationPlugin)
}

#endif
