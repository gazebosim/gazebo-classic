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

#ifndef _GAZEBO_PLUGINS_VALIDATION_STATE_
#define _GAZEBO_PLUGINS_VALIDATION_STATE_

#include <memory>
#include <mutex>
#include <string>
#include <ignition/transport/Node.hh>
#include "gazebo/common/Time.hh"
#include "gazebo/common/Timer.hh"
#include "gazebo/msgs/gz_string.pb.h"
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  /// \def
  enum class ValidationComponent_t
  {
    /// \brief .
    GAZEBO,
    /// \brief .
    CONTROLLER
  };

  /// \brief State pattern used for the state machine.
  template <typename T>
  class State
  {
    /// \brief Class constructor.
    /// \param[in] _name Name of the state.
    /// \param[in] _plugin Pointer to the model plugin.
    public: State(const std::string &_name,
                  T &_fsm,
                  const ValidationComponent_t &_componentType =
                      ValidationComponent_t::CONTROLLER)
      : name(_name),
        fsm(_fsm)
    {
      std::string inboundTopic = "/gazebo/validation/state";
      this->outboundTopic = "/gazebo/validation/feedback";
      if (_componentType == ValidationComponent_t::GAZEBO)
        std::swap(inboundTopic, outboundTopic);

      // This is the topic where we receive feedback.
      this->node.Subscribe(inboundTopic, &State::OnFeedback, this);

      // testing
      gazebo::msgs::GzString msg;
      ignition::transport::Node n;
      n.Advertise<gazebo::msgs::GzString>(outboundTopic);
      // testing

      // Advertise the current state.
      this->node.Advertise<gazebo::msgs::GzString>(outboundTopic);
    }

    /// \brief Update the state.
    public: void Update()
    {
      //std::lock_guard<std::mutex> lock(this->mutex);
      if (!this->initialized)
        this->Initialize();

      this->DoUpdate();

      // Time to publish the current state?
      auto now = gazebo::common::Time::GetWallTime();
      if (now - this->lastPublication >= this->publicationInterval)
        this->PublishState();
    }

    /// \brief Called once before changing to a different state.
    public: void Teardown()
    {
      //std::lock_guard<std::mutex> lock(this->mutex);
      this->initialized = false;

      //std::cout << "State::Teardown()" << std::endl;
      this->DoTeardown();

      this->PublishState();
    }

    /// \brief Equal to operator.
    /// \param[in] _state The state to compare against.
    /// \return true if the state has the same name.
    public: bool operator ==(const State &_state) const
    {
      return this->name == _state.name;
    }

    /// \brief Not equal to operator
    /// \param[in] _state The state to compare against
    /// \return true if the state doesn't have the same name.
    public: bool operator !=(const State &_state) const
    {
      return !(*this == _state);
    }

    /// \brief Return the current feedback value.
    protected: std::string Feedback() const
    {
      //std::lock_guard<std::mutex> lock(this->mutex);
      return this->feedback;
    }

    /// \brief ToDo.
    private: virtual void DoInitialize()
    {
      std::cout << "State::DoInitialize" << std::endl;
    }

    /// \brief ToDo.
    private: virtual void DoUpdate()
    {

    }

    /// \brief ToDo.
    private: virtual void DoTeardown()
    {

    }

    /// \brief ToDo.
    private: virtual void DoFeedback()
    {

    }

    /// \brief Initialize the state. Called once after a pause duration after
    /// entering state.
    private: void Initialize()
    {
      std::cout << "State::Initialize()" << std::endl;
      this->initialized = true;
      this->timer.Reset();
      this->timer.Start();
      this->DoInitialize();
    }

    /// \brief Callback that updates the feedback member variable.
    private: void OnFeedback(const gazebo::msgs::GzString &_msg)
    {
      //std::cout << "State::OnFeedback()" << std::endl;
      //std::lock_guard<std::mutex> lock(this->mutex);
      this->feedback = _msg.data();

      // Only if we're the active state.
      if (this->initialized)
        this->DoFeedback();
    }

     /// \brief Publish the current state.
    private: void PublishState()
    {
      gazebo::msgs::GzString msg;
      msg.set_data(this->name);
      this->node.Publish(this->outboundTopic, msg);
      this->lastPublication = common::Time::GetWallTime();
    }

    /// \brief Name of the state.
    public: const std::string name;

    /// \brief Reference to the finite state machine.
    protected: T &fsm;

    /// \brief Timer to measure time in the current state.
    protected: common::Timer timer;

    /// \brief Has initialized
    protected: bool initialized = false;

    /// \brief Transport node.
    protected: ignition::transport::Node node;

    /// \brief ToDo.
    private: std::string outboundTopic;

    /// \brief Last feedback message.
    private: std::string feedback;

    /// \brief Mutex to protect the feedback message.
    private: mutable std::mutex mutex;

    /// \brief Last time the we published the state.
    private: common::Time lastPublication;

    /// \brief Elapsed time between state publications.
    private: common::Time publicationInterval;
  };
}
#endif
