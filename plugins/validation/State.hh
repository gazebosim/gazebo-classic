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
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  // Forward declarations.
  class ValidationPlugin;

  /// \def
  enum class ValidationComponent_t
  {
    /// \brief .
    GAZEBO,
    /// \brief .
    CONTROLLER
  };

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
                  ValidationPlugin &_plugin,
                  const ValidationComponent_t &_componentType =
                      ValidationComponent_t::CONTROLLER);

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

    /// \brief Transport node.
    protected: ignition::transport::Node node;
  };
