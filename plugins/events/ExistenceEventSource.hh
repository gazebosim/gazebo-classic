/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_EXISTENCEVENTSOURCE_HH_
#define _GAZEBO_EXISTENCEVENTSOURCE_HH_

#include <string>

#include "EventSource.hh"

namespace gazebo
{
  class  ExistenceEventSource: public EventSource
  {
    /// \brief Constructor
    /// \param[in] _pub the publisher for SimEvents
    /// \param[in] _world the word
    public: ExistenceEventSource(transport::PublisherPtr _pub,
                                 physics::WorldPtr _world);

    /// \brief Reads the model filter and event name.
    /// \param[in] _sdf the element for this event source
    public: virtual void Load(const sdf::ElementPtr _sdf);

    /// \brief callback for gazebo event
    /// \param[in] _model the name of the model
    /// \param[in] _alive true for spawn, false for delete
    public: void OnExistence(const std::string &_model, bool _alive);

    /// \brief A filter to raise the event only for models
    /// with a name that starts with this model specific prefix.
    /// When empty, the existence event is raised for each
    /// model creation and destruction
    private: std::string model;

    /// \brief The Gazebo event, to receive a call when a new model is spawned
    /// or deleted
    private: event::ConnectionPtr existenceConnection;
  };
}

#endif
