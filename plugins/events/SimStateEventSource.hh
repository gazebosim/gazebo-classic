/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#ifndef _SIMSTATE_EVENTSOURCE_HH_
#define _SIMSTATE_EVENTSOURCE_HH_

#include "EventSource.hh"


namespace gazebo
{
  /// \brief SimEvent that fires when the simulation is paused/resumed
  class SimStateEventSource: public EventSource
  {
    /// \brief Constructor
    /// \param[in] _pub publisher for the SimEvents
    /// \param[in] _world pointer to the world.
    public: SimStateEventSource(transport::PublisherPtr _pub,
                                physics::WorldPtr _world);

    /// \brief Dtor
    public: virtual ~SimStateEventSource();

    /// \brief Load the name of the event from the world file
    /// \param[in] _sdf the event element in the world file
    public: virtual void Load(const sdf::ElementPtr _sdf);

    /// \brief Update for every time step
    /// \param[in] _info Update information provided by the server.
    public: virtual void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Callback for the pause event
    /// \param[in] _p true if the sim has been paused
    public: void OnPause(bool _p);

    /// brief True if the simulation is paused.
    private: bool hasPaused;

    /// \brief Pointer to the Gazebo pause event connection
    private: event::ConnectionPtr pauseConnection;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief Simulation time at the previous step
    private: common::Time simTime;
  };
}

#endif
