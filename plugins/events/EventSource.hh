/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#ifndef _EVENTSOURCE_HH_
#define _EVENTSOURCE_HH_

#include <string>

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/TransportTypes.hh"

#include "SimEventsException.hh"

namespace gazebo
{
  /// \brief The base class for emitting SimEvents
  class EventSource
  {
    /// \brief Constructor
    /// \param[in] _pub the publisher where the events are emitted
    /// \param[in] _type the type of event
    /// \param[in] _world Pointer to the world (in order to get model refs, etc)
    public: EventSource(transport::PublisherPtr _pub,
                        const std::string& _type,
                        physics::WorldPtr _world);

    /// \brief Destructor
    public: virtual ~EventSource();

    /// \brief emit an event with data to the internal publisher
    /// (and using the internal type)
    /// \param[in] _data the JSON data related to this event.
    public: void Emit(const std::string& _data);

    /// \brief Load from an sdf element (with possible configuration data)
    /// \param[in] _sdf the sdf element for the event in the world file
    public: virtual void Load(const sdf::ElementPtr &_sdf);

    /// \brief Initialize the event
    public: virtual void Init();

    /// \brief An event source can be used to enable other events. Inactive
    /// events do not generate an message when Emit is called.
    /// \return true if the event is active
    public: virtual bool IsActive();

    /// \brief Name of the event.
    protected: std::string name;

    /// \brief Type of event
    protected: std::string type;

    /// \brief Pointer to the world.
    protected: physics::WorldPtr world;

    /// \brief True if the event source is active.
    /// Inactive event sources do not emit events
    protected: bool active;

    /// \brief a way to send messages to the other topics (to the REST)
    protected: transport::PublisherPtr pub;
  };

  typedef boost::shared_ptr<EventSource> EventSourcePtr;

  /// \brief Gazebo events to detect model creation/deletion
  class  SimEventConnector
  {
    /// \brief Connect a boost::slot to the spawn model event
    /// \param[in] _subscriber the subscriber to this event
    /// \return a connection
    public: template<typename T>
        static event::ConnectionPtr
            ConnectSpawnModel(T _subscriber)
      { return spawnModel.Connect(_subscriber); }

    /// \brief Disconnect a boost::slot to the spawn model event
    /// \param[in] _subscriber the subscriber to this event
    public: static void DisconnectSpawnModel(
        event::ConnectionPtr _subscriber)
      { spawnModel.Disconnect(_subscriber); }

    /// \brief A model has been completed and uploaded onto the server.
    public: static event::EventT<void (std::string, bool)> spawnModel;
  };
}

#endif
