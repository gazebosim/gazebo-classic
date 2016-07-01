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

#ifndef _GAZEBO_PLUGINS_EVENTS_INREGIONEVENTSOURCE_HH_
#define _GAZEBO_PLUGINS_EVENTS_INREGIONEVENTSOURCE_HH_

#include <map>
#include <string>
#include <vector>

#include "plugins/events/Region.hh"
#include "plugins/events/EventSource.hh"

namespace gazebo
{
  /// \brief The event generator class
  class InRegionEventSource: public EventSource
  {
    /// \brief Constructor
    /// \param[in] _pub the publisher for the SimEvents
    /// \param[in] _world Pointer to the world.
    /// \param[in] _regions dictionary of regions in the world
    public: InRegionEventSource(transport::PublisherPtr _pub,
                physics::WorldPtr _world,
                const std::map<std::string, RegionPtr> &_regions);

    /// \brief Initialize the event
    public: virtual void Init();

    /// \brief Called every simulation step
    public: void Update();

    /// \brief Prints data about the event source to the log (useful for debug)
    public: void Info() const;

    /// \brief Loads the full name of the model and the region from the world
    /// file.
    /// \param[in] _sdf
    public: virtual void Load(const sdf::ElementPtr _sdf);

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief The model used for the in region check.
    private: std::string modelName;

    /// \brief A pointer to the model
    /// looked up at initialization, to avoid doing lookups during updates
    private: physics::ModelPtr model;

    /// \brief The region used for the in region check.
    private: std::string regionName;

    /// \brief The region pointer
    private: RegionPtr region;

    /// \brief A map of region names to region pointers.
    private: const std::map<std::string, RegionPtr> &regions;

    /// \brief true if the model is currently inside the region
    private: bool isInside;
  };
}
#endif
