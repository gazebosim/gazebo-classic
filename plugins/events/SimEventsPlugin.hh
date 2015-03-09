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
 * limitations under the License.k *
*/

#ifndef _SIMEVENTS_PLUGIN_HH_
#define _SIMEVENTS_PLUGIN_HH_

#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "SimEventsException.hh"
#include "InRegionEventSource.hh"
#include "ExistenceEventSource.hh"
#include "SimStateEventSource.hh"

namespace gazebo
{
  class SimEventsPlugin : public WorldPlugin
  {
    /// \brief Destrutor
    public: virtual ~SimEventsPlugin();

    /// \brief Called when the world file is loaded
    /// \param[in] _world the word
    /// \param[in] _sdf the plugin sdf element
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Called upon initialization
    public: void Init();

    /// \brief Called every simulation step
    public: void Update();

    /// \brief callback for ~/model/info topic
    /// \param[in] _msg model message
    private: void OnModelInfo(ConstModelPtr &_msg);

    /// \brief callback for ~/request topic
    /// \param [in] _msg the request message
    private: void OnRequest(ConstRequestPtr &_msg);

    /// \brief world pointer to get simulation state
    private: physics::WorldPtr world;

    /// \brief sdf element to read event
    private: sdf::ElementPtr sdf;

    /// \brief All the regions defined in the XML for scoring
    private: std::map<std::string, RegionPtr> regions;

    /// \brief List of all sim event emitters
    private: std::vector<EventSourcePtr> events;

    /// \brief the publisher for SimEvents
    private: transport::PublisherPtr pub;

    /// \brief subscription to the model/info
    private: transport::SubscriberPtr spawnSub;

    /// \brief known models that have been spawned already
    private: std::set<std::string> models;

    /// \brief subscription to the request topic
    private: transport::SubscriberPtr requestSub;
  };
}

#endif
