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

#ifndef _SPHERE_HYDRADEMO_PLUGIN_HH_
#define _SPHERE_HYDRADEMO_PLUGIN_HH_

#include <boost/thread/mutex.hpp>
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

namespace gazebo
{
  class HydraDemoPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: HydraDemoPlugin();

    /// \brief Destructor
    public: virtual ~HydraDemoPlugin();

    // Documentation Inherited.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    /// \param[in] _info Update information provided by the server.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief Callback executed every time a new hydra message is received.
    /// \param[in] _msg The hydra message.
    private: void OnHydra(ConstHydraPtr &_msg);

    /// \brief World pointer
    private: physics::WorldPtr world;

    /// \brief Model pointer
    private: physics::ModelPtr model;

    /// \brief Node used for using Gazebo communications.
    private: transport::NodePtr node;

    /// \brief Subscribe pointer.
    private: transport::SubscriberPtr hydraSub;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief Mutex to protect hydraMsgPtr.
    private: boost::mutex msgMutex;

    /// \brief Store the last message from hydra.
    private: boost::shared_ptr<const gazebo::msgs::Hydra> hydraMsgPtr;
  };
}
#endif
