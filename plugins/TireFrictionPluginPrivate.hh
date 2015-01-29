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
#ifndef _GAZEBO_TIRE_FRICTION_PLUGIN_PRIVATE_HH_
#define _GAZEBO_TIRE_FRICTION_PLUGIN_PRIVATE_HH_

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  /// \brief Private class attributes for TireFrictionPlugin
  class TireFrictionPluginPrivate
  {
    /// \brief SDF for this plugin;
    public: sdf::ElementPtr sdf;

    /// \brief Pointer to world.
    public: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    public: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    public: physics::ModelPtr model;

    /// \brief Pointer to link with attached tire.
    public: physics::LinkPtr link;

    /// \brief Pointer to tire collision.
    public: physics::CollisionPtr collision;

    /// \brief Connection to World Update events.
    public: event::ConnectionPtr updateConnection;

    // The following attributes are all related to receiving contact
    // information from transport topics.

    /// \brief Transport node used for subscribing to contact messages.
    public: transport::NodePtr node;

    /// \brief Subscriber to contact messages.
    public: transport::SubscriberPtr contactSub;

    /// \brief Mutex to protect reads and writes to newestContactsMsg.
    public: mutable boost::mutex mutex;

    /// \brief Store newest contacts message.
    public: msgs::Contacts newestContactsMsg;

    /// \brief Flag to indicate new contact message.
    public: bool newMsg;

    /// \brief Number of updates without having a new contacts message.
    public: common::Time newMsgWait;

    // The following attributes are related to the friction vs slip model.

    /// \brief Static coefficient of friction.
    public: double frictionStatic;

    /// \brief Dynamic coefficient of friction.
    public: double frictionDynamic;

    /// \brief Tire slip at static friction.
    public: double slipStatic;

    /// \brief Tire slip at dynamic friction.
    public: double slipDynamic;

    /// \brief Reference speed below which static friction is used.
    public: double speedStatic;
  };
}
#endif
