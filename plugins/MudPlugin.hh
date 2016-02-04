/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _MUD_PLUGIN_HH_
#define _MUD_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE MudPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: MudPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Callback for receipt of contact sensor messages.
    /// \param[in] _msg Contacts message from contact sensor.
    private: void OnContact(ConstContactsPtr &_msg);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief Transport node used for subscribing to contact sensor messages.
    private: transport::NodePtr node;

    /// \brief Subscriber to contact sensor messages.
    private: transport::SubscriberPtr contactSub;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    private: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    private: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    private: physics::ModelPtr model;

    /// \brief Name of model containing plugin.
    private: std::string modelName;

    /// \brief Pointer to canonical link in mud model. It is assumed that
    ///        the canonical link has the contact sensor.
    private: physics::LinkPtr link;

    /// \brief Name of contact sensor relative to model, scoped with '/',
    private: std::string contactSensorName;

    /// \brief Mutex to protect reads and writes.
    private: mutable boost::mutex mutex;

    /// \brief Store newest contacts message.
    private: msgs::Contacts newestContactsMsg;

    /// \brief Flag to indicate new contact message.
    private: bool newMsg;

    /// \brief Number of updates without having a new contacts message.
    private: unsigned int newMsgWait;

    /// \brief Stiffness parameter, used in conjunction with damping
    ///        to compute joint erp, cfm
    private: double stiffness;

    /// \brief Damping parameter, used in conjunction with stiffness
    ///        to compute joint erp, cfm
    private: double damping;

    /// \brief Names of allowed target links, specified in sdf parameters.
    private: std::vector<std::string> allowedLinks;

    /// \brief Pointer to link currently targeted by mud joint.
    private: std::vector<physics::LinkPtr> links;

    /// \brief Dynamically created joint for simulating mud forces.
    private: std::vector<physics::JointPtr> joints;

    /// \brief Custom bitmask associated to collision surface of allowed links
    private: unsigned int contactSurfaceBitmask;

    /// \brief SDF for this plugin;
    private: sdf::ElementPtr sdf;
  };
}
#endif  // ifndef _MUD_PLUGIN_HH_
