/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_FOOSBALL_PLUGIN_HH_
#define _GAZEBO_FOOSBALL_PLUGIN_HH_

#include <list>
#include <map>
#include <mutex>
#include <sdf/sdf.hh>
#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE FoosballPlugin : public ModelPlugin
  {
    /// \def Rod_t
    /// \brief A rod is composed by two joints (prismatic and revolute).
    typedef std::array<physics::JointPtr, 2> Rod_t;
    typedef std::map<std::string, std::vector<Rod_t>> Controller_t;

    /// \brief Constructor.
    public: FoosballPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    /// \param[in] _info Update information provided by the server.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief Callback executed every time a new hydra message is received.
    /// \param[in] _msg The hydra message.
    private: void OnHydra(ConstHydraPtr &_msg);

    /// \brief Initialize the starting position of the Hydra controllers.
    private: void Restart();

    private: void SwitchRod(std::vector<Rod_t> &_aController);

    /// \brief Pointer to model containing this plugin.
    private: physics::ModelPtr model;

    /// \brief SDF for this plugin.
    private: sdf::ElementPtr sdf;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Node used for using Gazebo communications.
    private: transport::NodePtr node;

    /// \brief Subscriber pointer.
    private: transport::SubscriberPtr hydraSub;

    /// \brief Reset pose of the left Hydra controller.
    private: math::Pose resetPoseLeft;

    /// \brief Reset pose of the right Hydra controller.
    private: math::Pose resetPoseRight;

    /// \brief Is Hydra control activated?
    private: bool activated = false;

    /// \brief Mutex to protect the hydra messages.
    private: std::mutex msgMutex;

    /// \brief Hydra messages.
    private: std::list<boost::shared_ptr<msgs::Hydra const>> hydraMsgs;

    private: physics::JointPtr rightModelRot, leftModelRot;
    private: physics::JointPtr rightModelTrans, leftModelTrans;
    private: math::Pose basePoseRight;
    private: math::Pose basePoseLeft;
    private: math::Pose leftStartPose;
    private: math::Pose rightStartPose;

    private: std::vector<Controller_t> controllers;
    private: bool leftTriggerPressed = false;
    private: bool rightTriggerPressed = false;
  };
}
#endif
