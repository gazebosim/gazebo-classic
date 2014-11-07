/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _MODEL_CONTROL_PLUGIN_HH_
#define _MODEL_CONTROL_PLUGIN_HH_

#include <string>
#include <vector>

#include <ignition/transport.hh>

#include <boost/thread/condition.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE ModelControlPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: ModelControlPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Callback for receipt of model control request
    /// \param[in] _msg Model message from gazebo
    private: void RequestControl();

    private: void OnControlResponse(gazebo::msgs::ControlResponse &_msg);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief Transport node used for publishing to control requests
    private: ignition::transport::Node node;

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

    /// \brief Mutex to protect reads and writes.
    private: mutable boost::mutex mutex;

    /// \brief Pointer to links
    private: std::vector<physics::LinkPtr> links;

    /// \brief Dynamically created joint for simulating joint forces.
    private: std::vector<physics::JointPtr> joints;

    /// \brief SDF for this plugin;
    private: sdf::ElementPtr sdf;

    /// \brief local copy of response
    private: gazebo::msgs::ControlResponse controlResponse;
    private: physics::ModelPtr targetModel;

    /// \brief capturing atlas pelvis pose
    private: physics::LinkPtr pelvisLink;

    /// \brief capturing atlas right hand link
    private: physics::LinkPtr endEffectorLink;

    /// \brief synchronization mechanism
    private: boost::condition_variable delayCondition;

    /// \brief joint ids without legs
    private: std::vector<unsigned int> jointId;
  };
}
#endif
