/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef GAZEBO_PLUGINS_CARTDEMOPLUGIN_HH_
#define GAZEBO_PLUGINS_CARTDEMOPLUGIN_HH_
#include <ignition/transport/Node.hh>

#include "gazebo/common/PID.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"

#define NUM_JOINTS 3

namespace gazebo
{
  /// \brief This plugin drives a four wheeled cart model forward and back
  /// by applying a small wheel torque.  Steering is controlled via
  /// a position pid.
  /// this is a test for general rolling contact stability.
  /// should refine the test to be more specific in the future.
  class GAZEBO_VISIBLE CartDemoPlugin : public ModelPlugin
  {
    public: CartDemoPlugin();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void OnUpdate();

    private: transport::NodePtr node;

    private: event::ConnectionPtr updateConnection;

    private: physics::ModelPtr model;

    private: physics::JointPtr joints[NUM_JOINTS];
    private: common::PID jointPIDs[NUM_JOINTS];
    private: double jointPositions[NUM_JOINTS];
    private: double jointVelocities[NUM_JOINTS];
    private: double jointMaxEfforts[NUM_JOINTS];

    private: common::Time prevUpdateTime;

    // Place ignition::transport objects at the end of this file to
    // guarantee they are destructed first.

    /// \brief Ignition communication node
    private: ignition::transport::Node nodeIgn;
  };
}
#endif
