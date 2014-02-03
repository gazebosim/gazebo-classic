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
#ifndef __GAZEBO_CART_TEST_PLUGIN_HH__
#define __GAZEBO_CART_TEST_PLUGIN_HH__

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gazebo.hh"

#define NUM_JOINTS 3
namespace gazebo
{
  class CartTestPlugin : public ModelPlugin
  {
    public: CartTestPlugin();
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
  };
}
#endif
