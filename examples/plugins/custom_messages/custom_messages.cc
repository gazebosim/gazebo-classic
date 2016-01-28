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
#include <boost/bind.hpp>
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"

#include "msgs/custom.pb.h"

namespace gazebo
{
  class CustomMessages : public ModelPlugin
  {
    public: void Load(physics::ModelPtr /*_parent*/, sdf::ElementPtr /*_sdf*/)
    {
      this->node.reset(new transport::Node());
      this->pub = node->Advertise<my_msgs::Custom>("~/my_msgs");

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&CustomMessages::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      my_msgs::Custom msg;
      msg.set_my_data("hello");
      this->pub->Publish(msg);
    }

    // Pointer to the model
    private: transport::NodePtr node;
    private: transport::PublisherPtr pub;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CustomMessages);
}
