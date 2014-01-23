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
#include "gazebo/gazebo_core.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
  class ProjectorPlugin : public ModelPlugin
  {
    public: ProjectorPlugin():state(true) {}
    public: ~ProjectorPlugin() {}

    //////////////////////////////////////////////////
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Create a new transport node
      this->node.reset(new transport::Node());

      // Initialize the node with the world name
      this->node->Init(_parent->GetWorld()->GetName());

      std::string name = std::string("~/") + _parent->GetName() + "/" +
                          _sdf->Get<std::string>("projector");

      // Create a publisher on the ~/physics topic
      this->projectorPub = node->Advertise<msgs::Projector>(name);

      this->prevTime = common::Time::GetWallTime();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ProjectorPlugin::OnUpdate, this));
    }

    //////////////////////////////////////////////////
    public: void OnUpdate()
    {
      if (common::Time::GetWallTime() - this->prevTime > common::Time(2, 0))
      {
        this->state = !this->state;
        msgs::Projector msg;
        msg.set_name("texture_projector");
        msg.set_enabled(this->state);
        this->projectorPub->Publish(msg);
        this->prevTime = common::Time::GetWallTime();
      }
    }

    private: transport::NodePtr node;
    private: transport::PublisherPtr projectorPub;
    private: common::Time prevTime;
    private: event::ConnectionPtr updateConnection;
    private: bool state;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ProjectorPlugin)
}
