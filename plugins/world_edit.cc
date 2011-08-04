/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include "physics/physics.h"
#include "msgs/msgs.h"
#include "gazebo.h"

namespace gazebo
{
  class WorldEdit : public WorldPlugin
  {
    public: void Load( physics::WorldPtr _parent, sdf::ElementPtr &_sdf )
    {
      // Create a new transport node
      transport::NodePtr node(new transport::Node());

      // Initialize the node with the world name
      node->Init(_sdf->GetWorldName());
    
      // Create a publisher on the ~/scene topic 
      transport::PublisherPtr scenePub = node->Advertise<msgs::Scene>("~/scene");

      // Create a publisher on the ~/physics topic
      transport::PublisherPtr physicsPub = node->Advertise<msgs::Physics>("~/physics");

      // Set the ambient color to red
      msgs::Scene sceneMsg;
      msgs::Init(sceneMsg);
      sceneMsg.mutable_ambient()->set_r(1.0);
      sceneMsg.mutable_ambient()->set_g(0.0);
      sceneMsg.mutable_ambient()->set_b(0.0);
      sceneMsg.mutable_ambient()->set_a(1.0);
      scenePub->Publish( sceneMsg );

      // Set the step time
      msgs::Physics physicsMsg;
      msgs::Init(physicsMsg);
      physicsMsg.set_type(msgs::Physics::ODE);
      physicsMsg.set_dt(0.01);
      physicsPub->Publish( physicsMsg );
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldEdit)
}
