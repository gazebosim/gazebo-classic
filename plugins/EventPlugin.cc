/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>

#include <gazebo/msgs/msgs.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>

#include "plugins/EventPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(EventPlugin)

/////////////////////////////////////////////////
void EventPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "EventPlugin world pointer is NULL");
  GZ_ASSERT(_sdf, "EventPlugin sdf pointer is NULL");

  this->world = _world;
  this->sdf = _sdf;

  // Setup communication
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_world->GetName());

  // Read all the regions.
  sdf::ElementPtr regionElem = _sdf->GetElement("region");
  while (regionElem)
  {
    // Create a new region
    EventPlugin::Region *region = new EventPlugin::Region;

    // Get the region's name
    region->name = regionElem->Get<std::string>("name");

    // Get the region's pose and bounding box
    region->pose = regionElem->Get<math::Pose>("pose");
    region->box = regionElem->Get<math::Vector3>("box");

    // Get the message that should be transmitted when an event occurs.
    sdf::ElementPtr triggerElem = regionElem->GetElement("on_trigger");
    std::string topic = triggerElem->Get<std::string>("topic");
    sdf::ElementPtr msgElem = triggerElem->GetElement("msg");
    std::string data = msgElem->Get<std::string>("data");
    region->msg = data;

    // Create the event publisher
    region->pub = this->node->Advertise<msgs::GzString>(topic);

    // Store the region
    this->regions[region->name] = region;

    // Get the next region, if one exists.
    regionElem = regionElem->GetNextElement("region");
  }

  // Connect to the update event.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&EventPlugin::Update, this));
}

/////////////////////////////////////////////////
void EventPlugin::Update()
{
  // Get all the models.
  physics::Model_V models = this->world->GetModels();

  // Process each model.
  for (physics::Model_V::iterator iter = models.begin();
       iter != models.end(); ++iter)
  {
    // Skip models that are static
    if ((*iter)->IsStatic())
      continue;

    math::Pose modelPose = (*iter)->GetWorldPose();

    // Check if the model's pose is inside a specified region
    for (std::map<std::string, Region*>::iterator rIter = this->regions.begin();
         rIter != this->regions.end(); ++rIter)
    {
      math::Vector3 min = rIter->second->pose.pos -
                          rIter->second->box / 2.0;

      math::Vector3 max = rIter->second->pose.pos +
                          rIter->second->box / 2.0;

      // If inside, then transmit the desired message.
      if (modelPose.pos.x > min.x && modelPose.pos.x < max.x &&
          modelPose.pos.y > min.y && modelPose.pos.y < max.y &&
          modelPose.pos.z > min.z && modelPose.pos.z < max.z)
      {
        msgs::GzString msg;
        msg.set_data(rIter->second->msg);
        rIter->second->pub->Publish(msg);
      }
    }
  }
}
