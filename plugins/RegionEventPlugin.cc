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

#include "plugins/RegionEventPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(RegionEventPlugin)

/////////////////////////////////////////////////
void RegionEventPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "RegionEventPlugin world pointer is NULL");
  GZ_ASSERT(_sdf, "RegionEventPlugin sdf pointer is NULL");

  this->world = _world;
  this->sdf = _sdf;

  // Setup communication
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_world->GetName());

  // Read all the regions.
  sdf::ElementPtr regionElem = _sdf->GetElement("region");
  while (regionElem)
  {
    bool create = true;
    std::string regionName;
    math::Pose pose;
    math::Vector3 box;
    std::string topic;
    std::string msgType;
    std::string data;

    // Get the region's name
    if (regionElem->HasElement("name"))
    {
      std::string regionName = regionElem->Get<std::string>("name");
    }
    else
    {
      create = false;
      gzerr << "Missing 'name' attribute for <region> in "
        << "event plugin region[" << region->name
        << "]. This region is skipped.\n";
    }

    // Get the region's pose
    if (regionElem->HasElement("pose"))
    {
      create = true;
      pose = regionElem->Get<math::Pose>("pose");
    }
    else
    {
      create = false;
      gzerr << "Missing <pose>, child of <region>, for event plugin "
        << "region[" << region->name << "]. This region is skipped.\n";
    }

    // Get the region's bounding box
    if (regionElem->HasElement("box"))
    {
      box = regionElem->Get<math::Vector3>("box");
    }
    else
    {
      create = false;
      gzerr << "Missing <box>, child of <region>, for event plugin "
        << "region[" << region->name << "]. This region is skipped.\n";
    }

    // Get the message that should be transmitted when an event occurs.
    if (regionElem->HasElement("on_trigger"))
    {
      sdf::ElementPtr triggerElem = regionElem->GetElement("on_trigger");
      sdf::ElementPtr msgElem;

      // Get the topic name
      if (triggerElem->HasElement("topic"))
        topic = triggerElem->Get<std::string>("topic");
      else
      {
        create = false;
        gzerr << "Missing <topic>, child of <on_trigger>, "
          << "in event plugin region[" << region->name
          << "]. This region is skipped.\n";
      }

      // Get the message information to transmit
      if (triggerElem->HasElement("msg"))
      {
        sdf::ElementPtr msgElem = triggerElem->GetElement("msg");

        // Get message type
        if (msgElem->HasAttribuet("type"))
        {
          msgType = msgElem->Get<std::string>("type");
        }
        else
        {
          create = false;
          gzerr << "Missing 'type' attribute for <msg> element "
            << "in event plugin region[" << region->name
            << "]. This region is skipped.\n";
        }

        // Get message data
        if (msg->HasElement("data"))
        {
          data = msgElem->Get<std::string>("data");
        }
        else
        {
          create = false;
          gzerr << "Missing <data>, child of <msg>, in event plugin "
            << "region[" << region->name << "]. This region is skipped.\n";
        }
      }
      else
      {
        create = false;
        gzerr << "Missing <msg>, child of <on_trigger>, "
          << "in event plugin region[" << region->name
          << "]. This region is skipped.\n";
      }
    }
    else
    {
      create = false;
      gzerr << "Missing <on_trigger> element in event plugin region["
        << region->name << "]. This region is skipped.\n";
    }

    // Create the event if all the elements were found.
    if (create)
    {
      // Create a new region
      RegionEventPlugin::Region *region = new RegionEventPlugin::Region;
      region->name = regionName;
      region->pose = pose;
      region->box = box;
      region->msg = data;

      // Create the event publisher
      region->pub = this->node->Advertise<msgs::GzString>(topic);

      // Store the region
      this->regions[region->name] = region;
    }

    // Get the next region, if one exists.
    regionElem = regionElem->GetNextElement("region");
  }

  // Connect to the update event.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RegionEventPlugin::Update, this));
}

/////////////////////////////////////////////////
void RegionEventPlugin::Update()
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
