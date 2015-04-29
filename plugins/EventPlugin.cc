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

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>

#include "plugins/EventPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(EventPlugin)

/////////////////////////////////////////////////
EventPlugin::EventPlugin()
{
}

/////////////////////////////////////////////////
EventPlugin::~EventPlugin()
{
}

/////////////////////////////////////////////////
void EventPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  printf("LOAD\n");
  GZ_ASSERT(_world, "EventPlugin world pointer is NULL");
  GZ_ASSERT(_sdf, "EventPlugin sdf pointer is NULL");
  this->world = _world;
  this->sdf = _sdf;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_world->GetName());

  sdf::ElementPtr regionElem = _sdf->GetElement("region");
  while (regionElem)
  {
    EventPlugin::Region *region = new EventPlugin::Region;

    region->name = regionElem->Get<std::string>("name");

    region->pose = regionElem->Get<math::Pose>("pose");
    region->box = regionElem->Get<math::Vector3>("box");

    sdf::ElementPtr triggerElem = regionElem->GetElement("on_trigger");
    std::string topic = triggerElem->Get<std::string>("topic");
    sdf::ElementPtr msgElem = triggerElem->GetElement("msg");
    std::string data = msgElem->Get<std::string>("data");
    region->msg = data;

    std::cout << "Publish on topic[" << topic << "]\n";
    region->pub = this->node->Advertise<msgs::GzString>(topic);


    this->regions[region->name] = region;

    regionElem = regionElem->GetNextElement("region");
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&EventPlugin::Update, this));
}

/////////////////////////////////////////////////
void EventPlugin::Update()
{
  physics::Model_V models = this->world->GetModels();

  // Process each model.
  for (physics::Model_V::iterator iter = models.begin();
       iter != models.end(); ++iter)
  {
    // Skip models that are static
    if ((*iter)->IsStatic())
      continue;

    math::Pose modelPose = (*iter)->GetWorldPose();
    // std::cout << "ModelPose[" << modelPose << "]\n";
    for (std::map<std::string, Region*>::iterator rIter = this->regions.begin();
         rIter != this->regions.end(); ++rIter)
    {
      math::Vector3 min = rIter->second->pose.pos -
                          rIter->second->box / 2.0;

      math::Vector3 max = rIter->second->pose.pos +
                          rIter->second->box / 2.0;
      // std::cout << "Pose[" << rIter->second->pose.pos << "] Box[" << rIter->second->box << "] Min[" << min << "] Max[" << max << "] \n";

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
