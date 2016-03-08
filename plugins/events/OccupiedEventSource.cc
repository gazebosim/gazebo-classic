/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>

#include <gazebo/msgs/msgs.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>

#include "plugins/events/OccupiedEventSource.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
OccupiedEventSource::OccupiedEventSource(transport::PublisherPtr _pub,
    physics::WorldPtr _world, const std::map<std::string, RegionPtr> &_regions)
  : EventSource(_pub, "occupied", _world), regions(_regions)
{
}

/////////////////////////////////////////////////
void OccupiedEventSource::Load(const sdf::ElementPtr _sdf)
{
  std::string topic;
  std::string data;

  GZ_ASSERT(_sdf, "OccupiedEventSource sdf pointer is NULL");
  EventSource::Load(_sdf);

  if (_sdf->HasElement("region"))
    this->regionName = _sdf->Get<std::string>("region");
  else
  {
    gzerr << "SimEventPlugin event[" << this->name << "] "
          << "is missing a region element. This event will be ignored.\n";
  }

  // Get the topic name
  if (_sdf->HasElement("topic"))
    topic = _sdf->Get<std::string>("topic");
  else
  {
    gzerr << "Missing <topic>, child of <event> with name[" << this->name
      << "]. This event will be skipped.\n";
  }

  // Get message data
  if (_sdf->HasElement("msg_data"))
  {
    data = _sdf->Get<std::string>("msg_data");
  }
  else
  {
    gzerr << "Missing <msg_data>, child of <event> with name[" << this->name
      << "]. This event will be skipped.\n";
  }

  auto regionIter = this->regions.find(this->regionName);
  if (regionIter == this->regions.end())
  {
    gzerr << "Unkown region with name[" << this->regionName << "] "
          << "in <event> with name[" << this->name << "]. "
          << "This event will be skipped.\n";
  }

  // Setup communications and connect to world update if everything is okay.
  if (!topic.empty() && !data.empty() && regionIter != this->regions.end())
  {
    // Setup communication
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->world->GetName());
    this->msgPub = this->node->Advertise<gazebo::msgs::GzString>(topic);

    this->msg.set_data(data);

    // Connect to the update event.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&OccupiedEventSource::Update, this));
  }
}

/////////////////////////////////////////////////
void OccupiedEventSource::Update()
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

    // If inside, then transmit the desired message.
    if (this->regions[this->regionName]->Contains((*iter)->GetWorldPose().pos))
    {
      this->msgPub->Publish(this->msg);
    }
  }
}
