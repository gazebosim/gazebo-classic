/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include "InRegionEventSource.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
InRegionEventSource::InRegionEventSource(transport::PublisherPtr _pub,
    physics::WorldPtr _world, const std::map<std::string, RegionPtr> &_regions)
  : EventSource(_pub, "region", _world), regions(_regions), isInside(false)
{
}

////////////////////////////////////////////////////////////////////////////////
void InRegionEventSource::Load(const sdf::ElementPtr _sdf)
{
  EventSource::Load(_sdf);
  if (_sdf->HasElement("model"))
    this->modelName = _sdf->Get<std::string>("model");
  else
    gzerr << this->name << " is missing a model element" << std::endl;

  if (_sdf->HasElement("region"))
    this->regionName = _sdf->Get<std::string>("region");
  else
    gzerr << this->name << " is missing a region element" << std::endl;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&InRegionEventSource::Update, this));
}

////////////////////////////////////////////////////////////////////////////////
void InRegionEventSource::Init()
{
  this->model = this->world->GetModel(this->modelName);
  if (!model)
  {
    gzerr << this->name << ": Model '" << this->modelName
        << "' does not exist" << std::endl;
  }

  std::map<std::string, RegionPtr>::const_iterator it =
    this->regions.find(this->regionName);
  if (it != this->regions.end())
  {
    this->region = it->second;
  }
  else
  {
    gzerr << this->name << ": Region '" << this->regionName
        << "' does not exist" << std::endl;
  }

  this->Info();
}

////////////////////////////////////////////////////////////////////////////////
void InRegionEventSource::Info() const
{
  std::stringstream ss;
  ss << "InRegionEventSource "
      << " model " << this->modelName << "  region [" << this->regionName
      << "]" << std::endl;

  for (auto v: this->region->boxes)
  {
    ss << "  Min ";
    ss << "[" << v.min.x << ", " << v.min.y << ", " << v.min.z << "]";
    ss << std::endl;
    ss << "  Max ";
    ss << "[" << v.max.x << ", " << v.max.y << ", " << v.max.z << "]\n";
  }
  ss << "  inside: " << this->isInside << std::endl;
  gzmsg << ss.str();
}

////////////////////////////////////////////////////////////////////////////////
void InRegionEventSource::Update()
{
  // model must exist
  if (!this->model)
    return;

  // region must exist
  if (!this->region)
    return;

  math::Vector3 point = this->model->GetWorldPose().pos;
  bool oldState = this->isInside;
  bool currentState = this->region->Contains(point);

  if (oldState != currentState)
  {
    this->isInside = currentState;
    std::string json = "{";
    if (this->isInside)
    {
      json += "\"state\":\"inside\",";
    }
    else
    {
      json += "\"state\":\"outside\",";
    }
    json += "\"region\":\"" + this->regionName + "\", ";
    json += "\"model\":\"" + this->modelName + "\"";
    json += "}";
    this->Emit(json);
  }
}
