/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
                                         physics::WorldPtr _world,
                                         const std::map<std::string, RegionPtr>
                                                                      &_regions)
  :EventSource(_pub, "region", _world), regions(_regions), isInside(false)
{
}

////////////////////////////////////////////////////////////////////////////////
void InRegionEventSource::Load(const sdf::ElementPtr &_sdf)
{
  EventSource::Load(_sdf);
  if (_sdf->HasElement("model"))
    this->modelName = _sdf->GetElement("model")->Get<std::string>();
  else
    gzerr << this->name << " is missing a model element" << std::endl;

  if (_sdf->HasElement("region"))
    this->regionName = _sdf->GetElement("region")->Get<std::string>();
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
  this->isInside = this->region->PointInRegion(point);
  if (oldState != this->isInside)
  {
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

////////////////////////////////////////////////////////////////////////////////
Volume::~Volume()
{
}

////////////////////////////////////////////////////////////////////////////////
bool Volume::PointInVolume(const math::Vector3 &_p) const
{
  return _p.x >= this->min.x && _p.x <= this->max.x &&
         _p.y >= this->min.y && _p.y <= this->max.y &&
         _p.z >= this->min.z && _p.z <= this->max.z;
}

////////////////////////////////////////////////////////////////////////////////
bool Region::PointInRegion(const math::Vector3 &_p) const
{
  for (unsigned int i = 0; i < this->volumes.size(); ++i)
  {
    if (this->volumes[i]->PointInVolume(_p))
    {
      return true;
    }
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////
void Region::Load(const sdf::ElementPtr &_sdf)
{
  sdf::ElementPtr child = _sdf->GetFirstElement();
  while (child)
  {
    std::string ename = child->GetName();
    if (ename == "volume")
    {
      VolumePtr volume = VolumePtr(new Volume());
      volume->min = child->GetElement("min")->Get<math::Vector3>();
      volume->max = child->GetElement("max")->Get<math::Vector3>();
      this->volumes.push_back(volume);
    }
    else if (ename == "name")
    {
      this->name = child->Get<std::string>();
    }
    else
    {
      std::string m;
      m += "Unexpected element \"" + ename + "\" in Region element";
      throw SimEventsException(m.c_str());
    }
    child = child->GetNextElement();
  }
}

