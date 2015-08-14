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

#include <string>

#include "JointEventSource.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
JointEventSource::JointEventSource(transport::PublisherPtr _pub,
    physics::WorldPtr _world)
  : EventSource(_pub, "joint", _world),
    range(INVALID),
    isTriggered(false)
{
}

////////////////////////////////////////////////////////////////////////////////
void JointEventSource::Load(const sdf::ElementPtr _sdf)
{
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&JointEventSource::Update, this));

  EventSource::Load(_sdf);

  if (_sdf->HasElement("model"))
  {
    this->modelName = _sdf->Get<std::string>("model");
  }
  else
  {
    gzerr << this->name << " is missing a model element" << std::endl;
  }

  if (_sdf->HasElement("joint"))
  {
    this->jointName = _sdf->Get<std::string>("joint");
  }
  else
  {
    gzerr << this->name << " is missing a joint element" << std::endl;
  }

  if (_sdf->HasElement("range"))
  {
    sdf::ElementPtr rangeElem = _sdf->GetElement("range");

    if (rangeElem->HasElement("min"))
    {
      this->min = rangeElem->Get<double>("min");
    }
    else
    {
      gzerr << this->name << ": <range>"
          << " should have a min element." << std::endl;
    }

    if (rangeElem->HasElement("max"))
    {
      this->max = rangeElem->Get<double>("max");
    }
    else
    {
      gzerr << this->name << ": <range>"
          << " should have a max element." << std::endl;
    }

    if (rangeElem->HasElement("type"))
    {
      std::string typeStr = rangeElem->Get<std::string>("type");
      this->SetRangeFromString(typeStr);
      if (this->range == INVALID)
      {
        gzerr << this->name << " has an invalid \"" << typeStr
            << " \" range type. "
            << " It should be one of: \"position\","
            << " \"normalized_angle\", \"velocity\"  or \"applied_force\""
            << std::endl;
      }
    }
    else
    {
      gzerr << this->name << ": range is missing a type element" << std::endl;
    }
  }
  else
  {
    gzerr << this->name << " is missing a range element" << std::endl;
  }
}

////////////////////////////////////////////////////////////////////////////////
void JointEventSource::Init()
{
  this->Info();
}

////////////////////////////////////////////////////////////////////////////////
void JointEventSource::SetRangeFromString(const std::string &_rangeStr)
{
  if (_rangeStr == "position")
    this->range = POSITION;
  else if (_rangeStr == "normalized_angle")
    this->range = ANGLE;
  else if (_rangeStr == "applied_force")
    this->range = FORCE;
  else if (_rangeStr == "velocity")
    this->range = VELOCITY;
  else
    this->range = INVALID;
}

////////////////////////////////////////////////////////////////////////////////
std::string JointEventSource::GetRangeAsString() const
{
  std::string rangeStr;
  switch (this->range)
  {
    case POSITION:
      rangeStr = "position";
      break;
    case VELOCITY:
      rangeStr = "velocity";
      break;
    case ANGLE:
      rangeStr = "normalized_angle";
      break;
    case FORCE: rangeStr = "applied_force";
      break;
    default: rangeStr = "invalid";
      break;
  }
  return rangeStr;
}

////////////////////////////////////////////////////////////////////////////////
void JointEventSource::Info() const
{
  std::stringstream ss;
  ss << "JointEventSource: " << this->name
      << " model: " << this->modelName
      << " joint: " << this->jointName
      << " range: " << this->GetRangeAsString()
      << " min: " << this->min
      << " max: " << this->max
      << " triggered: " << this->isTriggered
      << std::endl;
  gzmsg << ss.str();
}

////////////////////////////////////////////////////////////////////////////////
bool JointEventSource::LookupJoint()
{
  if (!this->model)
  {
    this->model = this->world->GetModel(this->modelName);
    // if the model name is not found
    if (!this->model)
    {
      // look for a model with a name that starts with our model name
      for (unsigned int i = 0; i < this->world->GetModelCount(); ++i)
      {
        physics::ModelPtr m = this->world->GetModel(i);
        size_t pos = m->GetName().find(this->modelName);
        if (pos == 0)
        {
          this->model = m;
          break;
        }
      }
    }
  }
  // if we have a model, let's look for the joint (full joint name only)
  if (this->model && !this->joint)
  {
    this->joint = this->model->GetJoint(this->jointName);
  }

  if (!this->model || !this->joint)
  {
    return false;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void JointEventSource::Update()
{
  if (!this->LookupJoint())
    return;

  bool oldState = this->isTriggered;
  double value = 0;

  double position = this->joint->GetAngle(0).Radian();
  math::Angle a = this->joint->GetAngle(0);
  // get a value between -PI and PI
  a.Normalize();
  double angle = a.Radian();
  double force = this->joint->GetForce(0);
  double velocity = this->joint->GetVelocity(0);

  switch (this->range)
  {
    case POSITION:
    {
      value = position;
      break;
    }
    case VELOCITY:
    {
      value = velocity;
      break;
    }
    case ANGLE:
    {
      value = angle;
      break;
    }
    case FORCE:
    {
      value = force;
      break;
    }
    default:
      // we can't do anything useful. Hopefully the error message during load
      // has attracted the attention of the user.
      return;
  }

  // check if the state has changed
  bool currentState = value >= this->min && value <= this->max;
  if (oldState != currentState)
  {
    this->isTriggered = currentState;
    std::string json = "{";
    if (currentState)
    {
      json += "\"state\":\"in_range\",";
    }
    else
    {
      json += "\"state\":\"out_of_range\",";
    }
    json += "\"joint\":\"" + this->jointName + "\", ";
    json += "\"position\":\"" + std::to_string(position) + "\", ";
    json += "\"velocity\":\"" + std::to_string(velocity) + "\", ";
    json += "\"force\":\"" + std::to_string(force) + "\", ";
    if (this->range == ANGLE)
      json += "\"angle\":\"" + std::to_string(angle) + "\", ";

    json += "\"range\":\"" + this->GetRangeAsString() + "\", ";
    json += "\"min\":\"" + std::to_string(this->min) + "\", ";
    json += "\"max\":\"" + std::to_string(this->max) + "\", ";
    json += "\"value\":\"" + std::to_string(value) + "\", ";
    json += "\"model\":\"" + this->modelName + "\"";
    json += "}";
    this->Emit(json);
  }
}

