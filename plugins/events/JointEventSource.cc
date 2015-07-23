/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "JointEventSource.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
JointEventSource::JointEventSource(transport::PublisherPtr _pub,
    physics::WorldPtr _world)
  : EventSource(_pub, "joint", _world),
    positionRange(false),
    isPositionTriggered(false),
    referenceAngleRange(false),
    isReferenceAngleTriggered(false)
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

  if (_sdf->HasElement("min_position") && _sdf->HasElement("max_position"))
  {
    this->minPosition = _sdf->Get<double>("min_position");
    this->maxPosition = _sdf->Get<double>("max_position");
    this->positionRange = true;
  }

  if (_sdf->HasElement("min_reference_angle") &&
      _sdf->HasElement("max_reference_angle"))
  {
    this->minReferenceAngle = _sdf->Get<double>("min_reference_angle");
    this->maxReferenceAngle = _sdf->Get<double>("max_reference_angle");
    this->referenceAngleRange = true;
  }

/*
  else
  {
    if (_sdf->HasElement("min_position") || _sdf->HasElement("max_position"))
    {
        gzerr << this->name << ": min_position or max_position missing"
          << std::endl;
    }
  }
*/
}

////////////////////////////////////////////////////////////////////////////////
void JointEventSource::Init()
{

  this->Info();
}

////////////////////////////////////////////////////////////////////////////////
void JointEventSource::Info() const
{
  std::stringstream ss;
  ss << "JointEventSource "
      << " model " << this->modelName
      << " joint " << this->jointName
      << " min_position " << this->minPosition
      << " max_position" << this->maxPosition
      << " position trigger " << this->isPositionTriggered
      << std::endl;
  gzmsg << ss.str();
}

////////////////////////////////////////////////////////////////////////////////
bool JointEventSource::LookupJoint()
{
  if(!this->model)
  {
    this->model = this->world->GetModel(this->modelName);
  }

  if(this->model && !this->joint)
  {
    this->joint = this->model->GetJoint(this->jointName);
  }

  if(!this->model || !this->joint)
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

//  gzerr << this->modelName << "::" << this->jointName
//        << " pos: "  << this->joint->GetAngle(0)
//        << " vel: " << this->joint->GetVelocity(0)
//        << " f: "   << this->joint->GetForce(0)
//        << std::endl;
  if(this->positionRange)
  {
    bool oldState = this->isPositionTriggered;
    double pos = this->joint->GetAngle(0).Radian();
    bool currentState = pos > this->minPosition && pos < this->maxPosition;
    if (oldState != currentState)
    {
      this->isPositionTriggered = currentState;
      std::string json = "{";
      if (currentState)
      {
        json += "\"state\":\"in_position\",";
      }
      else
      {
        json += "\"state\":\"out_of_position\",";
      }
      json += "\"joint\":\"" + this->jointName + "\", ";
      json += "\"model\":\"" + this->modelName + "\"";
      json += "}";

      gzerr << json << std::endl;
      this->Emit(json);
    }
  }

  if(this->positionRange)
  {
    bool oldState = this->isPositionTriggered;
    double pos = this->joint->GetAngle(0).Radian();
    bool currentState = pos > this->minPosition && pos < this->maxPosition;
    if (oldState != currentState)
    {
      this->isPositionTriggered = currentState;
      std::string json = "{";
      if (currentState)
      {
        json += "\"state\":\"in_position\",";
      }
      else
      {
        json += "\"state\":\"out_of_position\",";
      }
      json += "\"joint\":\"" + this->jointName + "\", ";
      json += "\"model\":\"" + this->modelName + "\"";
      json += "}";

      gzerr << json << std::endl;
      this->Emit(json);
    }
  }
}

