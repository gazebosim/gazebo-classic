/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include "gazebo/sensors/SensorFactory.hh"

#include "gazebo/common/common.hh"
#include "gazebo/math/gzmath.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/Noise.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/sensors/GpsSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("gps", GpsSensor)

/////////////////////////////////////////////////
GpsSensor::GpsSensor()
: Sensor(sensors::OTHER)
{
  this->horizontalPositionNoise = NoisePtr(new Noise());
  this->verticalPositionNoise = NoisePtr(new Noise());
}

/////////////////////////////////////////////////
GpsSensor::~GpsSensor()
{
  this->horizontalPositionNoise.reset();
  this->verticalPositionNoise.reset();
}

/////////////////////////////////////////////////
void GpsSensor::Load(const std::string &_worldName, sdf::ElementPtr &_sdf)
{
  Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
void GpsSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  this->parentEntity = this->world->GetEntity(this->parentName);
  this->lastGpsMsg.set_entity_name(this->parentName);

  this->topicName = "~/" + this->parentName + '/' + this->GetName();
  if (this->sdf->HasElement("topic"))
    this->topicName += '/' + this->sdf->GetValueString("topic");
  boost::replace_all(this->topicName, "::", "/");

  this->gpsPub = this->node->Advertise<msgs::GPS>(this->topicName, 50);

  // Todo: parse sdf noise parameters
  sdf::ElementPtr gpsElem = this->sdf->GetElement("gps");
  if (gpsElem)
  {
    sdf::ElementPtr posElem = gpsElem->GetElement("position");
    if (posElem)
    {
      {
        sdf::ElementPtr horElem = posElem->GetElement("horizontal");
        if (horElem)
          this->horizontalPositionNoise->Load(horElem->GetElement("noise"));
      }
      {
        sdf::ElementPtr vertElem = posElem->GetElement("vertical");
        if (vertElem)
          this->verticalPositionNoise->Load(vertElem->GetElement("noise"));
      }
    }
  }
}

/////////////////////////////////////////////////
void GpsSensor::Fini()
{
  Sensor::Fini();
  this->parentEntity.reset();
  this->sphericalCoordinates.reset();
}

//////////////////////////////////////////////////
void GpsSensor::Init()
{
  Sensor::Init();
  
  this->sphericalCoordinates = this->world->GetSphericalCoordinates();
}

//////////////////////////////////////////////////
void GpsSensor::UpdateImpl(bool /*_force*/)
{
  // Get latest pose information
  if (this->parentEntity)
  {
    math::Pose gpsPose = this->pose + this->parentEntity->GetWorldPose();

    // Apply position noise
    gpsPose.pos.x = this->horizontalPositionNoise->Apply(gpsPose.pos.x);
    gpsPose.pos.y = this->horizontalPositionNoise->Apply(gpsPose.pos.y);
    gpsPose.pos.z = this->verticalPositionNoise->Apply(gpsPose.pos.z);

    math::Vector3 spherical = this->sphericalCoordinates->Convert(gpsPose.pos);
    this->lastGpsMsg.set_latitude_deg(spherical.x);
    this->lastGpsMsg.set_longitude_deg(spherical.y);
    this->lastGpsMsg.set_altitude(spherical.z);
  }
  this->lastMeasurementTime = this->world->GetSimTime();
  msgs::Set(this->lastGpsMsg.mutable_time(), this->lastMeasurementTime);

  if (this->gpsPub)
    this->gpsPub->Publish(this->lastGpsMsg);
}
