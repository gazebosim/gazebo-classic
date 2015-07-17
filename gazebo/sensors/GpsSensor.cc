/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
}

/////////////////////////////////////////////////
GpsSensor::~GpsSensor()
{
}

/////////////////////////////////////////////////
void GpsSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
void GpsSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  physics::EntityPtr parentEntity = this->world->GetEntity(this->parentName);
  this->parentLink = boost::dynamic_pointer_cast<physics::Link>(parentEntity);

  this->lastGpsMsg.set_link_name(this->parentName);

  this->topicName = "~/" + this->parentName + '/' + this->GetName();
  if (this->sdf->HasElement("topic"))
    this->topicName += '/' + this->sdf->Get<std::string>("topic");
  boost::replace_all(this->topicName, "::", "/");

  this->gpsPub = this->node->Advertise<msgs::GPS>(this->topicName, 50);

  // Parse sdf noise parameters
  sdf::ElementPtr gpsElem = this->sdf->GetElement("gps");

  // Load position noise parameters
  {
    sdf::ElementPtr posElem = gpsElem->GetElement("position_sensing");
    this->noises[GPS_POSITION_LATITUDE_NOISE_METERS] =
      NoiseFactory::NewNoiseModel(
          posElem->GetElement("horizontal")->GetElement("noise"));
    this->noises[GPS_POSITION_LONGITUDE_NOISE_METERS] =
      NoiseFactory::NewNoiseModel(
          posElem->GetElement("horizontal")->GetElement("noise"));
    this->noises[GPS_POSITION_ALTITUDE_NOISE_METERS] =
      NoiseFactory::NewNoiseModel(
          posElem->GetElement("vertical")->GetElement("noise"));
  }

  // Load velocity noise parameters
  {
    sdf::ElementPtr velElem = gpsElem->GetElement("velocity_sensing");
    this->noises[GPS_VELOCITY_LATITUDE_NOISE_METERS] =
      NoiseFactory::NewNoiseModel(
          velElem->GetElement("horizontal")->GetElement("noise"));
    this->noises[GPS_VELOCITY_LONGITUDE_NOISE_METERS] =
      NoiseFactory::NewNoiseModel(
          velElem->GetElement("horizontal")->GetElement("noise"));
    this->noises[GPS_VELOCITY_ALTITUDE_NOISE_METERS] =
      NoiseFactory::NewNoiseModel(
          velElem->GetElement("vertical")->GetElement("noise"));
  }
}

/////////////////////////////////////////////////
void GpsSensor::Fini()
{
  Sensor::Fini();
  this->parentLink.reset();
  this->sphericalCoordinates.reset();
}

//////////////////////////////////////////////////
void GpsSensor::Init()
{
  Sensor::Init();

  this->sphericalCoordinates = this->world->GetSphericalCoordinates();
}

//////////////////////////////////////////////////
bool GpsSensor::UpdateImpl(bool /*_force*/)
{
  // Get latest pose information
  if (this->parentLink)
  {
    // Measure position and apply noise
    {
      // Get postion in Cartesian gazebo frame
      math::Pose gpsPose = this->pose + this->parentLink->GetWorldPose();

      // Apply position noise before converting to global frame
      gpsPose.pos.x =
        this->noises[GPS_POSITION_LATITUDE_NOISE_METERS]->Apply(gpsPose.pos.x);
      gpsPose.pos.y =
        this->noises[GPS_POSITION_LONGITUDE_NOISE_METERS]->Apply(gpsPose.pos.y);
      gpsPose.pos.z =
        this->noises[GPS_POSITION_ALTITUDE_NOISE_METERS]->Apply(gpsPose.pos.z);

      // Convert to global frames
      ignition::math::Vector3d spherical = this->sphericalCoordinates->
        SphericalFromLocal(gpsPose.pos.Ign());
      this->lastGpsMsg.set_latitude_deg(spherical.X());
      this->lastGpsMsg.set_longitude_deg(spherical.Y());
      this->lastGpsMsg.set_altitude(spherical.Z());
    }

    // Measure velocity and apply noise
    {
      ignition::math::Vector3d gpsVelocity =
        this->parentLink->GetWorldLinearVel(this->pose.pos).Ign();

      // Convert to global frame
      gpsVelocity = this->sphericalCoordinates->GlobalFromLocal(gpsVelocity);

      // Apply noise after converting to global frame
      gpsVelocity.X() = this->noises[
          GPS_VELOCITY_LATITUDE_NOISE_METERS]->Apply(gpsVelocity.X());
      gpsVelocity.Y() = this->noises[
          GPS_VELOCITY_LONGITUDE_NOISE_METERS]->Apply(gpsVelocity.Y());
      gpsVelocity.Z() = this->noises[
          GPS_VELOCITY_ALTITUDE_NOISE_METERS]->Apply(gpsVelocity.Z());

      this->lastGpsMsg.set_velocity_east(gpsVelocity.X());
      this->lastGpsMsg.set_velocity_north(gpsVelocity.Y());
      this->lastGpsMsg.set_velocity_up(gpsVelocity.Z());
    }
  }
  this->lastMeasurementTime = this->world->GetSimTime();
  msgs::Set(this->lastGpsMsg.mutable_time(), this->lastMeasurementTime);

  if (this->gpsPub)
    this->gpsPub->Publish(this->lastGpsMsg);

  return true;
}

//////////////////////////////////////////////////
math::Angle GpsSensor::GetLongitude() const
{
  math::Angle angle;
  angle.SetFromDegree(this->lastGpsMsg.longitude_deg());
  return angle;
}

//////////////////////////////////////////////////
math::Angle GpsSensor::GetLatitude() const
{
  math::Angle angle;
  angle.SetFromDegree(this->lastGpsMsg.latitude_deg());
  return angle;
}

//////////////////////////////////////////////////
double GpsSensor::GetAltitude() const
{
  return this->lastGpsMsg.altitude();
}
