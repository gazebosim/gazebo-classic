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

#include "gazebo/common/common.hh"
#include "gazebo/math/gzmath.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/noise/Noise.hh"
#include "gazebo/sensors/OrientationSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("orientation", OrientationSensor)

/////////////////////////////////////////////////
OrientationSensor::OrientationSensor()
        : Sensor(sensors::OTHER)
{
}

/////////////////////////////////////////////////
OrientationSensor::~OrientationSensor()
{
}

/////////////////////////////////////////////////
void OrientationSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
void OrientationSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  physics::EntityPtr parentEntity = this->world->GetEntity(this->parentName);
  this->parentLink = boost::dynamic_pointer_cast<physics::Link>(parentEntity);

  this->lastMagnetometerMsg.set_link_name(this->parentName);

  this->topicName = "~/" + this->parentName + '/' + this->GetName();
  if (this->sdf->HasElement("topic"))
    this->topicName += '/' + this->sdf->Get<std::string>("topic");
  boost::replace_all(this->topicName, "::", "/");

  this->orientPub = this->node->Advertise<msgs::Magnetometer>(this->topicName, 50);

  // Parse sdf noise parameters
  sdf::ElementPtr orientElem = this->sdf->GetElement("orientation");

  // Load noise parameters
  {
    sdf::ElementPtr fieldElem = orientElem->GetElement("euler_angle");
    this->noises[MagneticFieldNoiseX] = NoiseFactory::NewNoiseModel(
      fieldElem->GetElement("body_x")->GetElement("noise"));
    this->noises[MagneticFieldNoiseY] =  NoiseFactory::NewNoiseModel(
      fieldElem->GetElement("body_y")->GetElement("noise"));
    this->noises[MagneticFieldNoiseZ] = NoiseFactory::NewNoiseModel(
      fieldElem->GetElement("body_z")->GetElement("noise"));
  }
}

/////////////////////////////////////////////////
void OrientationSensor::Fini()
{
  Sensor::Fini();
  this->parentLink.reset();
}

//////////////////////////////////////////////////
void OrientationSensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
bool OrientationSensor::UpdateImpl(bool /*_force*/)
{
  // Get latest pose information
  if (this->parentLink)
  {
    // Measure position and apply noise
    {
      // Get pose in gazebo reference frame
      math::Pose magPose = this->pose + this->parentLink->GetWorldPose();

      // Get the reference magnetic field
      math::Vector3 M = this->world->GetPhysicsEngine()->GetMagneticField();

      // Rotate the magnetic field into the body frame
      M = magPose.rot.GetInverse().RotateVector(M);

      // Apply position noise before converting to global frame
      M.x = this->noises[OrientationNoiseX]->Apply(M.x);
      M.y = this->noises[OrientationNoiseY]->Apply(M.y);
      M.z = this->noises[OrientationNoiseZ]->Apply(M.z);
    }
  }

  // Save the time of the measurement
  this->lastMeasurementTime = this->world->GetSimTime();
  msgs::Set(this->lastOrientMsg.mutable_time(), this->lastMeasurementTime);

  // Publish the message if needed
  if (this->orientPub)
    this->orientPub->Publish(this->lastOrientMsg);

  return true;
}

//////////////////////////////////////////////////
math::Vector3 OrientationSensor::GetMagneticField() const
{
  boost::mutex::scoped_lock lock(this->mutex);
  return msgs::Convert(this->magMsg.magnetic_field());
}