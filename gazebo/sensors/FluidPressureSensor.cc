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

#include "gazebo/sensors/FluidPressureSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("fluid_pressure", FluidPressureSensor)

/////////////////////////////////////////////////
FluidPressureSensor::FluidPressureSensor()
: Sensor(sensors::OTHER)
{
}

/////////////////////////////////////////////////
FluidPressureSensor::~FluidPressureSensor()
{
}

/////////////////////////////////////////////////
void FluidPressureSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
void FluidPressureSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  // Save the link to which this sensor is attached
  physics::EntityPtr parentEntity = this->world->GetEntity(this->parentName);
  this->parentLink = boost::dynamic_pointer_cast<physics::Link>(parentEntity);
  this->lastFpMsg.set_link_name(this->parentName);

  // Create a message publisher to send out measurements
  this->topicName = "~/" + this->parentName + '/' + this->GetName();
  if (this->sdf->HasElement("topic"))
    this->topicName += '/' + this->sdf->Get<std::string>("topic");
  boost::replace_all(this->topicName, "::", "/");
  this->fpPub = this->node->Advertise<msgs::FluidPressure>(this->topicName, 50);

  // Parse sdf noise parameters
  sdf::ElementPtr fpElem = this->sdf->GetElement("fluid_pressure");

  // Parse sdf noise parameters
  this->noises[FluidPressureNoisePascals] = NoiseFactory::NewNoiseModel(
    fpElem->->GetElement("noise"));
}

/////////////////////////////////////////////////
void FluidPressureSensor::Fini()
{
  Sensor::Fini();
  this->parentLink.reset();
}

//////////////////////////////////////////////////
void FluidPressureSensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
bool FluidPressureSensor::UpdateImpl(bool /*_force*/)
{
  // Get latest pose information
  if (this->parentLink)
  {
    // Measure position and apply noise
    math::Pose fpPose = this->pose + this->parentLink->GetWorldPose();

    // Reference: https://en.wikipedia.org/wiki/Atmospheric_pressure
    double p0 = this->world->GetPhysicsEngine()->GetPressure();              // Reference pressure
    double t0 = this->world->GetPhysicsEngine()->GetTemperature();           // Reference temperature
    double g  = this->world->GetPhysicsEngine()->GetGravity().GetLength();   // Gravity
    double L  = 0.0065;                                                      // Temperature lapse rate
    double R  = 8.31447;                                                     // Universal gas constant
    double M  = 0.0289644;                                                   // Molar mass of dry air
    double h  = fpPose.pos.z;                                                // Altitude          

    // Calculate the pressure based on the altitude
    double p = p0 * exp(1.0 - L * h / t0, g * M / R / L);

    // Apply noise to the pressure
    p = this->noises[FluidPressureNoisePascals].Apply(p);

    // Set the sensor value
    this->lastFpMsg.set_fluid_pressure(p);
  }

  // Set the measurement time
  this->lastMeasurementTime = this->world->GetSimTime();
  msgs::Set(this->lastFpMsg.mutable_time(), this->lastMeasurementTime);

  // If we are publishing the sensor measurements, do so now
  if (this->fpPub)
    this->fpPub->Publish(this->lastFpMsg);

  return true;
}

//////////////////////////////////////////////////
double FluidPressureSensor::GetFluidPressure() const
{
  return this->lastFpMsg.fluid_pressure();
}
