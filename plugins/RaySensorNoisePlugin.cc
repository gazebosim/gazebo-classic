/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#include "gazebo/sensors/Noise.hh"
#include "RaySensorNoisePlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(RaySensorNoisePlugin)

/////////////////////////////////////////////////
RaySensorNoisePlugin::RaySensorNoisePlugin()
{
  this->fixedNoise = 0.2;
}

/////////////////////////////////////////////////
RaySensorNoisePlugin::~RaySensorNoisePlugin()
{
  this->parentSensor.reset();
}

/////////////////////////////////////////////////
void RaySensorNoisePlugin::Load(sensors::SensorPtr _parent,
    sdf::ElementPtr /*_sdf*/)
{
  // Get then name of the parent sensor
  this->parentSensor =
    boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parentSensor)
    gzthrow("RaySensorNoisePlugin requires a Ray Sensor as its parent");

  sensors::NoisePtr noise = this->parentSensor->GetNoise();

  if (noise)
  {
    noise->SetCustomNoiseCallback(
      boost::bind(&RaySensorNoisePlugin::OnApplyNoise, this, _1));
  }
  else
  {
    gzerr << "No noise found. Please add noise element to you ray sensor sdf "
        << "and set noise type to \"custom\"" << std::endl;
  }
}

/////////////////////////////////////////////////
double RaySensorNoisePlugin::OnApplyNoise(double _in)
{
  // Apply alternating noise.
  this->fixedNoise *= -1;
  return this->fixedNoise + _in;
}
