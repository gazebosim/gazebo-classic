/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <functional>
#include <ignition/math/Rand.hh>
#include "gazebo/sensors/Noise.hh"
#include "RaySensorNoisePlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(RaySensorNoisePlugin)

/////////////////////////////////////////////////
RaySensorNoisePlugin::RaySensorNoisePlugin()
{
  this->fixedNoiseRate = 0.005;
  this->sign = 1;
}

/////////////////////////////////////////////////
RaySensorNoisePlugin::~RaySensorNoisePlugin()
{
}

/////////////////////////////////////////////////
void RaySensorNoisePlugin::Load(sensors::SensorPtr _parent,
    sdf::ElementPtr /*_sdf*/)
{
  if (!_parent)
  {
    gzerr << "RaySensorNoisePlugin requires a ray sensor as its parent.\n";
    return;
  }

  sensors::NoisePtr noise = _parent->Noise(sensors::RAY_NOISE);

  if (noise)
  {
    noise->SetCustomNoiseCallback(
      std::bind(&RaySensorNoisePlugin::OnApplyNoise, this,
        std::placeholders::_1));
  }
  else
  {
    gzwarn << "No noise found. Please add noise element to you ray sensor sdf "
        << "and set noise type to \"custom\"" << std::endl;
  }
}

/////////////////////////////////////////////////
double RaySensorNoisePlugin::OnApplyNoise(double _in)
{
  // Apply alternating random noise.
  double randNoise = ignition::math::Rand::DblUniform(0, this->fixedNoiseRate);
  this->sign *= -1;
  return _in + this->sign*randNoise*_in;
}
