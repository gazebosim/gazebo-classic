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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/sensors/GaussianNoiseModel.hh"
#include "gazebo/sensors/Noise.hh"

using namespace gazebo;
using namespace sensors;

//////////////////////////////////////////////////
NoisePtr NoiseManager::LoadNoiseModel(sdf::ElementPtr _sdf,
    const std::string &_sensorType)
{
  GZ_ASSERT(_sdf != NULL, "noise sdf is NULL");
  std::string typeString = _sdf->Get<std::string>("type");

  NoisePtr noise;
  if (typeString == "gaussian" ||
      typeString == "gaussian_quantized")
  {
    if (_sensorType == "camera" || _sensorType == "depth" ||
      _sensorType == "multicamera")
    {
      noise.reset(new ImageGaussianNoiseModel());
    }
    else
      noise.reset(new GaussianNoiseModel());
  }
  else
  {
    noise.reset(new Noise());
  }
  noise->Load(_sdf);

  return noise;
}

//////////////////////////////////////////////////
Noise::Noise()
  : type(NONE),
    customNoiseCallback(NULL)
{
}

//////////////////////////////////////////////////
Noise::~Noise()
{
}

//////////////////////////////////////////////////
void Noise::Load(sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  GZ_ASSERT(this->sdf != NULL, "this->sdf is NULL");
  std::string typeString = this->sdf->Get<std::string>("type");
  if (typeString == "none")
    this->type = NONE;
  else if (typeString == "gaussian")
    this->type = GAUSSIAN;
  else if (typeString == "gaussian_quantized")
    this->type = GAUSSIAN_QUANTIZED;
  else if (typeString == "custom")
    this->type = CUSTOM;
  else
  {
    gzerr << "Unrecognized noise type: [" << typeString << "]"
          << ", using default [none]" << std::endl;
    this->type = NONE;
  }
}

//////////////////////////////////////////////////
double Noise::Apply(double _in) const
{
  if (this->type == NONE)
    return _in;
  else if (this->type == CUSTOM)
  {
    if (this->customNoiseCallback)
      return this->customNoiseCallback(_in);
    else
    {
      gzerr << "Custom noise callback function not set!"
          << " Please call SetCustomNoiseCallback within a sensor plugin."
          << std::endl;
      return _in;
    }
  }
  else
    return this->ApplyImpl(_in);
}

//////////////////////////////////////////////////
double Noise::ApplyImpl(double _in) const
{
  return _in;
}

//////////////////////////////////////////////////
Noise::NoiseType Noise::GetNoiseType() const
{
  return this->type;
}

//////////////////////////////////////////////////
void Noise::SetCustomNoiseCallback(boost::function<double (double)> _cb)
{
  this->type = CUSTOM;
  this->customNoiseCallback = _cb;
}

//////////////////////////////////////////////////
void Noise::Fini()
{
  this->customNoiseCallback = NULL;
}
