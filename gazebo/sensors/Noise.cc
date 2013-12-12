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


#include <boost/math/special_functions/round.hpp>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/math/Helpers.hh"
#include "gazebo/math/Rand.hh"

#include "gazebo/sensors/Noise.hh"

using namespace gazebo;
using namespace sensors;

//////////////////////////////////////////////////
Noise::Noise()
  : type(NONE),
    noiseModel(NULL)
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

  if (this->type == GAUSSIAN ||
      this->type == GAUSSIAN_QUANTIZED)
  {
    this->noiseModel = new GaussianNoiseModel();
    this->noiseModel->Load(this->sdf);
  }
}

//////////////////////////////////////////////////
double Noise::Apply(double _in) const
{
  if (this->type == NONE)
    return _in;
  else if (this->type == CUSTOM)
    return this->customNoiseCallback(_in);
  else
    return this->noiseModel->Apply(_in);
}

//////////////////////////////////////////////////
Noise::NoiseType Noise::GetNoiseType() const
{
  return this->type;
}

//////////////////////////////////////////////////
void Noise::SetCustomNoiseCallback(
    boost::function<double(double)> _cb)

{
  this->customNoiseCallback = _cb;
}


//////////////////////////////////////////////////
NoiseModel *Noise::GetNoiseModel() const
{
  return this->noiseModel;
}


//////////////////////////////////////////////////
NoiseModel::NoiseModel()
{
}

//////////////////////////////////////////////////
void NoiseModel::Load(sdf::ElementPtr /*_sdf*/)
{
}

//////////////////////////////////////////////////
double NoiseModel::Apply(double _in) const
{
  return _in;
}

//////////////////////////////////////////////////
GaussianNoiseModel::GaussianNoiseModel()
  : NoiseModel(),
    mean(0.0),
    stdDev(0.0),
    bias(0.0),
    precision(0.0)
{
}

//////////////////////////////////////////////////
void GaussianNoiseModel::Load(sdf::ElementPtr _sdf)
{
  this->mean = _sdf->Get<double>("mean");
  this->stdDev = _sdf->Get<double>("stddev");
  // Sample the bias
  double biasMean = _sdf->Get<double>("bias_mean");
  double biasStdDev = _sdf->Get<double>("bias_stddev");
  this->bias = math::Rand::GetDblNormal(biasMean, biasStdDev);
  // With equal probability, we pick a negative bias (by convention,
  // rateBiasMean should be positive, though it would work fine if
  // negative).
  if (math::Rand::GetDblUniform() < 0.5)
    this->bias = -this->bias;
  gzlog << "applying Gaussian noise model with mean " << this->mean
    << ", stddev " << this->stdDev
    << ", bias " << this->bias << std::endl;

  this->quantized = false;
  if (_sdf->HasElement("precision"))
  {
    this->precision = _sdf->Get<double>("precision");
    this->quantized = true;
  }
}

//////////////////////////////////////////////////
double GaussianNoiseModel::Apply(double _in) const
{
  double output = 0.0;

  double whiteNoise = math::Rand::GetDblNormal(this->mean, this->stdDev);
  output = _in + this->bias + whiteNoise;
  if (this->quantized)
  {
    // Apply this->precision
    if (!math::equal(this->precision, 0.0, 1e-6))
    {
      output = boost::math::round(output / this->precision) * this->precision;
    }
  }

  return output;
}

//////////////////////////////////////////////////
double GaussianNoiseModel::GetMean() const
{
  return this->mean;
}

//////////////////////////////////////////////////
double GaussianNoiseModel::GetStdDev() const
{
  return this->stdDev;
}

//////////////////////////////////////////////////
double GaussianNoiseModel::GetBias() const
{
  return this->bias;
}
