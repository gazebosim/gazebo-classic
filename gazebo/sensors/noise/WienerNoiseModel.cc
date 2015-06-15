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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/math/Helpers.hh"
#include "gazebo/math/Rand.hh"
#include "gazebo/sensors/noise/WienerNoiseModel.hh"

using namespace gazebo;
using namespace sensors;

//////////////////////////////////////////////////
WienerNoiseModel::WienerNoiseModel()
  : Noise(Noise::WIENER), mean(0.0), stdDev(0.0), rate(0.0), erro(0.0)
{
}

//////////////////////////////////////////////////
WienerNoiseModel::~WienerNoiseModel()
{
}

//////////////////////////////////////////////////
void WienerNoiseModel::Load(sdf::ElementPtr _sdf)
{
  Noise::Load(_sdf);
  this->mean = _sdf->Get<double>("mean");
  this->stdDev = _sdf->Get<double>("stddev");
}

//////////////////////////////////////////////////
void WienerNoiseModel::Fini()
{
  Noise::Fini();
}

//////////////////////////////////////////////////
double WienerNoiseModel::ApplyImpl(double _in)
{  
  // Error perturbation = carry error + reversion term + noise term
  this->error += this->mean + math::Rand::GetDblNormal(0.0, this->stdDev);

  // Return the error
  return (_in + this->error);
}

//////////////////////////////////////////////////
double WienerNoiseModel::GetMean() const
{
  return this->mean;
}

//////////////////////////////////////////////////
double WienerNoiseModel::GetStdDev() const
{
  return this->stdDev;
}
