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
#include "gazebo/sensors/noise/ExponentialNoiseModel.hh"

using namespace gazebo;
using namespace sensors;

//////////////////////////////////////////////////
ExponentialNoiseModel::ExponentialNoiseModel()
  : Noise(Noise::EXPONENTIAL), mean(0.0), stdDev(0.0), rate(0.0), erro(0.0)
{
}

//////////////////////////////////////////////////
ExponentialNoiseModel::~ExponentialNoiseModel()
{
}

//////////////////////////////////////////////////
void ExponentialNoiseModel::Load(sdf::ElementPtr _sdf)
{
  Noise::Load(_sdf);
  this->rate = _sdf->Get<double>("rate");
}

//////////////////////////////////////////////////
void ExponentialNoiseModel::Fini()
{
  Noise::Fini();
}

//////////////////////////////////////////////////
double ExponentialNoiseModel::ApplyImpl(double _in)
{  
  return _in - log(1.0-math::Rand::GetDblUniform(0.0,1.0))/this->rate;
}

//////////////////////////////////////////////////
double ExponentialNoiseModel::GetRate() const
{
  return this->rate;
}
