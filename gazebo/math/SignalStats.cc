/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include <cmath>
#include "gazebo/math/SignalStats.hh"

using namespace gazebo;
using namespace math;

//////////////////////////////////////////////////
SignalMean::SignalMean() : SignalStats()
  , sum(0.0)
  , count(0)
{
}

//////////////////////////////////////////////////
SignalMean::~SignalMean()
{
}

//////////////////////////////////////////////////
double SignalMean::Get() const
{
  if (this->count == 0)
  {
    return 0;
  }
  return this->sum / this->count;
}

//////////////////////////////////////////////////
unsigned int SignalMean::GetCount() const
{
  return this->count;
}

//////////////////////////////////////////////////
void SignalMean::Insert(double _data)
{
  this->sum += _data;
  this->count++;
}

//////////////////////////////////////////////////
void SignalMean::Reset()
{
  this->sum = 0;
  this->count = 0;
}

//////////////////////////////////////////////////
SignalRootMeanSquare::SignalRootMeanSquare() : SignalMean()
{
}

//////////////////////////////////////////////////
SignalRootMeanSquare::~SignalRootMeanSquare()
{
}

//////////////////////////////////////////////////
double SignalRootMeanSquare::Get() const
{
  if (this->count == 0)
  {
    return 0;
  }
  return sqrt(this->sum / this->count);
}

//////////////////////////////////////////////////
void SignalRootMeanSquare::Insert(double _data)
{
  this->sum += _data * _data;
  this->count++;
}

