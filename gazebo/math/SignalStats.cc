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
SignalStats::SignalStats()
  : data(0.0)
  , count(0)
{
}

//////////////////////////////////////////////////
SignalStats::~SignalStats()
{
}

//////////////////////////////////////////////////
unsigned int SignalStats::GetCount() const
{
  return this->count;
}

//////////////////////////////////////////////////
void SignalStats::Reset()
{
  this->data = 0;
  this->count = 0;
}

//////////////////////////////////////////////////
double SignalMean::Get() const
{
  if (this->count == 0)
  {
    return 0;
  }
  return this->data / this->count;
}

//////////////////////////////////////////////////
std::string SignalMean::GetShortName() const
{
  return "mean";
}

//////////////////////////////////////////////////
void SignalMean::Insert(double _data)
{
  this->data += _data;
  this->count++;
}

//////////////////////////////////////////////////
double SignalRootMeanSquare::Get() const
{
  if (this->count == 0)
  {
    return 0;
  }
  return sqrt(this->data / this->count);
}

//////////////////////////////////////////////////
std::string SignalRootMeanSquare::GetShortName() const
{
  return "rms";
}

//////////////////////////////////////////////////
void SignalRootMeanSquare::Insert(double _data)
{
  this->data += _data * _data;
  this->count++;
}

//////////////////////////////////////////////////
double SignalMaxAbsoluteValue::Get() const
{
  return this->data;
}

//////////////////////////////////////////////////
std::string SignalMaxAbsoluteValue::GetShortName() const
{
  return "max";
}

//////////////////////////////////////////////////
void SignalMaxAbsoluteValue::Insert(double _data)
{
  double fabsData = fabs(_data);
  if (fabsData > this->data)
  {
    this->data = fabsData;
  }
  this->count++;
}

