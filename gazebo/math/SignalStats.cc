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
#include <boost/algorithm/string.hpp>
#include "gazebo/math/SignalStats.hh"

using namespace gazebo;
using namespace math;

//////////////////////////////////////////////////
SignalStatistic::SignalStatistic()
  : data(0.0)
  , count(0)
{
}

//////////////////////////////////////////////////
SignalStatistic::~SignalStatistic()
{
}

//////////////////////////////////////////////////
unsigned int SignalStatistic::Count() const
{
  return this->count;
}

//////////////////////////////////////////////////
void SignalStatistic::Reset()
{
  this->data = 0;
  this->count = 0;
}

//////////////////////////////////////////////////
double SignalMean::Value() const
{
  if (this->count == 0)
  {
    return 0;
  }
  return this->data / this->count;
}

//////////////////////////////////////////////////
std::string SignalMean::ShortName() const
{
  return "Mean";
}

//////////////////////////////////////////////////
void SignalMean::InsertData(double _data)
{
  this->data += _data;
  this->count++;
}

//////////////////////////////////////////////////
double SignalRootMeanSquare::Value() const
{
  if (this->count == 0)
  {
    return 0;
  }
  return sqrt(this->data / this->count);
}

//////////////////////////////////////////////////
std::string SignalRootMeanSquare::ShortName() const
{
  return "Rms";
}

//////////////////////////////////////////////////
void SignalRootMeanSquare::InsertData(double _data)
{
  this->data += _data * _data;
  this->count++;
}

//////////////////////////////////////////////////
double SignalMaxAbsoluteValue::Value() const
{
  return this->data;
}

//////////////////////////////////////////////////
std::string SignalMaxAbsoluteValue::ShortName() const
{
  return "MaxAbs";
}

//////////////////////////////////////////////////
void SignalMaxAbsoluteValue::InsertData(double _data)
{
  double absData = std::abs(_data);
  if (absData > this->data)
  {
    this->data = absData;
  }
  this->count++;
}

//////////////////////////////////////////////////
SignalStats::SignalStats()
{
}

//////////////////////////////////////////////////
SignalStats::~SignalStats()
{
}

//////////////////////////////////////////////////
unsigned int SignalStats::Count() const
{
  unsigned int count = 0;
  if (!this->stats.empty())
  {
    count = this->stats.front()->Count();
  }
  return count;
}

//////////////////////////////////////////////////
std::map<std::string, double> SignalStats::Map() const
{
  std::map<std::string, double> map;
  for (SignalStatistic_V::const_iterator iter = this->stats.begin();
       iter != this->stats.end(); ++iter)
  {
    map[(*iter)->ShortName()] = (*iter)->Value();
  }
  return map;
}

//////////////////////////////////////////////////
void SignalStats::InsertData(double _data)
{
  for (SignalStatistic_V::iterator iter = this->stats.begin();
       iter != this->stats.end(); ++iter)
  {
    (*iter)->InsertData(_data);
  }
}

//////////////////////////////////////////////////
bool SignalStats::InsertStatistic(const std::string &_name)
{
  // Check if the statistic is already inserted
  {
    std::map<std::string, double> map = this->Map();
    if (map.find(_name) != map.end())
    {
      std::cerr << "Unable to InsertStatistic ["
                << _name
                << "] since it has already been inserted."
                << std::endl;
      return false;
    }
  }

  SignalStatisticPtr stat;
  if (_name == "MaxAbs")
  {
    stat.reset(new SignalMaxAbsoluteValue());
    this->stats.push_back(stat);
  }
  else if (_name == "Mean")
  {
    stat.reset(new SignalMean());
    this->stats.push_back(stat);
  }
  else if (_name == "Rms")
  {
    stat.reset(new SignalRootMeanSquare());
    this->stats.push_back(stat);
  }
  else
  {
    // Unrecognized name string
    std::cerr << "Unable to InsertStatistic ["
              << _name
              << "] since it is an unrecognized name."
              << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
bool SignalStats::InsertStatistics(const std::string &_names)
{
  if (_names.empty())
  {
    std::cerr << "Unable to InsertStatistics "
              << "since no names were supplied."
              << std::endl;
    return false;
  }

  bool result = true;
  std::vector<std::string> names;
  boost::split(names, _names, boost::is_any_of(","));
  for (std::vector<std::string>::iterator iter = names.begin();
       iter != names.end(); ++iter)
  {
    result = result && this->InsertStatistic(*iter);
  }
  return result;
}

//////////////////////////////////////////////////
void SignalStats::Reset()
{
  for (SignalStatistic_V::iterator iter = this->stats.begin();
       iter != this->stats.end(); ++iter)
  {
    (*iter)->Reset();
  }
}

//////////////////////////////////////////////////
Vector3Stats::Vector3Stats()
{
}

//////////////////////////////////////////////////
Vector3Stats::~Vector3Stats()
{
}

//////////////////////////////////////////////////
void Vector3Stats::InsertData(const Vector3 &_data)
{
  this->x.InsertData(_data.x);
  this->y.InsertData(_data.y);
  this->z.InsertData(_data.z);
  this->mag.InsertData(_data.GetLength());
}

//////////////////////////////////////////////////
bool Vector3Stats::InsertStatistic(const std::string &_name)
{
  bool result = true;
  result = result && this->x.InsertStatistic(_name);
  result = result && this->y.InsertStatistic(_name);
  result = result && this->z.InsertStatistic(_name);
  result = result && this->mag.InsertStatistic(_name);
  return result;
}

//////////////////////////////////////////////////
bool Vector3Stats::InsertStatistics(const std::string &_names)
{
  bool result = true;
  result = result && this->x.InsertStatistics(_names);
  result = result && this->y.InsertStatistics(_names);
  result = result && this->z.InsertStatistics(_names);
  result = result && this->mag.InsertStatistics(_names);
  return result;
}

//////////////////////////////////////////////////
void Vector3Stats::Reset()
{
  this->x.Reset();
  this->y.Reset();
  this->z.Reset();
  this->mag.Reset();
}

