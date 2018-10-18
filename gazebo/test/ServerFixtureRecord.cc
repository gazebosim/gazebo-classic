/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <string>

#include "gazebo/math/SignalStats.hh"
#include "gazebo/math/Vector3Stats.hh"
#include "ServerFixture.hh"

using namespace gazebo;

/////////////////////////////////////////////////
void ServerFixture::Record(const std::string &_name, const double _data)
{
  std::ostringstream stream;
  stream << _data;
  RecordProperty(_name, stream.str());
}

/////////////////////////////////////////////////
void ServerFixture::Record(const std::string &_prefix,
                           const math::SignalStats &_stats)
{
  auto map = _stats.Map();
  for (auto const &stat : map)
  {
    this->Record(_prefix + stat.first, stat.second);
  }
}

/////////////////////////////////////////////////
void ServerFixture::Record(const std::string &_prefix,
                           const math::Vector3Stats &_stats)
{
  this->Record(_prefix + "_x_", _stats.X());
  this->Record(_prefix + "_y_", _stats.Y());
  this->Record(_prefix + "_z_", _stats.Z());
  this->Record(_prefix + "_mag_", _stats.Mag());
}
