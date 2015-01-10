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

#include <stdio.h>
#include <string>

#include "gazebo/math/SignalStats.hh"
#include "test/ServerFixture.hh"

using namespace gazebo;

/////////////////////////////////////////////////
void ServerFixture::Record(const std::string &_name, double _data)
{
    std::ostringstream stream;
    stream << _data;
    RecordProperty(_name, stream.str());
}

/////////////////////////////////////////////////
void ServerFixture::Record(const std::string &_prefix,
                           const math::SignalStats &_stats)
{
  std::map<std::string, double> map = _stats.Map();
  for (std::map<std::string, double>::iterator iter = map.begin();
       iter != map.end(); ++iter)
  {
    Record(_prefix + iter->first, iter->second);
  }
}

/////////////////////////////////////////////////
void ServerFixture::Record(const std::string &_prefix,
                           const math::Vector3Stats &_stats)
{
  Record(_prefix + "_x_", _stats.X());
  Record(_prefix + "_y_", _stats.Y());
  Record(_prefix + "_z_", _stats.Z());
  Record(_prefix + "_mag_", _stats.Mag());
}

