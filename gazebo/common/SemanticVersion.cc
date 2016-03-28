/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <string>
#include <iostream>
#include <algorithm>

#include "SemanticVersion.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
SemanticVersion::SemanticVersion(const std::string &_versionStr)
  :maj(0), min(0), patch(0)
{
  if (_versionStr.empty())
    return;
  unsigned int points = std::count(_versionStr.begin(), _versionStr.end(), '.');
  if (points == 0)
    sscanf(_versionStr.c_str(), "%5u", &this->maj);
  if (points == 1)
    sscanf(_versionStr.c_str(), "%5u.%5u", &this->maj, &this->min);
  if (points >= 2)
  {
    sscanf(_versionStr.c_str(), "%5u.%5u.%5u", &this->maj, &this->min,
        &this->patch);
  }
}

/////////////////////////////////////////////////
SemanticVersion::SemanticVersion(const unsigned int _maj,
  const unsigned int _min,
  const unsigned int _patch)
{
  this->maj = _maj;
  this->min = _min;
  this->patch = _patch;
}

/////////////////////////////////////////////////
std::string SemanticVersion::Version() const
{
  return std::to_string(this->maj) + "." + std::to_string(this->min) \
    + "." + std::to_string(this->patch);
}

/////////////////////////////////////////////////
bool SemanticVersion::operator<(const SemanticVersion &_other) const
{
  if (this == &_other)
    return false;

  if (this->maj < _other.maj)
  {
    return true;
  }
  if (this->maj > _other.maj)
  {
    return false;
  }
  if (this->min < _other.min)
  {
    return true;
  }
  if (this->min > _other.min)
  {
    return false;
  }
  if (this->patch < _other.patch)
  {
    return true;
  }
  if (this->patch > _other.patch)
  {
    return false;
  }
  // _other is equal
  return false;
}

/////////////////////////////////////////////////
bool SemanticVersion::operator<=(const SemanticVersion &_other) const
{
  return (_other > *this) || (_other == *this);
}

/////////////////////////////////////////////////
bool SemanticVersion::operator>(const SemanticVersion &_other) const
{
  return (_other < *this);
}

/////////////////////////////////////////////////
bool SemanticVersion::operator>=(const SemanticVersion &_other) const
{
  return (_other < *this) || (_other == *this);
}

/////////////////////////////////////////////////
bool SemanticVersion::operator==(const SemanticVersion &_other) const
{
  if ( &_other == this)
    return true;

  return (_other.maj == this->maj) && (_other.min == this->min) \
    && (_other.patch == this->patch);
}

/////////////////////////////////////////////////
bool SemanticVersion::operator!=(const SemanticVersion &_other) const
{
  return !(*this == _other);
}
