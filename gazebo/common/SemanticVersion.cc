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

namespace gazebo
{
  namespace common
  {
    class SemanticVersionPrivate
    {
      /// \brief Major revision (incompatible api changes)
      public: unsigned int maj;

      /// \brief Minor revision (backwards compatible new functionality)
      public: unsigned int min;

      /// \brief Patch (bug fixes)
      public: unsigned int patch;
    };
  }
}

/////////////////////////////////////////////////
SemanticVersion::SemanticVersion(const std::string &_versionStr)
  :dataPtr(new SemanticVersionPrivate())
{
  if (_versionStr.empty())
    return;
  unsigned int points = std::count(
    _versionStr.begin(), _versionStr.end(), '.');
  if (points == 0)
    sscanf(_versionStr.c_str(), "%5u", &this->dataPtr->maj);
  if (points == 1)
    sscanf(_versionStr.c_str(),
      "%5u.%5u", &this->dataPtr->maj, &this->dataPtr->min);
  if (points >= 2)
  {
    sscanf(_versionStr.c_str(),
      "%5u.%5u.%5u", &this->dataPtr->maj, &this->dataPtr->min,
        &this->dataPtr->patch);
  }
}

/////////////////////////////////////////////////
SemanticVersion::~SemanticVersion()
{
}

/////////////////////////////////////////////////
SemanticVersion::SemanticVersion(const unsigned int _maj,
  const unsigned int _min,
  const unsigned int _patch)
{
  this->dataPtr->maj = _maj;
  this->dataPtr->min = _min;
  this->dataPtr->patch = _patch;
}

/////////////////////////////////////////////////
std::string SemanticVersion::Version() const
{
  return std::to_string(this->dataPtr->maj) + "." +
    std::to_string(this->dataPtr->min) \
    + "." + std::to_string(this->dataPtr->patch);
}

/////////////////////////////////////////////////
bool SemanticVersion::operator<(const SemanticVersion &_other) const
{
  if (this == &_other)
    return false;

  if (this->dataPtr->maj < _other.dataPtr->maj)
  {
    return true;
  }
  if (this->dataPtr->maj > _other.dataPtr->maj)
  {
    return false;
  }
  if (this->dataPtr->min < _other.dataPtr->min)
  {
    return true;
  }
  if (this->dataPtr->min > _other.dataPtr->min)
  {
    return false;
  }
  if (this->dataPtr->patch < _other.dataPtr->patch)
  {
    return true;
  }
  if (this->dataPtr->patch > _other.dataPtr->patch)
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
  if (this == &_other)
    return true;

  return (_other.dataPtr->maj == this->dataPtr->maj)
    && (_other.dataPtr->min == this->dataPtr->min)
    && (_other.dataPtr->patch == this->dataPtr->patch);
}

/////////////////////////////////////////////////
bool SemanticVersion::operator!=(const SemanticVersion &_other) const
{
  return !(*this == _other);
}


