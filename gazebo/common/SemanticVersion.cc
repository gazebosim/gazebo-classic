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
{
  this->Parse(_versionStr);
}

/////////////////////////////////////////////////
SemanticVersion::SemanticVersion(const unsigned int _maj,
  const unsigned int _min,
  const unsigned int _patch,
  const std::string &_prerelease,
  const std::string &_build)
: maj(_maj), min(_min), patch(_patch), prerelease(_prerelease),
  build(_build)
{
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

  // If this version has prelrease and the _other doesn't, then this
  // version is lower. We don't compare the prerelease strings because
  // they can contain any alphanumeric values.
  if (!this->prerelease.empty() && _other.prerelease.empty())
    return true;

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

  return (_other.maj == this->maj) && (_other.min == this->min)
    && (_other.patch == this->patch);
}

/////////////////////////////////////////////////
bool SemanticVersion::operator!=(const SemanticVersion &_other) const
{
  return !(*this == _other);
}

/////////////////////////////////////////////////
unsigned int SemanticVersion::Major() const
{
  return this->maj;
}

/////////////////////////////////////////////////
unsigned int SemanticVersion::Minor() const
{
  return this->min;
}

/////////////////////////////////////////////////
unsigned int SemanticVersion::Patch() const
{
  return this->patch;
}

/////////////////////////////////////////////////
std::string SemanticVersion::Prerelease() const
{
  return this->prerelease;
}

/////////////////////////////////////////////////
std::string SemanticVersion::Build() const
{
  return this->build;
}

/////////////////////////////////////////////////
bool SemanticVersion::Parse(const std::string &_versionStr)
{
  if (_versionStr.empty())
    return false;

  size_t numericEnd = _versionStr.size();
  size_t prereleaseStart = _versionStr.find("-");
  size_t buildStart = _versionStr.find("+");

  // Build meta data, if present, must be after prerelease string, if
  // present
  if (buildStart != std::string::npos &&
      prereleaseStart != std::string::npos &&
      buildStart < prereleaseStart)
  {
    return false;
  }

  // Check if a prerelease version is present
  if (prereleaseStart != std::string::npos)
  {
    // Set the end of the numeric (major.minor.patch) portion to the
    // start of the prerelease
    numericEnd = prereleaseStart;

    // Check if build metadata is present
    if (buildStart != std::string::npos)
    {
      // Set prerelease to the portion between "-" and "+"
      this->prerelease = _versionStr.substr(numericEnd + 1,
           buildStart - numericEnd - 1);

      // Get the build metadata.
      this->build = _versionStr.substr(buildStart + 1);
    }
    else
    {
      // Set prerelease to the portion between "-" and the string's end
      this->prerelease = _versionStr.substr(numericEnd + 1);
    }
  }
  // Check if build metadata is present
  else if (buildStart != std::string::npos)
  {
    // Set the end of the numeric (major.minor.patch) portion to the
    // start of the build metadata
    numericEnd = buildStart;

    // Pre-release info can't follow the build metadata.
    this->build = _versionStr.substr(numericEnd + 1);
  }

  std::string numeric = _versionStr.substr(0, numericEnd);

  unsigned int points = std::count(numeric.begin(), numeric.end(), '.');
  if (points == 0)
  {
    sscanf(_versionStr.c_str(), "%5u", &this->maj);
  }

  if (points == 1)
  {
    sscanf(_versionStr.c_str(), "%5u.%5u", &this->maj, &this->min);
  }

  if (points >= 2)
  {
    sscanf(_versionStr.c_str(), "%5u.%5u.%5u", &this->maj, &this->min,
        &this->patch);
  }

  return true;
}
