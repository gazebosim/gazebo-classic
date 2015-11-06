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

#include <sstream>

#include "gazebo/math/Helpers.hh"
#include "gazebo/gui/model/DensityModel.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
DensityEntry::DensityEntry(const std::string &_desc, double _value)
{
  this->desc = _desc;
  this->value = _value;
}

/////////////////////////////////////////////////
DensityEntry::~DensityEntry()
{
}

/////////////////////////////////////////////////
std::string DensityEntry::ToString() const
{
  std::stringstream stream;

  stream << "[Density: "
         << this->desc
         << " "
         << this->value
         << " kg/m^3]";

  return stream.str();
}

/////////////////////////////////////////////////
DensityModel::DensityModel()
{
  // Source: https://en.wikipedia.org/wiki/Density

  this->entries.push_back(new DensityEntry("Aluminum", 2700.0));
  this->entries.push_back(new DensityEntry("Brass", 8600.0));
  this->entries.push_back(new DensityEntry("Copper", 8940.0));
  this->entries.push_back(new DensityEntry("Concrete", 2000.0));
  this->entries.push_back(new DensityEntry("Ice", 916.0));
  this->entries.push_back(new DensityEntry("Iron", 7870.0));
  this->entries.push_back(new DensityEntry("Oak", 710.0));
  this->entries.push_back(new DensityEntry("Pine", 373.0));
  this->entries.push_back(new DensityEntry("Plastic", 1175.0));
  this->entries.push_back(new DensityEntry("Steel, Alloy", 7600.0));
  this->entries.push_back(new DensityEntry("Steel, Stainless", 7800.0));
  this->entries.push_back(new DensityEntry("Styrofoam", 75.0));
  this->entries.push_back(new DensityEntry("Tungsten", 19300.0));
  this->entries.push_back(new DensityEntry("Water", 1000.0));
  this->entries.push_back(new DensityEntry("Wood", 700.0));
}

/////////////////////////////////////////////////
DensityModel::~DensityModel()
{
  for (auto it : this->entries)
    delete it;
  this->entries.clear();
}

/////////////////////////////////////////////////
std::vector<const DensityEntry *> DensityModel::Entries() const
{
  return this->entries;
}

/////////////////////////////////////////////////
const DensityEntry *DensityModel::EntryByDesc(const std::string &_desc) const
{
  for (auto it : this->entries)
  {
    if (it->desc == _desc)
      return it;
  }
  return NULL;
}

/////////////////////////////////////////////////
const DensityEntry *DensityModel::EntryByValue(double _value) const
{
  for (auto it : this->entries)
  {
	if (math::equal(it->value, _value))
		return it;
  }
  return NULL;
}
