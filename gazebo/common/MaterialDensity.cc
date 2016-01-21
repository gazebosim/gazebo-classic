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
#include <ignition/math/Helpers.hh>
#include "gazebo/common/EnumIface.hh"
#include "gazebo/common/MaterialDensity.hh"

using namespace gazebo;
using namespace common;

// Initialize enum iterator, and string converter
GZ_ENUM(MaterialType,
  MaterialType::BEGIN,
  MaterialType::END,
  "Styrofoam",
  "Pine",
  "Wood",
  "Oak",
  "Ice",
  "Water",
  "Plastic",
  "Concrete",
  "Aluminum",
  "Steel, Alloy",
  "Steel, Stainless",
  "Iron",
  "Brass",
  "Copper",
  "Tungsten",
  "end"
)

// Initialize the materials
std::map<MaterialType, double> MaterialDensity::materials =
{
  {MaterialType::STYROFOAM, 75.0},
  {MaterialType::PINE, 373.0},
  {MaterialType::WOOD, 700.0},
  {MaterialType::OAK, 710.0},
  {MaterialType::ICE, 916.0},
  {MaterialType::WATER, 1000.0},
  {MaterialType::PLASTIC, 1175.0},
  {MaterialType::CONCRETE, 2000.0},
  {MaterialType::ALUMINUM, 2700.0},
  {MaterialType::STEEL_ALLOY, 7600.0},
  {MaterialType::STEEL_STAINLESS, 7800.0},
  {MaterialType::IRON, 7870.0},
  {MaterialType::BRASS, 8600.0},
  {MaterialType::COPPER, 8940.0},
  {MaterialType::TUNGSTEN, 19300.0}
};

/////////////////////////////////////////////////
const std::map<MaterialType, double> &MaterialDensity::Materials()
{
  return materials;
}

/////////////////////////////////////////////////
double MaterialDensity::Density(const std::string &_material)
{
  MaterialType type = MaterialType::END;
  EnumIface<MaterialType>::Set(type, _material);

  if (type != MaterialType::END)
    return materials[type];
  else
    return -1;
}

/////////////////////////////////////////////////
double MaterialDensity::Density(const MaterialType _material)
{
  return materials[_material];
}

/////////////////////////////////////////////////
std::tuple<MaterialType, double> MaterialDensity::Nearest(
    const double _value, const double _epsilon)
{
  double min = IGN_DBL_MAX;
  std::tuple<MaterialType, double> result
  {
    MaterialType::END, -1.0
  };

  for (auto const &mat : materials)
  {
    double diff = std::fabs(mat.second - _value);
    if (diff < min && diff < _epsilon)
    {
      min = diff;
      result = mat;
    }
  }

  return result;
}

/////////////////////////////////////////////////
MaterialType MaterialDensity::NearestMaterial(const double _value,
    const double _epsilon)
{
  return std::get<0>(Nearest(_value, _epsilon));
}
