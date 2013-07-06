/*
 * Copyright 2013 Open Source Robotics Foundation
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
#include <math>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "SphericalCoordinates.hh"

using namespace gazebo;
using namespace physics;

// Parameters for EARTH_WGS84 model
// a: Semi-major equatorial axis (meters)
// b: Semi-minor polar axis (meters)
// if: inverse flattening (no units)
// wikipedia: World_Geodetic_System#A_new_World_Geodetic_System:_WGS_84
const double g_earth_wgs84_axis_a = 6378137.0;
const double g_earth_wgs84_axis_b = 6356752.314245;
const double g_earth_wgs84_axis_if = 298.257223563;

//////////////////////////////////////////////////
SphericalCoordinates::SphericalCoordinates()
{
}

//////////////////////////////////////////////////
SphericalCoordinates::~SphericalCoordinates()
{
}

//////////////////////////////////////////////////
void SphericalCoordinates::Load(sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_sdf != NULL, "_sdf parameter is NULL");

  this->sdf = _sdf;

  // Identify surface model type
  std::string surfaceModelStr = _sdf->GetValueString("surface_model");
  switch (surfaceModelStr)
  {
    case "EARTH_WGS84":
      this->surfaceModel = EARTH_WGS84;
      break;

    default:
      gzwarn << "surface_model parameter not recognized, "
             << "EARTH_WGS84 will be used by default\n";
      this->surfaceModel = EARTH_WGS84;
      break;
  }
  this->latitudeReferenceDegrees = _sdf->GetValueDouble("latitude_deg");
  this->longitudeReferenceDegrees = _sdf->GetValueDouble("longitude_deg");
  this->headingOffsetDegrees = _sdf->GetValueDouble("heading_deg");
}

//////////////////////////////////////////////////
void SphericalCoordinates::Init()
{
  switch (this->surfaceModel)
  {
    case: EARTH_WGS84:
      
      break;
  }
  //cos(this->latitudeReferenceDegrees*M_PI) * g_earth_wgs84_axis_a
  //sin(this->latitudeReferenceDegrees*M_PI) * g_earth_wgs84_axis_b
}

//////////////////////////////////////////////////
math::Vector3 SphericalCoordinates::Convert(const math::Vector3 &_xyz) const
{
  math::Vector3 spherical;
  return spherical;
}
