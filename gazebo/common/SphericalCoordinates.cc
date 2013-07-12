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
#include <math.h>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "SphericalCoordinates.hh"

using namespace gazebo;
using namespace common;

// Parameters for EARTH_WGS84 model
// a: Semi-major equatorial axis (meters)
// b: Semi-minor polar axis (meters)
// if: inverse flattening (no units)
// wikipedia: World_Geodetic_System#A_new_World_Geodetic_System:_WGS_84
const double g_earth_wgs84_axis_equatorial = 6378137.0;
const double g_earth_wgs84_axis_polar = 6356752.314245;
const double g_earth_wgs84_flattening = 1/298.257223563;

//////////////////////////////////////////////////
SphericalCoordinates::SphericalCoordinates() :
      surfaceModel(EARTH_WGS84),
      headingCosine(1.0),
      headingSine(0.0),
      radiusMeridional(1.0),
      radiusNormal(1.0)
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

  if ("EARTH_WGS84" == surfaceModelStr)
    this->surfaceModel = EARTH_WGS84;
  else
  {
    gzwarn << "surface_model parameter not recognized, "
           << "EARTH_WGS84 will be used by default\n";
    this->surfaceModel = EARTH_WGS84;
  }

  this->latitudeReference.SetFromDegree(_sdf->GetValueDouble("latitude_deg"));
  this->longitudeReference.SetFromDegree(_sdf->GetValueDouble("longitude_deg"));
  this->headingOffset.SetFromDegree(_sdf->GetValueDouble("heading_deg"));
  this->headingCosine = cos(this->headingOffset.Radian());
  this->headingSine = sin(this->headingOffset.Radian());
}

//////////////////////////////////////////////////
void SphericalCoordinates::Init()
{
  switch (this->surfaceModel)
  {
    case EARTH_WGS84:
      // Currently uses radius of curvature equations from wikipedia
      // http://en.wikipedia.org/wiki/Earth_radius#Radius_of_curvature
      {
        double a = g_earth_wgs84_axis_equatorial;
        double b = g_earth_wgs84_axis_polar;
        double ab = a*b;
        double cosLat = cos(this->latitudeReference.Radian());
        double sinLat = sin(this->latitudeReference.Radian());
        double denom = (a*cosLat)*(a*cosLat) + (b*sinLat)*(b*sinLat);
        this->radiusMeridional = ab*ab / denom / sqrt(denom);
        this->radiusNormal = a*a / sqrt(denom);
      }
      break;

    default:
      break;
  }
}

//////////////////////////////////////////////////
math::Vector3 SphericalCoordinates::Convert(const math::Vector3 &_xyz) const
{
  math::Vector3 spherical;
  double east  = _xyz.x * this->headingCosine - _xyz.y * this->headingSine;
  double north = _xyz.x * this->headingSine   + _xyz.y * this->headingCosine;
  // Assumes small changes in latitude / longitude.
  // May not work well near the north / south poles.
  math::Angle deltaLatitude(north / this->radiusMeridional);
  math::Angle deltaLongitude(east / this->radiusNormal);
  // geodetic latitude in degrees
  spherical.x = this->latitudeReference.Degree() + deltaLatitude.Degree();
  // geodetic longitude in degrees
  spherical.y = this->longitudeReference.Degree() + deltaLongitude.Degree();
  // geodetic altitude
  spherical.z = this->radiusNormal + _xyz.z;
  return spherical;
}
