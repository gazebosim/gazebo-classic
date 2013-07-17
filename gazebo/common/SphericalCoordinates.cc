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
SphericalCoordinates::SurfaceType SphericalCoordinates::Convert(const std::string &_str)
{
  if ("EARTH_WGS84" == _str)
    return EARTH_WGS84;
  // else
  gzerr << "SurfaceModel string not recognized, "
        << "EARTH_WGS84 returned by default" << std::endl;
  return EARTH_WGS84;
}

//////////////////////////////////////////////////
SphericalCoordinates::SphericalCoordinates() :
      surfaceModel(EARTH_WGS84),
      elevationReference(0.0)
{
}

//////////////////////////////////////////////////
SphericalCoordinates::SphericalCoordinates(const SurfaceType _model) :
      surfaceModel(_model)
{
}

//////////////////////////////////////////////////
SphericalCoordinates::SphericalCoordinates(
    const SurfaceType _model, const math::Angle &_latitude,
    const math::Angle &_longitude, double _elevation,
    const math::Angle &_heading) :
      surfaceModel(_model), latitudeReference(_latitude),
      longitudeReference(_longitude), elevationReference(_elevation),
      headingOffset(_heading)
{
}

//////////////////////////////////////////////////
SphericalCoordinates::~SphericalCoordinates()
{
}

//////////////////////////////////////////////////
SphericalCoordinates::SurfaceType SphericalCoordinates::GetSurfaceModel() const
{
  return this->surfaceModel;
}

//////////////////////////////////////////////////
math::Angle SphericalCoordinates::GetLatitudeReference() const
{
  return this->latitudeReference;
}

//////////////////////////////////////////////////
math::Angle SphericalCoordinates::GetLongitudeReference() const
{
  return this->longitudeReference;
}

//////////////////////////////////////////////////
double SphericalCoordinates::GetElevationReference() const
{
  return this->elevationReference;
}

//////////////////////////////////////////////////
math::Angle SphericalCoordinates::GetHeadingOffset() const
{
  return this->headingOffset;
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetSurfaceModel(const SurfaceType &_model)
{
  this->surfaceModel = _model;
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetLatitudeReference(const math::Angle &_angle)
{
  this->latitudeReference.SetFromRadian(_angle.Radian());
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetLongitudeReference(const math::Angle &_angle)
{
  this->longitudeReference.SetFromRadian(_angle.Radian());
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetElevationReference(double _elevation)
{
  this->elevationReference = _elevation;
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetHeadingOffset(const math::Angle &_angle)
{
  this->headingOffset.SetFromRadian(_angle.Radian());
}

//////////////////////////////////////////////////
math::Vector3 SphericalCoordinates::Convert(const math::Vector3 &_xyz) const
{
  double radiusMeridional = 1.0;
  double radiusNormal = 1.0;
  double headingSine = sin(this->headingOffset.Radian());
  double headingCosine = cos(this->headingOffset.Radian());

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
        radiusMeridional = ab*ab / denom / sqrt(denom);
        radiusNormal = a*a / sqrt(denom);
      }
      break;

    default:
      break;
  }

  math::Vector3 spherical;
  double east  = _xyz.x * headingCosine - _xyz.y * headingSine;
  double north = _xyz.x * headingSine   + _xyz.y * headingCosine;
  // Assumes small changes in latitude / longitude.
  // May not work well near the north / south poles.
  math::Angle deltaLatitude(north / radiusMeridional);
  math::Angle deltaLongitude(east / radiusNormal);
  // geodetic latitude in degrees
  spherical.x = this->latitudeReference.Degree() + deltaLatitude.Degree();
  // geodetic longitude in degrees
  spherical.y = this->longitudeReference.Degree() + deltaLongitude.Degree();
  // altitude relative to sea level
  spherical.z = this->elevationReference + _xyz.z;
  return spherical;
}
