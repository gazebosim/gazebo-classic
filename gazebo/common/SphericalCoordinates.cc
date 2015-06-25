/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/common/SphericalCoordinates.hh"
#include "gazebo/common/SphericalCoordinatesPrivate.hh"


using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
SphericalCoordinates::SurfaceType SphericalCoordinates::Convert(
  const std::string &_str)
{
  if ("EARTH_WGS84" == _str)
    return EARTH_WGS84;

  gzerr << "SurfaceType string not recognized, "
        << "EARTH_WGS84 returned by default" << std::endl;
  return EARTH_WGS84;
}

//////////////////////////////////////////////////
SphericalCoordinates::SphericalCoordinates()
  : dataPtr(new SphericalCoordinatesPrivate)
{
  this->SetSurfaceType(EARTH_WGS84);
  this->SetElevationReference(0.0);
}

//////////////////////////////////////////////////
SphericalCoordinates::SphericalCoordinates(const SurfaceType _type)
  : dataPtr(new SphericalCoordinatesPrivate)
{
  this->SetSurfaceType(_type);
  this->SetElevationReference(0.0);
}

//////////////////////////////////////////////////
SphericalCoordinates::SphericalCoordinates(const SurfaceType _type,
                                           const math::Angle &_latitude,
                                           const math::Angle &_longitude,
                                           double _elevation,
                                           const math::Angle &_heading)
  : dataPtr(new SphericalCoordinatesPrivate)
{
  // Set the reference and calculate ellipse parameters
  this->SetSurfaceType(_type);

  // Set the coordinate transform parameters
  this->dataPtr->latitudeReference.SetFromRadian(_latitude.Radian());
  this->dataPtr->longitudeReference.SetFromRadian(_longitude.Radian());
  this->dataPtr->elevationReference = _elevation;
  this->dataPtr->headingOffset.SetFromRadian(_heading.Radian());

  // Generate transformation matrix
  this->UpdateTransformationMatrix();
}

//////////////////////////////////////////////////
SphericalCoordinates::~SphericalCoordinates()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
SphericalCoordinates::SurfaceType SphericalCoordinates::GetSurfaceType() const
{
  return this->dataPtr->surfaceType;
}

//////////////////////////////////////////////////
math::Angle SphericalCoordinates::GetLatitudeReference() const
{
  return this->dataPtr->latitudeReference;
}

//////////////////////////////////////////////////
math::Angle SphericalCoordinates::GetLongitudeReference() const
{
  return this->dataPtr->longitudeReference;
}

//////////////////////////////////////////////////
double SphericalCoordinates::GetElevationReference() const
{
  return this->dataPtr->elevationReference;
}

//////////////////////////////////////////////////
math::Angle SphericalCoordinates::GetHeadingOffset() const
{
  return this->dataPtr->headingOffset;
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetSurfaceType(const SurfaceType &_type)
{
  this->dataPtr->surfaceType = _type;
  switch (this->dataPtr->surfaceType)
  {
  case EARTH_WGS84: default:
    ell_a = 6378137.0;
    ell_b = 6356752.31424518;
    ell_f = (ell_a - ell_b) / ell_a;
    ell_e = sqrt((ell_a*ell_a-ell_b*ell_b)/(ell_a*ell_a));
    ell_p = sqrt((ell_a*ell_a-ell_b*ell_b)/(ell_b*ell_b));
    break;
  }
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetLatitudeReference(const math::Angle &_angle)
{
  this->dataPtr->latitudeReference.SetFromRadian(_angle.Radian());
  this->UpdateTransformationMatrix();
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetLongitudeReference(const math::Angle &_angle)
{
  this->dataPtr->longitudeReference.SetFromRadian(_angle.Radian());
  this->UpdateTransformationMatrix();
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetElevationReference(double _elevation)
{
  this->dataPtr->elevationReference = _elevation;
  this->UpdateTransformationMatrix();
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetHeadingOffset(const math::Angle &_angle)
{
  this->dataPtr->headingOffset.SetFromRadian(_angle.Radian());
  this->UpdateTransformationMatrix();
}

//////////////////////////////////////////////////
void SphericalCoordinates::UpdateTransformationMatrix()
{
  // Cache trig results
  double cosLat = cos(this->dataPtr->latitudeReference.Radian());
  double sinLat = sin(this->dataPtr->latitudeReference.Radian());
  double cosLon = cos(this->dataPtr->longitudeReference.Radian());
  double sinLon = sin(this->dataPtr->longitudeReference.Radian());

  // Create a rotation matrix that moves ECEF to GLOBAL
  this->Recef2enu = math::Matrix3(
                      -sinLon,           cosLon,          0.0,
                      -cosLon * sinLat, -sinLon * sinLat, cosLat,
                       cosLon * cosLat,  sinLon * cosLat, sinLat
                    );

  // Cache heading transforms -- note that we have to negate the heading in 
  // order to preserve backward compatibility. ie. Gazebo has traditionally 
  // expressed positive angle as a COCKWISE rotation that takes the GLOBAL frame
  // to the LOCAL frame. However, right hand coordinate systems require this
  // to be expressed as an ANTI-CLOCKWISE roation. So, we negate the rotation.
  this->cosHea = cos(-this->dataPtr->headingOffset.Radian());
  this->sinHea = sin(-this->dataPtr->headingOffset.Radian());

  // Cache the ECEF coordinate of the origin
  this->origin = math::Vector3(
    this->dataPtr->latitudeReference.Radian(),
    this->dataPtr->longitudeReference.Radian(),
    this->dataPtr->elevationReference);
  this->origin = this->PositionTransform(this->origin,SPHERICAL,ECEF);
}

//////////////////////////////////////////////////
math::Vector3 SphericalCoordinates::PositionTransform(const math::Vector3 &_pos,
  const CoordinateType &_in, const CoordinateType &_out) const
{
  // Output value
  math::Vector3 tmp = _pos;  // Cache input value locally

  // Cache trig results
  double cosLat = cos(_pos.x);
  double sinLat = sin(_pos.x);
  double cosLon = cos(_pos.y);
  double sinLon = sin(_pos.y);

  // Radius of planet curvature (meters) 
  double N = 1.0-this->ell_e*this->ell_e*sinLat*sinLat;
  if (N < 0)
    N = this->ell_a;
  else
    N = this->ell_a/sqrt(N);

  // Convert whatever arrives to a more flexible ECEF coordinate
  switch (_in)
  {
  case LOCAL: // -> ENU (note no break at end of case)
    tmp.x = -_pos.x * cosHea + _pos.y * sinHea;
    tmp.y = -_pos.x * sinHea - _pos.y * cosHea;
  case GLOBAL: // -> spherical
    tmp = this->origin  + this->Recef2enu.Inverse() * tmp;
    break;
  case SPHERICAL: // -> ECEF
    tmp.x = (_pos.z + N) * cosLat * cosLon;
    tmp.y = (_pos.z + N) * cosLat * sinLon;
    tmp.z = ((this->ell_b*this->ell_b)/(this->ell_a*this->ell_a)*N+_pos.z) 
             * sinLat;
    break;
  case ECEF: default:  // Do nothing
    tmp = _pos; 
    break;
  }

  // CASE 1 : Return ECEF
  if (_out == ECEF) 
    return tmp;

  // CASE 2 : Return SPHERICAL
  if (_out == SPHERICAL)
  {
    // Convert from ECEF to SPHERICAL
    double p = sqrt(tmp.x*tmp.x + tmp.y*tmp.y);
    double T = atan((tmp.z*this->ell_a)/(p*this->ell_b));
    double sinT = sin(T);
    double cosT = cos(T);

    // Calculate latitude and longitude
    double lat = atan((tmp.z + this->ell_p*this->ell_p*this->ell_b*sinT*sinT*sinT)/
      (p - this->ell_e*this->ell_e*this->ell_a*cosT*cosT*cosT));
    double lon = atan2(tmp.y,tmp.x);

    // Recalculate radius of planet curvature
    double n_sinLat = sin(lat);
    double n_cosLat = cos(lat);
    double n_N = 1.0-this->ell_e*this->ell_e*n_sinLat*n_sinLat;
    if (n_N < 0)
      n_N = this->ell_a;
    else
      n_N = this->ell_a/sqrt(n_N);  

    // Now calculate Z
    tmp.x = lat;
    tmp.y = lon;
    tmp.z = p/n_cosLat - n_N;
    return tmp;
  }

  // Convert from ECEF TO GLOBAL
  tmp = this->Recef2enu * (tmp - this->origin);

  // CASE 2 : Return GLOBAL
  if (_out == GLOBAL)
    return tmp;

  // CASE 4 : Convert from GLOBAL to LOCAL
  return math::Vector3(
    tmp.x * cosHea - tmp.y * sinHea,
    tmp.x * sinHea + tmp.y * cosHea,
    tmp.z
  );
}

//////////////////////////////////////////////////
math::Vector3 SphericalCoordinates::VelocityTransform(const math::Vector3 &_vel,
  const CoordinateType &_in, const CoordinateType &_out) const
{
  // Sanity check -- velocity should not be expressed in spherical coordinates
  if (_in == SPHERICAL || _out == SPHERICAL)
  {
    gzwarn << "Spherical velocities are not supported";
    return _vel;
  }

  // Intermediate data type
  math::Vector3 tmp = _vel;

  // Convert whatever arrives to an ECEF vector
  switch (_in)
  {
  case LOCAL: // -> ENU (note no break at end of case)
    tmp.x = -_vel.x * cosHea + _vel.y * sinHea;
    tmp.y = -_vel.x * sinHea - _vel.y * cosHea;
  case GLOBAL: // -> spherical
    tmp = this->Recef2enu.Inverse() * tmp;
    break;
  case ECEF: default:  // Do nothing
    tmp = _vel; 
    break;
  }

  // CASE 1: Return the ECEF vector
  if (_out == ECEF)
    return tmp;

  // CASE 2: Convert from ECEF to GLOBAL
  tmp = this->Recef2enu * tmp;
  if (_out == GLOBAL)
    return tmp;

  // CASE 3 : Convert from GLOBAL TO LOCAL
  return math::Vector3(
    tmp.x * cosHea - tmp.y * sinHea,
    tmp.x * sinHea + tmp.y * cosHea,
    tmp.z
  );
}

//////////////////////////////////////////////////
math::Vector3 SphericalCoordinates::SphericalFromLocal(
    const math::Vector3 &_xyz) const
{
  return this->PositionTransform(_xyz,LOCAL,SPHERICAL);
}

//////////////////////////////////////////////////
math::Vector3 SphericalCoordinates::LocalFromSpherical(
    const math::Vector3 &_xyz) const
{
  return this->PositionTransform(_xyz,SPHERICAL,LOCAL);
}

//////////////////////////////////////////////////
math::Vector3 SphericalCoordinates::GlobalFromLocal(const math::Vector3 &_xyz)
    const
{
  return this->VelocityTransform(_xyz,LOCAL,GLOBAL);
}

//////////////////////////////////////////////////
math::Vector3 SphericalCoordinates::LocalFromGlobal(const math::Vector3 &_xyz)
    const
{
  return this->VelocityTransform(_xyz,GLOBAL,LOCAL);
}

//////////////////////////////////////////////////
/// Based on Haversine formula (http://en.wikipedia.org/wiki/Haversine_formula).
double SphericalCoordinates::Distance(const math::Angle &_latA,
                                      const math::Angle &_lonA,
                                      const math::Angle &_latB,
                                      const math::Angle &_lonB)
{
  math::Angle dLat = _latB - _latA;
  math::Angle dLon = _lonB - _lonA;
  double a = sin(dLat.Radian() / 2) * sin(dLat.Radian() / 2) +
             sin(dLon.Radian() / 2) * sin(dLon.Radian() / 2) *
             cos(_latA.Radian()) * cos(_latB.Radian());
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return 6371000.0 * c;
}
