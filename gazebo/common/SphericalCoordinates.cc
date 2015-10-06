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

// Parameters for EARTH_WGS84 model
// a: Semi-major equatorial axis (meters)
// b: Semi-minor polar axis (meters)
// if: inverse flattening (no units)
// wikipedia: World_Geodetic_System#A_new_World_Geodetic_System:_WGS_84
const double g_EarthWGS84AxisEquatorial = 6378137.0;
const double g_EarthWGS84AxisPolar = 6356752.314245;

const double g_EarthSphere = 6371000.0;
// const double g_EarthWGS84Flattening = 1/298.257223563;

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
  this->dataPtr->latitudeReference = _latitude.Radian();
  this->dataPtr->longitudeReference = _longitude.Radian();
  this->dataPtr->elevationReference = _elevation;
  this->dataPtr->headingOffset = _heading.Radian();

  // Generate transformation matrix
  this->UpdateTransformationMatrix();
}

//////////////////////////////////////////////////
SphericalCoordinates::SphericalCoordinates(const SurfaceType _type,
    const ignition::math::Angle &_latitude,
    const ignition::math::Angle &_longitude,
    double _elevation,
    const ignition::math::Angle &_heading)
: dataPtr(new SphericalCoordinatesPrivate)
{
  // Set the reference and calculate ellipse parameters
  this->SetSurfaceType(_type);

  // Set the coordinate transform parameters
  this->dataPtr->latitudeReference = _latitude;
  this->dataPtr->longitudeReference = _longitude;
  this->dataPtr->elevationReference = _elevation;
  this->dataPtr->headingOffset = _heading;

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
  return this->LatitudeReference();
}

//////////////////////////////////////////////////
ignition::math::Angle SphericalCoordinates::LatitudeReference() const
{
  return this->dataPtr->latitudeReference;
}

//////////////////////////////////////////////////
math::Angle SphericalCoordinates::GetLongitudeReference() const
{
  return this->LongitudeReference();
}

//////////////////////////////////////////////////
ignition::math::Angle SphericalCoordinates::LongitudeReference() const
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
  return this->HeadingOffset();
}

//////////////////////////////////////////////////
ignition::math::Angle SphericalCoordinates::HeadingOffset() const
{
  return this->dataPtr->headingOffset;
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetSurfaceType(const SurfaceType &_type)
{
  this->dataPtr->surfaceType = _type;

  switch (this->dataPtr->surfaceType)
  {
    case EARTH_WGS84:
      this->dataPtr->ell_a = g_EarthWGS84AxisEquatorial;
      this->dataPtr->ell_b = g_EarthWGS84AxisPolar;
      this->dataPtr->ell_f = (this->dataPtr->ell_a - this->dataPtr->ell_b) /
        this->dataPtr->ell_a;

      this->dataPtr->ell_e = sqrt(
          (this->dataPtr->ell_a * this->dataPtr->ell_a - this->dataPtr->ell_b *
           this->dataPtr->ell_b) /
          (this->dataPtr->ell_a * this->dataPtr->ell_a));

      this->dataPtr->ell_p = sqrt(
          (this->dataPtr->ell_a * this->dataPtr->ell_a - this->dataPtr->ell_b *
           this->dataPtr->ell_b) /
          (this->dataPtr->ell_b * this->dataPtr->ell_b));

      break;
    default:
      gzerr << "Unknown surface type[" << this->dataPtr->surfaceType << "]\n";
      break;
  }
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetLatitudeReference(
    const math::Angle &_angle)
{
  this->SetLatitudeReference(ignition::math::Angle(_angle.Radian()));
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetLatitudeReference(
    const ignition::math::Angle &_angle)
{
  this->dataPtr->latitudeReference = _angle;
  this->UpdateTransformationMatrix();
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetLongitudeReference(
    const math::Angle &_angle)
{
  this->SetLongitudeReference(ignition::math::Angle(_angle.Radian()));
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetLongitudeReference(
    const ignition::math::Angle &_angle)
{
  this->dataPtr->longitudeReference = _angle;
  this->UpdateTransformationMatrix();
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetElevationReference(double _elevation)
{
  this->dataPtr->elevationReference = _elevation;
  this->UpdateTransformationMatrix();
}

//////////////////////////////////////////////////
void SphericalCoordinates::SetHeadingOffset(const ignition::math::Angle &_angle)
{
  this->dataPtr->headingOffset.Radian(_angle.Radian());
  this->UpdateTransformationMatrix();
}

//////////////////////////////////////////////////
math::Vector3 SphericalCoordinates::SphericalFromLocal(
    const math::Vector3 &_xyz) const
{
  return this->SphericalFromLocal(_xyz.Ign());
}

//////////////////////////////////////////////////
math::Vector3 SphericalCoordinates::GlobalFromLocal(
    const math::Vector3 &_xyz) const
{
  return this->GlobalFromLocal(_xyz.Ign());
}

//////////////////////////////////////////////////
ignition::math::Vector3d SphericalCoordinates::SphericalFromLocal(
    const ignition::math::Vector3d &_xyz) const
{
  ignition::math::Vector3d result =
    this->PositionTransform(_xyz, LOCAL, SPHERICAL);
  result.X(IGN_RTOD(result.X()));
  result.Y(IGN_RTOD(result.Y()));
  return result;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SphericalCoordinates::LocalFromSpherical(
    const ignition::math::Vector3d &_xyz) const
{
  ignition::math::Vector3d result = _xyz;
  result.X(IGN_DTOR(result.X()));
  result.Y(IGN_DTOR(result.Y()));
  return this->PositionTransform(result, SPHERICAL, LOCAL);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SphericalCoordinates::GlobalFromLocal(
    const ignition::math::Vector3d &_xyz) const
{
  return this->VelocityTransform(_xyz, LOCAL, GLOBAL);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SphericalCoordinates::LocalFromGlobal(
    const ignition::math::Vector3d &_xyz) const
{
  return this->VelocityTransform(_xyz, GLOBAL, LOCAL);
}

//////////////////////////////////////////////////
/// Based on Haversine formula (http://en.wikipedia.org/wiki/Haversine_formula).
double SphericalCoordinates::Distance(const math::Angle &_latA,
                                      const math::Angle &_lonA,
                                      const math::Angle &_latB,
                                      const math::Angle &_lonB)
{
  return Distance(ignition::math::Angle(_latA.Radian()),
                  ignition::math::Angle(_lonA.Radian()),
                  ignition::math::Angle(_latB.Radian()),
                  ignition::math::Angle(_lonB.Radian()));
}

//////////////////////////////////////////////////
/// Based on Haversine formula (http://en.wikipedia.org/wiki/Haversine_formula).
double SphericalCoordinates::Distance(const ignition::math::Angle &_latA,
                                      const ignition::math::Angle &_lonA,
                                      const ignition::math::Angle &_latB,
                                      const ignition::math::Angle &_lonB)
{
  ignition::math::Angle dLat = _latB - _latA;
  ignition::math::Angle dLon = _lonB - _lonA;

  double a = sin(dLat.Radian() / 2) * sin(dLat.Radian() / 2) +
             sin(dLon.Radian() / 2) * sin(dLon.Radian() / 2) *
             cos(_latA.Radian()) * cos(_latB.Radian());

  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double d = g_EarthSphere * c;
  return d;
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
  this->dataPtr->rotECEF2ENU = ignition::math::Matrix3d(
                      -sinLon,           cosLon,          0.0,
                      -cosLon * sinLat, -sinLon * sinLat, cosLat,
                       cosLon * cosLat,  sinLon * cosLat, sinLat);

  // Cache heading transforms -- note that we have to negate the heading in
  // order to preserve backward compatibility. ie. Gazebo has traditionally
  // expressed positive angle as a CLOCKWISE rotation that takes the GLOBAL
  // frame to the LOCAL frame. However, right hand coordinate systems require
  // this to be expressed as an ANTI-CLOCKWISE rotation. So, we negate it.
  this->dataPtr->cosHea = cos(-this->dataPtr->headingOffset.Radian());
  this->dataPtr->sinHea = sin(-this->dataPtr->headingOffset.Radian());

  // Cache the ECEF coordinate of the origin
  this->dataPtr->origin = ignition::math::Vector3d(
    this->dataPtr->latitudeReference.Radian(),
    this->dataPtr->longitudeReference.Radian(),
    this->dataPtr->elevationReference);
  std::cout << "Origin1[" << this->dataPtr->origin << "]\n";
  this->dataPtr->origin =
    this->PositionTransform(this->dataPtr->origin, SPHERICAL, ECEF);
  std::cout << "Origin2[" << this->dataPtr->origin << "]\n";
}

/////////////////////////////////////////////////
ignition::math::Vector3d SphericalCoordinates::PositionTransform(
    const ignition::math::Vector3d &_pos,
    const CoordinateType &_in, const CoordinateType &_out) const
{
  ignition::math::Vector3d tmp = _pos;

  // Cache trig results
  double cosLat = cos(_pos.X());
  double sinLat = sin(_pos.X());
  double cosLon = cos(_pos.Y());
  double sinLon = sin(_pos.Y());

  // Radius of planet curvature (meters)
  double curvature = 1.0 - this->dataPtr->ell_e * this->dataPtr->ell_e *
    sinLat * sinLat;
  if (curvature < 0)
    curvature = this->dataPtr->ell_a;
  else
    curvature = this->dataPtr->ell_a / sqrt(curvature);

  // Convert whatever arrives to a more flexible ECEF coordinate
  switch (_in)
  {
    // ENU (note no break at end of case)
    case LOCAL:
      tmp.X(-_pos.X() * this->dataPtr->cosHea + _pos.Y() *
            this->dataPtr->sinHea);
      tmp.Y(-_pos.X() * this->dataPtr->sinHea - _pos.Y() *
            this->dataPtr->cosHea);

    // spherical
    case GLOBAL:
      tmp = this->dataPtr->origin + this->dataPtr->rotECEF2ENU.Inverse() * tmp;
      break;

    // ECEF
    case SPHERICAL:
      tmp.X((_pos.Z() + curvature) * cosLat * cosLon);
      tmp.Y((_pos.Z() + curvature) * cosLat * sinLon);
      tmp.Z(((this->dataPtr->ell_b * this->dataPtr->ell_b)/
            (this->dataPtr->ell_a * this->dataPtr->ell_a) *
            curvature + _pos.Z()) * sinLat);
      break;
    // Do nothing
    case ECEF:
      break;
    default:
      gzerr << "Invalid coordinate type[" << _in << "]\n";
      return _pos;
  }

  // Convert ECEF to the requested output coordinate system
  switch (_out)
  {
    case SPHERICAL:
      {
        // Convert from ECEF to SPHERICAL
        double p = sqrt(tmp.X() * tmp.X() + tmp.Y() * tmp.Y());
        double t = atan((tmp.Z() * this->dataPtr->ell_a) /
            (p * this->dataPtr->ell_b));
        double sinT = sin(t);
        double cosT = cos(t);

        // Calculate latitude and longitude
        double lat = atan((tmp.Z() + this->dataPtr->ell_p *
              this->dataPtr->ell_p * this->dataPtr->ell_b * sinT
              * sinT * sinT) /
            (p - this->dataPtr->ell_e * this->dataPtr->ell_e *
             this->dataPtr->ell_a * cosT * cosT * cosT));

        double lon = atan2(tmp.Y(), tmp.X());

        // Recalculate radius of planet curvature
        double nSinLat = sin(lat);
        double nCosLat = cos(lat);
        double nCurvature = 1.0 - this->dataPtr->ell_e *
          this->dataPtr->ell_e * nSinLat * nSinLat;

        if (nCurvature < 0)
          nCurvature = this->dataPtr->ell_a;
        else
          nCurvature = this->dataPtr->ell_a / sqrt(nCurvature);

        // Now calculate Z
        tmp.X(lat);
        tmp.Y(lon);
        tmp.Z(p/nCosLat - nCurvature);

        // altitude relative to sea level
        //tmp.Z(this->dataPtr->elevationReference + _pos.Z());
        break;
      }

    // Convert from ECEF TO GLOBAL
    case GLOBAL:
      tmp = this->dataPtr->rotECEF2ENU * (tmp - this->dataPtr->origin);
      break;

    // Convert from ECEF TO LOCAL
    case LOCAL:
      std::cout << "Tmp1 [" << tmp << "]\n";
      std::cout << "Origin[" << this->dataPtr->origin << "]\n";
      std::cout << "Minus[" << tmp - this->dataPtr->origin << "]\n";
      std::cout << "ROT [" << this->dataPtr->rotECEF2ENU << "]\n";

      tmp = this->dataPtr->rotECEF2ENU * (tmp - this->dataPtr->origin);
      std::cout << "ElevationRef[" << this->dataPtr->elevationReference
        << "] tmp.Z()[" << tmp.Z() << "]\n";

      tmp = ignition::math::Vector3d(
          tmp.X() * this->dataPtr->cosHea - tmp.Y() * this->dataPtr->sinHea,
          tmp.X() * this->dataPtr->sinHea + tmp.Y() * this->dataPtr->cosHea,
          tmp.Z());
      break;

    // Return ECEF (do nothing)
    case ECEF:
      break;

    default:
      gzerr << "Unknown coordinate type[" << _out << "]\n";
      return _pos;
  }

  return tmp;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SphericalCoordinates::VelocityTransform(
    const ignition::math::Vector3d &_vel,
    const CoordinateType &_in, const CoordinateType &_out) const
{
  // Sanity check -- velocity should not be expressed in spherical coordinates
  if (_in == SPHERICAL || _out == SPHERICAL)
  {
    gzwarn << "Spherical velocities are not supported";
    return _vel;
  }

  // Intermediate data type
  ignition::math::Vector3d tmp = _vel;

  // First, convert to an ECEF vector
  switch (_in)
  {
    // ENU (note no break at end of case)
    case LOCAL:
      tmp.X(-_vel.X() * this->dataPtr->cosHea + _vel.Y() *
            this->dataPtr->sinHea);
      tmp.Y(-_vel.X() * this->dataPtr->sinHea - _vel.Y() *
            this->dataPtr->cosHea);
    // spherical
    case GLOBAL:
      tmp = this->dataPtr->rotECEF2ENU.Inverse() * tmp;
      break;
    // Do nothing
    case ECEF:
      tmp = _vel;
      break;
    default:
      gzerr << "Unknown coordinate type[" << _in << "]\n";
      return _vel;
  }

  // Then, convert to the request coordinate type
  switch (_out)
  {
    // ECEF, do nothing
    case ECEF:
      break;

    // Convert from ECEF to global
    case GLOBAL:
      tmp = this->dataPtr->rotECEF2ENU * tmp;
      break;

    // Convert from ECEF to local
    case LOCAL:
      tmp = this->dataPtr->rotECEF2ENU * tmp;
      tmp = ignition::math::Vector3d(
          tmp.X() * this->dataPtr->cosHea - tmp.Y() * this->dataPtr->sinHea,
          tmp.X() * this->dataPtr->sinHea + tmp.Y() * this->dataPtr->cosHea,
          tmp.Z());
      break;

    default:
      gzerr << "Unknown coordinate type[" << _out << "]\n";
      return _vel;
  }

  return tmp;
}
