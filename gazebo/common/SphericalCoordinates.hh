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
#ifndef _GAZEBO_SPHERICALCOORDINATES_HH_
#define _GAZEBO_SPHERICALCOORDINATES_HH_

#include <string>

#include <ignition/math/Angle.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/math/Angle.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Matrix3.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class SphericalCoordinatesPrivate;

    /// \addtogroup gazebo_common
    /// \{

    /// \class SphericalCoordinates SphericalCoordinates.hh commmon/common.hh
    /// \brief Convert spherical coordinates for planetary surfaces.
    class GZ_COMMON_VISIBLE SphericalCoordinates
    {
      /// \enum SurfaceType
      /// \brief Unique identifiers for planetary surface models.
      public: enum SurfaceType
              {
                /// \brief Model of reference ellipsoid for earth, based on
                /// WGS 84 standard. see wikipedia: World_Geodetic_System
                EARTH_WGS84 = 1
              };

      /// \enum CoordinateType
      /// \brief Unique identifiers for coordinate types.
      public: enum CoordinateType
              {
                /// \brief Latitude, Longitude and Altitude by SurfaceType
                SPHERICAL = 1,

                /// \brief Earth centered, earth fixed Cartesian
                ECEF = 2,

                /// \brief Local tangent plane (East, North, Up)
                GLOBAL = 3,

                /// \brief Heading-adjusted tangent plane (Z, Y, Z)
                LOCAL = 4
              };

      /// \brief Constructor.
      public: SphericalCoordinates();

      /// \brief Constructor with surface type input.
      /// \param[in] _type SurfaceType specification.
      public: SphericalCoordinates(const SurfaceType _type);

      /// \brief Constructor with surface type, angle, and elevation inputs.
      /// \param[in] _type SurfaceType specification.
      /// \param[in] _latitude Reference latitude.
      /// \param[in] _longitude Reference longitude.
      /// \param[in] _elevation Reference elevation.
      /// \param[in] _heading Heading offset.
      /// \deprecated See SphericalCoordinates constructor that accepts
      /// ignition::math::Angle objects.
      public: SphericalCoordinates(const SurfaceType _type,
                  const math::Angle &_latitude,
                  const math::Angle &_longitude,
                  double _elevation,
                  const math::Angle &_heading) GAZEBO_DEPRECATED(6.0);

      /// \brief Constructor with surface type, angle, and elevation inputs.
      /// \param[in] _type SurfaceType specification.
      /// \param[in] _latitude Reference latitude.
      /// \param[in] _longitude Reference longitude.
      /// \param[in] _elevation Reference elevation.
      /// \param[in] _heading Heading offset.
      public: SphericalCoordinates(const SurfaceType _type,
                                   const ignition::math::Angle &_latitude,
                                   const ignition::math::Angle &_longitude,
                                   double _elevation,
                                   const ignition::math::Angle &_heading);

      /// \brief Destructor.
      public: ~SphericalCoordinates();

      /// \brief Convert a Cartesian position vector to geodetic coordinates.
      /// \param[in] _xyz Cartesian position vector in gazebo's world frame.
      /// \return Cooordinates: geodetic latitude (deg), longitude (deg),
      ///         altitude above sea level (m).
      /// \deprecated See SphericalFromLocal function that use
      /// ignition::math::Vector3d objects.
      public: math::Vector3 SphericalFromLocal(const math::Vector3 &_xyz) const
              GAZEBO_DEPRECATED(6.0);

      /// \brief Convert a Cartesian velocity vector in the local gazebo frame
      ///        to a global Cartesian frame with components East, North, Up.
      /// \param[in] _xyz Cartesian vector in gazebo's world frame.
      /// \return Rotated vector with components (x,y,z): (East, North, Up).
      /// \deprecated See GlobalFromLocal function that use
      /// ignition::math::Vector3d objects.
      public: math::Vector3 GlobalFromLocal(const math::Vector3 &_xyz) const
              GAZEBO_DEPRECATED(6.0);

      /// \brief Get SurfaceType currently in use.
      /// \return Current SurfaceType value.
      public: SurfaceType GetSurfaceType() const;

      /// \brief Get reference geodetic latitude.
      /// \return Reference geodetic latitude.
      /// \deprecated See LatitudeReference function that returns an
      /// ignition::math::Angle object.
      public: math::Angle GetLatitudeReference() const GAZEBO_DEPRECATED(6.0);

      /// \brief Get reference geodetic latitude.
      /// \return Reference geodetic latitude.
      public: ignition::math::Angle LatitudeReference() const;

      /// \brief Get reference longitude.
      /// \return Reference longitude.
      /// \deprecated See LongitudeReference function that returns an
      /// ignition::math::Angle object.
      public: math::Angle GetLongitudeReference() const GAZEBO_DEPRECATED(6.0);

      /// \brief Get reference longitude.
      /// \return Reference longitude.
      public: ignition::math::Angle LongitudeReference() const;

      /// \brief Get reference elevation in meters.
      /// \return Reference elevation.
      public: double GetElevationReference() const;

      /// \brief Get heading offset for gazebo reference frame, expressed as
      ///        angle from East to gazebo x-axis, or equivalently
      ///        from North to gazebo y-axis.
      /// \return Heading offset of gazebo reference frame.
      /// \deprecated See HeadingOffset() function that returns an
      /// ignition::math::Angle object.
      public: math::Angle GetHeadingOffset() const GAZEBO_DEPRECATED(6.0);

      /// \brief Get heading offset for gazebo reference frame, expressed as
      ///        angle from East to gazebo x-axis, or equivalently
      ///        from North to gazebo y-axis.
      /// \return Heading offset of gazebo reference frame.
      public: ignition::math::Angle HeadingOffset() const;

      /// \brief Set SurfaceType for planetary surface model.
      /// \param[in] _type SurfaceType value.
      public: void SetSurfaceType(const SurfaceType &_type);

      /// \brief Set reference geodetic latitude.
      /// \param[in] _angle Reference geodetic latitude.
      /// \deprecated See SetLatitudeReference function that accepts an
      /// ignition::math::Angle object.
      public: void SetLatitudeReference(const math::Angle &_angle)
              GAZEBO_DEPRECATED(6.0);

      /// \brief Set reference geodetic latitude.
      /// \param[in] _angle Reference geodetic latitude.
      public: void SetLatitudeReference(const ignition::math::Angle &_angle);

      /// \brief Set reference longitude.
      /// \param[in] _angle Reference longitude.
      public: void SetLongitudeReference(const ignition::math::Angle &_angle);

      /// \brief Set reference longitude.
      /// \param[in] _angle Reference longitude.
      /// \deprecated See SetLongitudeReference function that accepts an
      /// ignition::math::Angle object.
      public: void SetLongitudeReference(const math::Angle &_angle)
              GAZEBO_DEPRECATED(6.0);

      /// \brief Set reference elevation above sea level in meters.
      /// \param[in] _elevation Reference elevation.
      public: void SetElevationReference(double _elevation);

      /// \brief Set heading angle offset for gazebo frame.
      /// \param[in] _angle Heading offset for gazebo frame.
      public: void SetHeadingOffset(const ignition::math::Angle &_angle);

      /// \brief Convert a Cartesian position vector to geodetic coordinates.
      /// \param[in] _xyz Cartesian position vector in gazebo's world frame.
      /// \return Cooordinates: geodetic latitude (deg), longitude (deg),
      ///         altitude above sea level (m).
      public: ignition::math::Vector3d SphericalFromLocal(
                  const ignition::math::Vector3d &_xyz) const;

      /// \brief Convert a geodetic position vector to Cartesian coordinates.
      /// \param[in] _xyz Geodetic position int the planetary frame of reference
      /// \return Coordinates: Cartesian position vector in gazebo's world frame
      public: ignition::math::Vector3d LocalFromSpherical(
                  const ignition::math::Vector3d &_xyz) const;

      /// \brief Convert a Cartesian velocity vector in the local gazebo frame
      ///        to a global Cartesian frame with components East, North, Up.
      /// \param[in] _xyz Cartesian vector in gazebo's world frame.
      /// \return Rotated vector with components (x,y,z): (East, North, Up).
      public: ignition::math::Vector3d GlobalFromLocal(
                  const ignition::math::Vector3d &_xyz) const;

      /// \brief Convert a Cartesian velocity vector with components East,
      /// North, Up to a global Gazebo cartesian frame vector XYZ
      /// \param[in] _xyz Cartesian vector in gazebo's world frame.
      /// \return Rotated vector with components (x,y,z): (East, North, Up).
      public: ignition::math::Vector3d LocalFromGlobal(
                  const ignition::math::Vector3d &_xyz) const;

      /// \brief Convert a string to a SurfaceType.
      /// \param[in] _str String to convert.
      /// \return Conversion to SurfaceType.
      public: static SurfaceType Convert(const std::string &_str);

      /// \brief Get the distance between two points expressed in geographic
      /// latitude and longitude. It assumes that both points are at sea level.
      /// Example: _latA = 38.0016667 and _lonA = -123.0016667) represents
      /// the point with latitude 38d 0'6.00"N and longitude 123d 0'6.00"W.
      /// \param[in] _latA Latitude of point A.
      /// \param[in] _longA Longitude of point A.
      /// \param[in] _latB Latitude of point B.
      /// \param[in] _longB Longitude of point B.
      /// \return Distance in meters.
      /// \deprecated See Distance() function that accepts
      /// ignition::math::Angle objects.
      public: static double Distance(const math::Angle &_latA,
                  const math::Angle &_lonA,
                  const math::Angle &_latB,
                  const math::Angle &_lonB) GAZEBO_DEPRECATED(6.0);

      /// \brief Get the distance between two points expressed in geographic
      /// latitude and longitude. It assumes that both points are at sea level.
      /// Example: _latA = 38.0016667 and _lonA = -123.0016667) represents
      /// the point with latitude 38d 0'6.00"N and longitude 123d 0'6.00"W.
      /// \param[in] _latA Latitude of point A.
      /// \param[in] _longA Longitude of point A.
      /// \param[in] _latB Latitude of point B.
      /// \param[in] _longB Longitude of point B.
      /// \return Distance in meters.
      public: static double Distance(const ignition::math::Angle &_latA,
                                     const ignition::math::Angle &_lonA,
                                     const ignition::math::Angle &_latB,
                                     const ignition::math::Angle &_lonB);

      /// \brief Update coordinate transformation matrix with reference location
      public: void UpdateTransformationMatrix();

      /// \brief Convert between positions in SPHERICAL/ECEF/LOCAL/GLOBAL frame
      /// \param[in] _pos Position vectot in frame defined by parameter _in
      /// \param[in] _in  CoordinateType for input
      /// \param[in] _out CoordinateType for output
      /// \return Transformed coordinate using cached orgin
      public: ignition::math::Vector3d
              PositionTransform(const ignition::math::Vector3d &_pos,
                  const CoordinateType &_in, const CoordinateType &_out) const;

      /// \brief Convert between velocity in SPHERICAL/ECEF/LOCAL/GLOBAL frame
      /// \param[in] _pos Velocity vector in frame defined by parameter _in
      /// \param[in] _in  CoordinateType for input
      /// \param[in] _out CoordinateType for output
      /// \return Transformed velocity vector
      public: ignition::math::Vector3d VelocityTransform(
                  const ignition::math::Vector3d &_vel,
                  const CoordinateType &_in, const CoordinateType &_out) const;

      /// \internal
      /// \brief Pointer to the private data
      private: SphericalCoordinatesPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
