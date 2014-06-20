/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _SPHERICALCOORDINATES_HH_
#define _SPHERICALCOORDINATES_HH_

#include <string>

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Vector3.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common
    /// \{

    /// \class SphericalCoordinates SphericalCoordinates.hh commmon/common.hh
    /// \brief Convert spherical coordinates for planetary surfaces.
    class SphericalCoordinates
    {
      /// \enum SurfaceType
      /// \brief Unique identifiers for planetary surface models.
      public: enum SurfaceType
              {
                /// \brief Model of reference ellipsoid for earth, based on
                /// WGS 84 standard. see wikipedia: World_Geodetic_System
                EARTH_WGS84 = 1
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
      public: SphericalCoordinates(const SurfaceType _type,
                                   const math::Angle &_latitude,
                                   const math::Angle &_longitude,
                                   double _elevation,
                                   const math::Angle &_heading);

      /// \brief Destructor.
      public: ~SphericalCoordinates();

      /// \brief Convert a Cartesian position vector to geodetic coordinates.
      /// \param[in] _xyz Cartesian position vector in gazebo's world frame.
      /// \return Cooordinates: geodetic latitude (deg), longitude (deg),
      ///         altitude above sea level (m).
      public: math::Vector3 SphericalFromLocal(const math::Vector3 &_xyz)
              const;

      /// \brief Convert a Cartesian velocity vector in the local gazebo frame
      ///        to a global Cartesian frame with components East, North, Up.
      /// \param[in] _xyz Cartesian vector in gazebo's world frame.
      /// \return Rotated vector with components (x,y,z): (East, North, Up).
      public: math::Vector3 GlobalFromLocal(const math::Vector3 &_xyz)
              const;

      /// \brief Convert a string to a SurfaceType.
      /// \param[in] _str String to convert.
      /// \return Conversion to SurfaceType.
      public: static SurfaceType Convert(const std::string &_str);

      /// \brief Get SurfaceType currently in use.
      /// \return Current SurfaceType value.
      public: SurfaceType GetSurfaceType() const;

      /// \brief Get reference geodetic latitude.
      /// \return Reference geodetic latitude.
      public: math::Angle GetLatitudeReference() const;

      /// \brief Get reference longitude.
      /// \return Reference longitude.
      public: math::Angle GetLongitudeReference() const;

      /// \brief Get reference elevation in meters.
      /// \return Reference elevation.
      public: double GetElevationReference() const;

      /// \brief Get heading offset for gazebo reference frame, expressed as
      ///        angle from East to gazebo x-axis, or equivalently
      ///        from North to gazebo y-axis.
      /// \return Heading offset of gazebo reference frame.
      public: math::Angle GetHeadingOffset() const;

      /// \brief Set SurfaceType for planetary surface model.
      /// \param[in] _type SurfaceType value.
      public: void SetSurfaceType(const SurfaceType &_type);

      /// \brief Set reference geodetic latitude.
      /// \param[in] _angle Reference geodetic latitude.
      public: void SetLatitudeReference(const math::Angle &_angle);

      /// \brief Set reference longitude.
      /// \param[in] _angle Reference longitude.
      public: void SetLongitudeReference(const math::Angle &_angle);

      /// \brief Set reference elevation above sea level in meters.
      /// \param[in] _elevation Reference elevation.
      public: void SetElevationReference(double _elevation);

      /// \brief Set heading angle offset for gazebo frame.
      /// \param[in] _angle Heading offset for gazebo frame.
      public: void SetHeadingOffset(const math::Angle &_angle);

      /// \brief Type of surface being used.
      private: SurfaceType surfaceType;

      /// \brief Latitude of reference point.
      private: math::Angle latitudeReference;

      /// \brief Longitude of reference point.
      private: math::Angle longitudeReference;

      /// \brief Elevation of reference point relative to sea level in meters.
      private: double elevationReference;

      /// \brief Heading offset, expressed as angle from East to
      ///        gazebo x-axis, or equivalently from North to gazebo y-axis.
      private: math::Angle headingOffset;
    };
    /// \}
  }
}
#endif
