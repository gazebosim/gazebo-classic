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
      public: enum SurfaceType {
                /// \brief Model of reference ellipsoid for earth, based on
                /// WGS 84 standard. see wikipedia: World_Geodetic_System
                EARTH_WGS84 = 1
              };

      /// \brief Constructor.
      public: SphericalCoordinates();

      /// \brief Constructor with surface model input.
      /// \param[in] _model SurfaceModel specification.
      public: SphericalCoordinates(const SurfaceType _model);

      /// \brief Constructor with surface model and angle inputs.
      /// \param[in] _model SurfaceModel specification.
      /// \param[in] _latitude Reference latitude.
      /// \param[in] _longitude Reference longitude.
      /// \param[in] _heading Heading offset.
      public: SphericalCoordinates(const SurfaceType _model,
                                   const math::Angle &_latitude,
                                   const math::Angle &_longitude,
                                   const math::Angle &_heading);

      /// \brief Destructor.
      public: ~SphericalCoordinates();

      /// \brief Convert a Cartesian position vector to geodetic coordinates.
      /// \return Cooordinates: geodetic latitude (deg), longitude (deg),
      ///         altitude above sea level (m).
      public: math::Vector3 Convert(const math::Vector3 &_xyz) const;

      // TODO: finish doxygen comments
      /// \brief Convert a string to a SurfaceType.
      public: static SurfaceType Convert(const std::string &_str);

      public: SurfaceType GetSurfaceModel() const;
      public: math::Angle GetLatitudeReference() const;
      public: math::Angle GetLongitudeReference() const;
      public: math::Angle GetHeadingOffset() const;
      public: void SetSurfaceModel(const SurfaceType &_model);
      public: bool SetSurfaceModel(const std::string &_str);
      public: void SetLatitudeReference(const math::Angle &_angle);
      public: void SetLongitudeReference(const math::Angle &_angle);
      public: void SetHeadingOffset(const math::Angle &_angle);

      /// \brief Type of surface model being used.
      private: SurfaceType surfaceModel;

      /// \brief Latitude of reference point.
      private: math::Angle latitudeReference;

      /// \brief Longitude of reference point.
      private: math::Angle longitudeReference;

      /// \brief Heading offset, expressed as angle from East to
      ///        gazebo x-axis, or equivalently from North to gazebo y-axis.
      private: math::Angle headingOffset;
    };
    /// \}
  }
}
#endif
