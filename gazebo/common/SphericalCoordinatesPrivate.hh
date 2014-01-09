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

#ifndef _SPHERICALCOORDINATES_PRIVATE_HH_
#define _SPHERICALCOORDINATES_PRIVATE_HH_

#include "gazebo/math/Angle.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common
    /// \{

    /// \class SphericalCoordinatesPrivate SphericalCoordinatesPrivate.hh
    ///        commmon/common.hh
    /// \brief Private data for the SphericalCoordinates class.
    class SphericalCoordinatesPrivate
    {
      /// \enum SurfaceType
      /// \brief Unique identifiers for planetary surface models.
      public: enum SurfaceType
              {
                /// \brief Model of reference ellipsoid for earth, based on
                /// WGS 84 standard. see wikipedia: World_Geodetic_System
                EARTH_WGS84 = 1
              };

      /// \brief Type of surface being used.
      public: SurfaceType surfaceType;

      /// \brief Latitude of reference point.
      public: math::Angle latitudeReference;

      /// \brief Longitude of reference point.
      public: math::Angle longitudeReference;

      /// \brief Elevation of reference point relative to sea level in meters.
      public: double elevationReference;

      /// \brief Heading offset, expressed as angle from East to
      ///        gazebo x-axis, or equivalently from North to gazebo y-axis.
      public: math::Angle headingOffset;
    };
    /// \}
  }
}
#endif
