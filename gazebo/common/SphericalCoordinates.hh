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

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/sdf/sdf.hh"

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
      /// \enum SurfaceModelType
      /// \brief Unique identifies for planetary surface models.
      public: enum SurfaceModelType {
                /// \brief Model of reference ellipsoid for earth, based on
                /// WGS 84 standard. see wikipedia: World_Geodetic_System
                EARTH_WGS84 = 1
              };

      /// \brief Constructor.
      public: explicit SphericalCoordinates();

      /// \brief Destructor.
      public: ~SphericalCoordinates();

      /// \brief Load using SDF parameters.
      /// \param[in] _sdf SDF parameters.
      public: void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: void Init();

      /// \brief Convert a Cartesian position vector to geodetic coordinates.
      /// \return Cooordinates: geodetic latitude (deg), longitude (deg),
      ///         altitude (m).
      public: math::Vector3 Convert(const math::Vector3 &_xyz) const;

      /// \brief The SDF values for this object.
      private: sdf::ElementPtr sdf;

      /// \brief Type of surface model being used.
      private: SurfaceModelType surfaceModel;

      /// \brief Latitude of reference point.
      private: math::Angle latitudeReference;

      /// \brief Longitude of reference point.
      private: math::Angle longitudeReference;

      /// \brief Heading offset, expressed as angle from East to
      ///        gazebo x-axis, or equivalently from North to gazebo y-axis.
      private: math::Angle headingOffset;

      /// \brief Cosine of heading offset angle.
      private: double headingCosine;

      /// \brief Sine of heading offset angle.
      private: double headingSine;

      /// \brief Meridional radius of curvature of earth at reference latitude.
      private: double radiusMeridional;

      /// \brief Radius of curvature of earth at reference latitude computed
      ///        as ellipsoidal normal.
      private: double radiusNormal;
    };
    /// \}
  }
}
#endif
