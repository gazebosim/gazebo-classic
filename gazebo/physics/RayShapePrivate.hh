/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef GAZEBO_PHYSICS_RAYSHAPEPRIVATE_HH_
#define GAZEBO_PHYSICS_RAYSHAPEPRIVATE_HH_

#include <string>

#include <ignition/math/Vector3.hh>
#include "gazebo/physics/ShapePrivate.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for RayShape
    class RayShapePrivate : public ShapePrivate
    {
      // Contact information; this is filled out during collision
      // detection.
      /// \brief Length of the ray.
      public: double contactLen;

      /// \brief Retro reflectance value
      public: double contactRetro;

      /// \brief Fiducial ID value.
      public: int contactFiducial;

      /// \brief Start position of the ray, relative to the body
      public: ignition::math::Vector3d relativeStartPos;

      /// \brief End position of the ray, relative to the body
      public: ignition::math::Vector3d relativeEndPos;

      /// \brief Start position of the ray in global cs
      public: ignition::math::Vector3d globalStartPos;

      /// \brief End position of the ray in global cs
      public: ignition::math::Vector3d globalEndPos;

      /// \brief Name of the object this ray collided with
      public: std::string collisionName;
    };
  }
}
#endif
