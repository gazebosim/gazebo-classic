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
#ifndef _GAZEBO_PHYSICS_SHAPEPRIVATE_HH_
#define _GAZEBO_PHYSICS_SHAPEPRIVATE_HH_

#include <ignition/math/Vector3.hh>

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/BasePrivate.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Shape private data class
    class ShapePrivate : public BasePrivate
    {
      /// \brief This shape's collision parent.
      public: CollisionPtr collisionParent;

      /// \brief This shape's scale;
      public: ignition::math::Vector3d scale = ignition::math::Vector3d::One;
    };
  }
}
#endif
