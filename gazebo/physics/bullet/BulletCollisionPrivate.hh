/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_PHYSICS_BULLET_BULLETCOLLISIONPRIVATE_HH_
#define _GAZEBO_PHYSICS_BULLET_BULLETCOLLISIONPRIVATE_HH_

#include "gazebo/physics/CollisionPrivate.hh"

namespace gazebo
{
  namespace physics
  {
    class BulletCollisionPrivate : public CollisionPrivate
    {
      public: btCollisionShape *collisionShape;

      /// \brief Category bits for collision detection
      public: unsigned int categoryBits;

      /// \brief Collide bits for collision detection
      public: unsigned int collideBits;
    };
  }
}
#endif
