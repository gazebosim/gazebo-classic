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
#ifndef _GAZEBO_PHYSICS_BULLET_BULLETLINKPRIVATE_HH_
#define _GAZEBO_PHYSICS_BULLET_BULLETLINKPRIVATE_HH_

#include "gazebo/physics/LinkPrivate.hh"
#include "gazebo/physics/bullet/bullet_inc.h"

namespace gazebo
{
  namespace physics
  {
    class BulletLinkPrivate : public LinkPrivate
    {
      /// \brief Pointer to bullet compound shape, which is a container
      ///        for other child shapes.
      public: btCollisionShape *compoundShape;

      /// \brief Pointer to bullet motion state, which manages updates to the
      ///        world pose from bullet.
      public: BulletMotionStatePtr motionState;

      /// \brief Pointer to the bullet rigid body object.
      public: btRigidBody *rigidLink;

      /// \brief Pointer to the bullet physics engine.
      public: BulletPhysicsPtr bulletPhysics;
    };
  }
}

#endif
