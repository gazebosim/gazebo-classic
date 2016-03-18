/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_PHYSICS_BULLET_BULLETJOINT_PRIVATE_HH_
#define _GAZEBO_PHYSICS_BULLET_BULLETJOINT_PRIVATE_HH_

#include "gazebo/physics/JointPrivate.hh"
#include "gazebo/physics/bullet/bullet_inc.h"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Bullet joint private class
    class BulletJointPrivate : public JointPrivate
    {
      /// \brief Pointer to a contraint object in Bullet.
      public: btTypedConstraint *constraint;

      /// \brief Pointer to Bullet's btDynamicsWorld.
      public: btDynamicsWorld *bulletWorld;

      /// \brief Feedback data for this joint
      public: btJointFeedback *feedback;

      /// \brief internal variable to keep track if ConnectJointUpdate
      /// has been called on a damping method
      public: bool stiffnessDampingInitialized;

      /// \brief Save force applied by user
      /// This plus the joint feedback (joint contstraint forces) is the
      /// equivalent of simulated force torque sensor reading
      /// Allocate a 2 vector in case hinge2 joint is used.
      /// This is used by Bullet to store external force applied by the user.
      public: double forceApplied[MAX_JOINT_AXIS];

      /// \brief Save time at which force is applied by user
      /// This will let us know if it's time to clean up forceApplied.
      public: common::Time forceAppliedTime;
    };
  }
}

#endif
