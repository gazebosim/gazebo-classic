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

#ifndef _GAZEBO_DARTJOINT_PRIVATE_HH_
#define _GAZEBO_DARTJOINT_PRIVATE_HH_

#include "gazebo/common/Time.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for DARTJoint
    class DARTJointPrivate
    {
      /// \brief Constructor
      public: DARTJointPrivate(const DARTPhysicsPtr &_dartPhysicsEngine)
        : forceApplied {0.0, 0.0},
          forceAppliedTime(),
          dartPhysicsEngine(_dartPhysicsEngine),
          dtJoint(NULL),
          dtChildBodyNode(NULL)
      {
      }

      /// \brief Default destructor
      public: ~DARTJointPrivate() = default;

      /// \brief Save force applied by user
      /// This plus the joint feedback (joint contstraint forces) is the
      /// equivalent of simulated force torque sensor reading
      /// Allocate a 2 vector in case hinge2 joint is used.
      /// This is used by DART to store external force applied by the user.
      public: double forceApplied[MAX_JOINT_AXIS];

      /// \brief Save time at which force is applied by user
      /// This will let us know if it's time to clean up forceApplied.
      public: common::Time forceAppliedTime;

      /// \brief DARTPhysics engine pointer
      public: DARTPhysicsPtr dartPhysicsEngine;

      /// \brief DART joint pointer
      public: dart::dynamics::Joint *dtJoint;

      /// \brief DART child body node pointer
      public: dart::dynamics::BodyNode *dtChildBodyNode;
    };
  }
}
#endif
