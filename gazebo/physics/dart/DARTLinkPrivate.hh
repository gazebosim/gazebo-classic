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

#ifndef _GAZEBO_DARTLINK_PRIVATE_HH_
#define _GAZEBO_DARTLINK_PRIVATE_HH_

#include <vector>

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for DARTLink
    class DARTLinkPrivate
    {
      /// \brief Constructor
      public: DARTLinkPrivate()
        : dartPhysics(NULL),
          dtBodyNode(NULL),
          dartParentJoint(NULL),
          dartChildJoints {},
          staticLink(false),
          dtWeldJointConst(NULL)
      {
      }

      /// \brief Default destructor
      public: ~DARTLinkPrivate()
      {
        // We don't need to delete dtBodyNode because skeletone will delete
        // dtBodyNode if it is registered to the skeletone.

        delete dtWeldJointConst;
      }

      /// \brief Pointer to the DART physics engine.
      public: DARTPhysicsPtr dartPhysics;

      /// \brief Pointer to the DART BodyNode.
      public: dart::dynamics::BodyNode *dtBodyNode;

      /// \brief Pointer to the parent joint.
      public: DARTJointPtr dartParentJoint;

      /// \brief List of pointers to the child joints.
      public: std::vector<DARTJointPtr> dartChildJoints;

      /// \brief If true, freeze link to world (inertial) frame.
      public: bool staticLink;

      /// \brief Weld joint constraint for SetLinkStatic()
      public: dart::constraint::WeldJointConstraint *dtWeldJointConst;
    };
  }
}
#endif
