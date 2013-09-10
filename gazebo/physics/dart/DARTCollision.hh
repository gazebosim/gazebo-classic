/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _DARTCOLLISION_HH_
#define _DARTCOLLISION_HH_

#include "gazebo/common/CommonTypes.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Collision.hh"

#include "gazebo/physics/dart/dart_inc.h"

namespace gazebo
{
  namespace physics
  {
    /// \brief Base class for all DART collisions.
    class DARTCollision : public Collision
    {
      /// \brief Constructor.
      /// \param[in] _link Parent Link
      public: explicit DARTCollision(LinkPtr _parent);

      /// \brief Destructor.
      public: virtual ~DARTCollision();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void Fini();

      /// \brief Set the encapsulated collsion object.
      /// \param[in] _placeable True to make the object m.
      public: virtual void SetCollision(bool _placeable);

      // Documentation inherited.
      public: virtual void OnPoseChange();

      // Documentation inherited.
      public: virtual void SetCategoryBits(unsigned int bits);

      // Documentation inherited.
      public: virtual void SetCollideBits(unsigned int bits);

      // Documentation inherited.
      public: virtual math::Box GetBoundingBox() const;

      /// @brief DART body node associated with this collision.
      public: dart::dynamics::BodyNode* dartBodyNode;

      /// @brief DART collision shape associated with this collision.
      public: dart::dynamics::Shape* dartCollShape;
    };
  }
}
#endif
