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

#ifndef _GAZEBO_DARTSPHERESHAPE_PRIVATE_HH_
#define _GAZEBO_DARTSPHERESHAPE_PRIVATE_HH_

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for DARTSphereShape
    class DARTSphereShapePrivate
    {
      /// \brief Constructor
      public: DARTSphereShapePrivate()
      {
      }

      /// \brief Default destructor
      public: ~DARTSphereShapePrivate() = default;

      // \brief returns the shape
      public: dart::dynamics::ShapeNodePtr ShapeNode() const
      {
        return this->dtSphereShape;
      }

      // \brief returns the shape
      public: dart::dynamics::SphereShape* Shape() const
      {
        GZ_ASSERT(this->dtSphereShape, "SphereShape is NULL");
        return static_cast<dart::dynamics::SphereShape*>
                      (this->dtSphereShape->getShape().get());
      }

      /// \brief Creates the shape
      /// \param[in] _bodyNode the body node to use for the shape
      public: void CreateShape(const dart::dynamics::BodyNodePtr& _bodyNode)
      {
        GZ_ASSERT(_bodyNode, "BodyNode is NULL");
        dart::dynamics::ShapePtr shape(new dart::dynamics::SphereShape(1));
        dart::dynamics::ShapeNode *node = _bodyNode->createShapeNodeWith<
                                      dart::dynamics::VisualAspect,
                                      dart::dynamics::CollisionAspect,
                                      dart::dynamics::DynamicsAspect>(shape);
        this->dtSphereShape.set(node);
      }

      /// \brief DART sphere shape
      private: dart::dynamics::ShapeNodePtr dtSphereShape;
    };
  }
}
#endif
