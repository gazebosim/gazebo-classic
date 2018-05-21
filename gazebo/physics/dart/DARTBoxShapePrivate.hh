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

#ifndef _GAZEBO_DARTBOXSHAPE_PRIVATE_HH_
#define _GAZEBO_DARTBOXSHAPE_PRIVATE_HH_

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for DARTBoxShape
    class DARTBoxShapePrivate
    {
      /// \brief Constructor
      public: DARTBoxShapePrivate()
      {
      }

      /// \brief Default destructor
      public: ~DARTBoxShapePrivate() = default;

      // \brief returns the shape
      public: dart::dynamics::ShapeNodePtr ShapeNode() const
      {
        return this->dtBoxShape;
      }

      // \brief returns the shape
      public: dart::dynamics::BoxShape* Shape() const
      {
        GZ_ASSERT(dtBoxShape, "BoxShape is NULL");
        return static_cast<dart::dynamics::BoxShape*>
                        (dtBoxShape->getShape().get());
      }

      /// \brief Creates the shape
      /// \param[in] _bodyNode the body node to use for the shape
      public: void CreateShape(const dart::dynamics::BodyNodePtr& _bodyNode)
      {
        GZ_ASSERT(_bodyNode, "BodyNode is NULL");
        dart::dynamics::ShapePtr shape(
          new dart::dynamics::BoxShape(Eigen::Vector3d(1, 1, 1)));
        dart::dynamics::ShapeNode *node =
          _bodyNode->createShapeNodeWith<dart::dynamics::VisualAspect,
                                      dart::dynamics::CollisionAspect,
                                      dart::dynamics::DynamicsAspect>(shape);
        this->dtBoxShape.set(node);
      }

      /// \brief DART box shape
      private: dart::dynamics::ShapeNodePtr dtBoxShape;
    };
  }
}

#endif
