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

#ifndef _GAZEBO_DARTPLANESHAPE_PRIVATE_HH_
#define _GAZEBO_DARTPLANESHAPE_PRIVATE_HH_

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for DARTPlaneShape
    class DARTPlaneShapePrivate
    {
      /// \brief Constructor
      public: DARTPlaneShapePrivate()
      {
      }

      /// \brief Default destructor
      public: ~DARTPlaneShapePrivate() = default;

      public: dart::dynamics::BoxShape* GetShape()
      {
        GZ_ASSERT(dtBoxShape.get() != nullptr, "BodyNode is NULL");
        return static_cast<dart::dynamics::BoxShape*>
          (dtBoxShape->getShape().get());
      }

      public: dart::dynamics::ShapeNodePtr GetShapeNode()
      {
        return dtBoxShape;
      }

      public: void CreateShape(const dart::dynamics::BodyNodePtr& bodyNode)
      {
        GZ_ASSERT(bodyNode.get() != nullptr,
                  "BodyNode is NULL");
        dart::dynamics::ShapePtr shape(new dart::dynamics::BoxShape(
                                         Eigen::Vector3d(2100, 2100, 2100)));
        dart::dynamics::ShapeNode *node = bodyNode->createShapeNodeWith<
                                      dart::dynamics::VisualAspect,
                                      dart::dynamics::CollisionAspect,
                                      dart::dynamics::DynamicsAspect>(shape);
        Eigen::Isometry3d trans = Eigen::Isometry3d::Identity();
        trans.translate(Eigen::Vector3d(0.0, 0.0, -2100*0.5));
        node->setRelativeTransform(trans);
        dtBoxShape.set(node);
      }

      /// \brief DART box shape
      private: dart::dynamics::ShapeNodePtr dtBoxShape;
      // We use BoxShape untile PlaneShape is completely supported in DART.
      // Please see: https://github.com/dartsim/dart/issues/114
    };
  }
}
#endif
