/*
 * Copyright 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_DARTPLANESHAPE_HH_
#define _GAZEBO_DARTPLANESHAPE_HH_

#include "gazebo/physics/PlaneShape.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief An DART Plane shape.
    class GAZEBO_VISIBLE DARTPlaneShape : public PlaneShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit DARTPlaneShape(CollisionPtr _parent)
              : PlaneShape(_parent) {}

      /// \brief Destructor.
      public: virtual ~DARTPlaneShape() {}

      // Documentation inherited
      public: virtual void CreatePlane()
      {
        PlaneShape::CreatePlane();

        DARTCollisionPtr dartCollisionParent =
            boost::dynamic_pointer_cast<DARTCollision>(this->collisionParent);

        // math::Vector3 n = this->GetNormal();

        dart::dynamics::BodyNode *dtBodyNode =
            dartCollisionParent->GetDARTBodyNode();
        dart::dynamics::BoxShape *dtBoxShape =
            new dart::dynamics::BoxShape(Eigen::Vector3d(2100, 2100, 0.01));
        dtBodyNode->addCollisionShape(dtBoxShape);
        dtBoxShape->setOffset(Eigen::Vector3d(0.0, 0.0, -0.005));
        dartCollisionParent->SetDARTCollisionShape(dtBoxShape, false);
      }

      // Documentation inherited
      public: virtual void SetAltitude(const math::Vector3 &_pos)
      {
        PlaneShape::SetAltitude(_pos);
      }
    };
  }
}
#endif
