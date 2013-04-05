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

#ifndef _RTQL8CYLINDERSHAPE_HH_
#define _RTQL8CYLINDERSHAPE_HH_

#include "gazebo/physics/CylinderShape.hh"
#include "gazebo/physics/rtql8/RTQL8Physics.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief RTQL8 cylinder shape
    class RTQL8CylinderShape : public CylinderShape
    {
      /// \brief Constructor
      /// \param[in] _parent Collision parent.
      public: explicit RTQL8CylinderShape(CollisionPtr _parent)
              : CylinderShape(_parent) {}

      /// \brief Destructor.
      public: virtual ~RTQL8CylinderShape() {}

      // Documentation inerited.
      public: void SetSize(double /*_radius*/, double /*_length*/)
      {
//      CylinderShape::SetSize(_radius, _length);
//      RTQL8CollisionPtr oParent;
//      oParent =
//        boost::shared_dynamic_cast<RTQL8Collision>(this->collisionParent);
// 
//      if (oParent->GetCollisionId() == NULL)
//        oParent->SetCollision(dCreateCylinder(0, _radius, _length), true);
//      else
//        dGeomCylinderSetParams(oParent->GetCollisionId(), _radius, _length);
      }
    };
  }
}
#endif
