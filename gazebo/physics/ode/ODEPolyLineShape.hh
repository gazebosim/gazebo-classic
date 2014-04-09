/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _ODEPOLYLINESHAPE_HH_
#define _ODEPOLYLINESHAPE_HH_
#include <vector>

#include "gazebo/math/Vector2d.hh"
#include "gazebo/physics/PolyLineShape.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief ODE polyline shape
    class GAZEBO_VISIBLE ODEPolyLineShape : public PolyLineShape
    {
      /// \brief Constructor
      /// \param[in] _parent Collision parent.
      public: explicit ODEPolyLineShape(ODECollisionPtr _parent)
              : PolyLineShape(_parent) {}

      /// \brief Destructor.
      public: virtual ~ODEPolyLineShape() {}

      // Documentation inherited.
      public: void SetPolylineShape(double _height,
                                    std::vector<math::Vector2d> _vertices)
      {
        PolyLineShape::SetHeight(_height);
        PolyLineShape::SetVertices(_vertices);
        ODECollisionPtr oParent;
        oParent =
          boost::dynamic_pointer_cast<ODECollision>(this->collisionParent);

        if (oParent->GetCollisionId() == NULL)
          oParent->SetCollision(dCreatePolyLine(0, _height, _vertices), true);
        else
          dGeomPolyLineSetParams(oParent->GetCollisionId(), _height, _vertices);
      }
    };
  }
}
#endif
