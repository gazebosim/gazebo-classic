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
#ifndef _PluginBOXSHAPE_HH_
#define _PluginBOXSHAPE_HH_

#include "gazebo/math/Vector3.hh"

#include "gazebo/physics/plugin/PluginPhysics.hh"
#include "gazebo/physics/plugin/PluginTypes.hh"
#include "gazebo/physics/plugin/PluginCollision.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/BoxShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief Plugin Box shape
    class GAZEBO_VISIBLE PluginBoxShape : public BoxShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit PluginBoxShape(PluginCollisionPtr _parent)
              : BoxShape(_parent) {}

      /// \brief Destructor.
      public: virtual ~PluginBoxShape() {}

      // Documentation inherited.
      public: virtual void SetSize(const math::Vector3 &_size)
      {
        BoxShape::SetSize(_size);

        PluginCollisionPtr oParent;
        oParent = boost::dynamic_pointer_cast<PluginCollision>(
            this->collisionParent);
      }
    };
  }
}
#endif
