/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_PHYSICS_SIMBODY_CYLINDERSHAPE_HH_
#define _GAZEBO_PHYSICS_SIMBODY_CYLINDERSHAPE_HH_

#include "gazebo/physics/CylinderShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief Cylinder collision
    class GZ_PHYSICS_VISIBLE SimbodyCylinderShape : public CylinderShape
    {
      /// \brief Constructor
      public: explicit SimbodyCylinderShape(CollisionPtr _parent);

      /// \brief Destructor
      public: virtual ~SimbodyCylinderShape();

      // Documentation inherited
      public: void SetSize(const double _radius, const double _length);
    };
    /// \}
  }
}
#endif
