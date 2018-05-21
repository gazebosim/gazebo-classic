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

#ifndef _GAZEBO_DARTFIXEDJOINT_HH_
#define _GAZEBO_DARTFIXEDJOINT_HH_

#include "gazebo/physics/FixedJoint.hh"
#include "gazebo/physics/dart/DARTJoint.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_dart
    /// \{

    /// \brief A single axis hinge joint.
    class GZ_PHYSICS_VISIBLE DARTFixedJoint : public FixedJoint<DARTJoint>
    {
      /// \brief Constructor
      /// \param[in] _parent Parent of the Joint
      public: explicit DARTFixedJoint(BasePtr _parent);

      /// \brief Destructor
      public: virtual ~DARTFixedJoint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited
      public: virtual ignition::math::Vector3d GlobalAxis(
          const unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetAxis(const unsigned int _index,
                  const ignition::math::Vector3d &_axis);
    };
    /// \}
  }
}
#endif
