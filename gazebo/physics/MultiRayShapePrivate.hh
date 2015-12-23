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
#ifndef _GAZEBO_PHYSICS_MULTIRAYSHAPEPRIVATE_HH_
#define _GAZEBO_PHYSICS_MULTIRAYSHAPEPRIVATE_HH_
#include <vector>
#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/common/Event.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/ShapePrivate.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data for MultiRayShape
    class MultiRayShapePrivate : public ShapePrivate
    {
      /// \brief Ray data
      public: std::vector<RayShapePtr> rays;

      /// \brief Pose offset of all the rays.
      public: ignition::math::Pose3d offset;

      /// \brief Ray SDF element pointer.
      public: sdf::ElementPtr rayElem;

      /// \brief Scan SDF element pointer.
      public: sdf::ElementPtr scanElem;

      /// \brief Horizontal SDF element pointer.
      public: sdf::ElementPtr horzElem;

      /// \brief Vertical SDF element pointer.
      public: sdf::ElementPtr vertElem;

      /// \brief Range SDF element pointer.
      public: sdf::ElementPtr rangeElem;

      /// \brief New laser scans event.
      public: event::EventT<void()> newLaserScans;
    };
  }
}
#endif
