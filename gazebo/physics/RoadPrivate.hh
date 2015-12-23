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
#ifndef _GAZEBO_PHYSICS_ROADPRIVATE_HH_
#define _GAZEBO_PHYSICS_ROADPRIVATE_HH_

#include <vector>
#include <ignition/math/Vector3.hh>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/physics/BasePrivate.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data for Road
    class RoadPrivate : public BasePrivate
    {
      /// \brief Width of the road.
      public: double width;

      /// \brief Points that makes up the mid-line of the road.
      public: std::vector<ignition::math::Vector3d> points;

      /// \brief Transportation node.
      public: transport::NodePtr node;

      /// \brief Publisher for road information.
      public: transport::PublisherPtr roadPub;
    };
  }
}
#endif
