/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#ifndef GAZEBO_PHYSICS_ROAD_HH_
#define GAZEBO_PHYSICS_ROAD_HH_

#include <string>
#include <vector>
#include <algorithm>
#include <ignition/math/Vector3.hh>
#include <ignition/transport/Node.hh>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/physics/Base.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class Road Road.hh physics/physics.hh
    /// \brief for building a Road from SDF
    class GZ_PHYSICS_VISIBLE Road : public Base
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent of this road object.
      public: explicit Road(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~Road();

      /// \brief Load the road from SDF.
      /// \param[in] _sdf SDF values to load from.
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the road.
      public: virtual void Init();

      /// \brief Finalize the road.
      public: virtual void Fini();

      /// \brief Get the points that define the road.
      /// \return The vector of points that define the road.
      public: const std::vector<ignition::math::Vector3d> &Points() const;

      /// \brief Get the road width in meters.
      /// \return Road width in meters.
      public: double GetWidth() const;

      /// \brief Width of the road.
      private: double width;

      /// \brief Points that makes up the mid-line of the road.
      private: std::vector<ignition::math::Vector3d> points;

      /// \brief Transportation node.
      private: transport::NodePtr node;

      /// \brief Publisher for road information.
      private: transport::PublisherPtr roadPub;

      // Place ignition::transport objects at the end of this file to
      // guarantee they are destructed first.

      /// \brief Transportation node.
      private: ignition::transport::Node nodeIgn;

      /// \brief Publisher for road information.
      private: ignition::transport::Node::Publisher roadPubIgn;
    };
    /// \}
  }
}
#endif
