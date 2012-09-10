/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: A link state
 * Author: Nate Koenig
 */

#ifndef LINK_STATE_HH
#define LINK_STATE_HH

#include <vector>
#include <string>

#include "sdf/sdf.hh"
#include "physics/State.hh"
#include "physics/CollisionState.hh"
#include "math/Pose.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Keeps track of states of a physics::Link
    class LinkState : public State
    {
      /// \brief Default constructor
      public: LinkState();

      /// \brief Constructor
      public: LinkState(const LinkPtr _link);

      /// \brief Destructor
      public: virtual ~LinkState();

      /// \brief Load state from SDF element
      public: virtual void Load(sdf::ElementPtr _elem);

      /// \brief Get the link pose
      public: math::Pose GetPose() const;

      /// \brief Get the number of link states
      public: unsigned int GetCollisionStateCount() const;

      /// \brief Get a link state
      public: CollisionState GetCollisionState(unsigned int _index) const;

      /// \brief Get a link state by link name
      public: CollisionState GetCollisionState(
                  const std::string &_collisionName) const;

      /// \brief Fill a State SDF element with state info
      public: void FillStateSDF(sdf::ElementPtr _elem);

      /// \brief Update a Link SDF element with this state info
      public: void UpdateLinkSDF(sdf::ElementPtr _elem);

      /// 3D pose of the link relative to the model
      private: math::Pose pose;

      /// Velocity of the link (linear and angular)
      private: math::Pose velocity;

      /// Acceleration of the link (linear and angular)
      private: math::Pose acceleration;

      // Forces on the link(linear and angular)
      private: std::vector<math::Pose> forces;

      private: std::vector<CollisionState> collisionStates;
    };
    /// \}
  }
}
#endif
