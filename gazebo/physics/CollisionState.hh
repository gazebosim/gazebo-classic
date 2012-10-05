/*
 * Copyright 2011 Nate Koenig
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
/* Desc: A collision state
 * Author: Nate Koenig
 */

#ifndef _COLLISION_STATE_HH_
#define _COLLISION_STATE_HH_

#include "physics/State.hh"
#include "math/Pose.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class CollisionState CollisionState.hh physics/CollisionState.hh
    /// \brief Store state information of a physics::Collision object
    ///
    /// This class captures the entire state of a Collision at one
    /// specific time during a simulation run.
    ///
    /// State of a Collision is its Pose.
    class CollisionState : public State
    {
      /// \brief Default constructor
      public: CollisionState();

      /// \brief Constructor
      ///
      /// Build a CollisionState from an existing Collision.
      /// \param _model Pointer to the Link from which to gather state
      /// info.
      public: CollisionState(const CollisionPtr _collision);

      /// \brief Destructor
      public: virtual ~CollisionState();

      /// \brief Load state from SDF element
      ///
      /// Load CollisionState information from stored data in and SDF::Element
      /// \param _elem Pointer to the SDF::Element containing state info.
      public: virtual void Load(sdf::ElementPtr _elem);

      /// \brief Get the Collision pose
      public: math::Pose GetPose() const;

      /// Pose of the Collision object
      private: math::Pose pose;
    };
    /// \}
  }
}
#endif
