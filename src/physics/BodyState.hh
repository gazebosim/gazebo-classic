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
/* Desc: A body state
 * Author: Nate Koenig
 */

#ifndef BODY_STATE_HH
#define BODY_STATE_HH

#include <vector>
#include <string>

#include "sdf/sdf.h"
#include "physics/State.hh"
#include "physics/CollisionState.hh"
#include "math/Pose.hh"

namespace gazebo
{
  namespace physics
  {
    class BodyState : public State
    {
      /// \brief Default constructor
      public: BodyState();

      /// \brief Constructor
      public: BodyState(const BodyPtr _body);

      /// \brief Destructor
      public: virtual ~BodyState();

      /// \brief Load state from SDF element
      public: virtual void Load(sdf::ElementPtr _elem);

      /// \brief Get the body pose
      public: math::Pose GetPose() const;

      /// \brief Get the number of body states
      public: unsigned int GetCollisionStateCount() const;

      /// \brief Get a body state
      public: CollisionState GetCollisionState(unsigned int _index) const;

      /// \brief Get a body state by body name
      public: CollisionState GetCollisionState(
                  const std::string &_collisionName) const;

      /// \brief Fill a State SDF element with state info
      public: void FillStateSDF(sdf::ElementPtr _elem);

      /// \brief Update a Body SDF element with this state info
      public: void UpdateBodySDF(sdf::ElementPtr _elem);

      /// 3D pose of the body relative to the model
      private: math::Pose pose;

      /// Velocity of the body (linear and angular)
      private: math::Pose velocity;

      /// Acceleration of the body (linear and angular)
      private: math::Pose acceleration;

      // Forces on the body(linear and angular)
      private: std::vector<math::Pose> forces;

      private: std::vector<CollisionState> collisionStates;
    };
  }
}
#endif
