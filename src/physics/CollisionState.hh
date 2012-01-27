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
/* Desc: A collision state
 * Author: Nate Koenig
 */

#ifndef COLLISION_STATE_HH
#define COLLISION_STATE_HH

#include "physics/State.hh"
#include "math/Pose.hh"

namespace gazebo
{
  namespace physics
  {
    class CollisionState : public State
    {
      /// \brief Default constructor
      public: CollisionState();

      /// \brief Constructor
      public: CollisionState(const CollisionPtr _collision);

      /// \brief Destructor
      public: virtual ~CollisionState();

      /// \brief Get the collision pose
      public: math::Pose GetPose() const;

      private: math::Pose pose;
    };
  }
}
#endif
