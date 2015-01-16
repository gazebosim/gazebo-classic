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

#include <map>
#include <string>
#include <vector>

#include "gazebo/physics/ode/ODETypes.hh"

namespace gazebo
{
  namespace physics
  {

    /// \brief Data structure for contact feedbacks
    class GAZEBO_VISIBLE ODEJointFeedback
    {
      public: ODEJointFeedback() : contact(NULL), count(0) {}

      /// \brief Contact information.
      public: Contact *contact;

      /// \brief Number of elements in feedbacks array.
      public: int count;

      /// \brief Contact joint feedback information.
      public: dJointFeedback feedbacks[MAX_CONTACT_JOINTS];
    };


    class GAZEBO_VISIBLE ODEPhysicsPrivate
    { 
      /// \brief Top-level world for all bodies
      public: dWorldID worldId;

      /// \brief Top-level space for all sub-spaces/collisions
      public: dSpaceID spaceId;

      /// \brief Collision attributes
      public: dJointGroupID contactGroup;

      /// \brief The type of the solver.
      public: std::string stepType;

      /// \brief Buffer of contact feedback information.
      public: std::vector<ODEJointFeedback*> jointFeedbacks;

      /// \brief All the collsiion spaces.
      public: std::map<std::string, dSpaceID> spaces;

      /// \brief All the normal colliders.
      public: std::vector< std::pair<ODECollision*, ODECollision*> > colliders;

      /// \brief All the triangle mesh colliders.
      public: std::vector< std::pair<ODECollision*, ODECollision*> >
               trimeshColliders;

      /// \brief Array of contact collisions.
      public: dContactGeom contactCollisions[MAX_COLLIDE_RETURNS];

      /// \brief Indices used during creation of contact joints.
      public: int indices[MAX_CONTACT_JOINTS];
    };
  }
}
