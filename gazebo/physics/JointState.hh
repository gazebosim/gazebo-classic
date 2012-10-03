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
/* Desc: A joint state
 * Author: Nate Koenig
 */

#ifndef JOINT_STATE_HH
#define JOINT_STATE_HH

#include <vector>
#include <string>

#include "physics/State.hh"
#include "math/Pose.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief keeps track of state of a physics::Joint
    class JointState : public State
    {
      /// \brief Default constructor
      public: JointState();

      /// \brief Constructor
      public: JointState(JointPtr _joint);

      /// \brief Destructor
      public: virtual ~JointState();

      /// \brief Load state from SDF element
      public: virtual void Load(sdf::ElementPtr _elem);

      /// \brief Get the number of angles
      public: unsigned int GetAngleCount() const;

      /// \brief Get the joint angle
      public: math::Angle GetAngle(unsigned int _axis) const;

      private: std::vector<math::Angle> angles;
    };
    /// \}
  }
}
#endif
