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

#ifndef _JOINT_STATE_HH_
#define _JOINT_STATE_HH_

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

    /// \class JointState JointState.hh physics/physics.hh
    /// \brief keeps track of state of a physics::Joint
    class JointState : public State
    {
      /// \brief Default constructor.
      public: JointState();

      /// \brief Constructor.
      /// \param[in] _joint Joint to get the state of.
      public: explicit JointState(JointPtr _joint);

      /// \brief Destructor.
      public: virtual ~JointState();

      /// \brief Load state from SDF element.
      /// \param[in] _elem SDf values to load from.
      public: virtual void Load(sdf::ElementPtr _elem);

      /// \brief Get the number of angles.
      /// \return The number of angles.
      public: unsigned int GetAngleCount() const;

      /// \brief Get the joint angle.
      /// \param[in] _axis The axis index.
      /// \return Angle of the axis.
      public: math::Angle GetAngle(unsigned int _axis) const;

      /// \brief Vector of all the axis angles.
      private: std::vector<math::Angle> angles;
    };
    /// \}
  }
}
#endif
