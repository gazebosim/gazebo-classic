/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _JOINTSTATE_HH_
#define _JOINTSTATE_HH_

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <vector>
#include <string>

#include "gazebo/physics/State.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class JointState JointState.hh physics/physics.hh
    /// \brief keeps track of state of a physics::Joint
    class GZ_PHYSICS_VISIBLE JointState : public State
    {
      /// \brief Default constructor.
      public: JointState();

      /// \brief Constructor.
      /// \param[in] _joint Joint to get the state of.
      /// \param[in] _realTime Real time stamp.
      /// \param[in] _simTime Sim time stamp.
      /// \param[in] _iterations Simulation iterations.
      public: JointState(JointPtr _joint, const common::Time &_realTime,
                  const common::Time &_simTime, const uint64_t _iterations);

      /// \brief Constructor.
      /// \param[in] _joint Joint to get the state of.
      public: explicit JointState(JointPtr _joint);

      /// \brief Constructor
      ///
      /// Build a JointState from SDF data
      /// \param[in] _sdf SDF data to load a joint state from.
      public: explicit JointState(const sdf::ElementPtr _sdf);

      /// \brief Destructor.
      public: virtual ~JointState();

      /// \brief Load.
      /// \param[in] _joint Joint to get the state of.
      /// \param[in] _realTime Real time stamp.
      /// \param[in] _simTime Sim time stamp.
      public: void Load(JointPtr _joint, const common::Time &_realTime,
                  const common::Time &_simTime);

      /// \brief Load state from SDF element.
      /// \param[in] _elem SDf values to load from.
      public: virtual void Load(const sdf::ElementPtr _elem);

      /// \brief Get the number of angles.
      /// \return The number of angles.
      public: unsigned int GetAngleCount() const;

      /// \brief Get the joint angle.
      /// \param[in] _axis The axis index.
      /// \return Angle of the axis.
      /// \throw common::Exception When _axis is invalid.
      public: math::Angle GetAngle(unsigned int _axis) const;

      /// \brief Get the angles.
      /// \return Vector of angles.
      public: const std::vector<math::Angle> &GetAngles() const;

      /// \brief Return true if the values in the state are zero.
      /// \return True if the values in the state are zero.
      public: bool IsZero() const;

      /// \brief Populate a state SDF element with data from the object.
      /// \param[out] _sdf SDF element to populate.
      public: void FillSDF(sdf::ElementPtr _sdf);

      /// \brief Assignment operator
      /// \param[in] _state State value
      /// \return this
      public: JointState &operator=(const JointState &_state);

      /// \brief Subtraction operator.
      /// \param[in] _pt A state to substract.
      /// \return The resulting state.
      public: JointState operator-(const JointState &_state) const;

      /// \brief Addition operator.
      /// \param[in] _pt A state to add.
      /// \return The resulting state.
      public: JointState operator+(const JointState &_state) const;

      /// \brief Stream insertion operator.
      /// \param[in] _out output stream.
      /// \param[in] _state Joint state to output.
      /// \return The stream.
      public: inline friend std::ostream &operator<<(std::ostream &_out,
                  const gazebo::physics::JointState &_state)
      {
        _out << "<joint name='" << _state.GetName() << "'>";

        int i = 0;
        for (std::vector<math::Angle>::const_iterator iter =
            _state.angles.begin(); iter != _state.angles.end(); ++iter)
        {
          _out << "<angle axis='" << i << "'>" << (*iter) << "</angle>";
        }

        _out << "</joint>";

        return _out;
      }

      private: std::vector<math::Angle> angles;
    };
    /// \}
  }
}
#endif
