/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#ifndef _GAZEBO_WRENCH_HH_
#define _GAZEBO_WRENCH_HH_

#include "gazebo/math/Vector3.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class Wrench Wrench.hh physics/physics.hh
    /// \brief Wrench information.  These are
    /// forces and torques relative to a reference frame.
    class Wrench
    {
      /// \brief Operator =
      /// \param[in] _wrench wrench to set from.
      /// \return *this
      public: Wrench &operator =(const Wrench &_wrench)
              {
                this->force = _wrench.force;
                this->torque = _wrench.torque;
                return *this;
              }

      /// \brief Operator +
      /// \param[in] _wrench wrench to add
      /// \return *this
      public: inline Wrench &operator +(const Wrench &_wrench)
              {
                this->force += _wrench.force;
                this->torque += _wrench.torque;
                return *this;
              }

      /// \brief Operator -
      /// \param[in] _wrench wrench to subtract
      /// \return *this
      public: inline Wrench &operator -(const Wrench &_wrench)
              {
                this->force -= _wrench.force;
                this->torque -= _wrench.torque;
                return *this;
              }

      /// \brief linear forces
      public: math::Vector3 force;

      /// \brief angular torques
      public: math::Vector3 torque;

      /// \brief reference link frame
      public: physics::LinkPtr referenceFrame;
    };
    /// \}
  }
}
#endif
