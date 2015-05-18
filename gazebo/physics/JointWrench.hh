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
/* Desc: Specification of a contact
 * Author: Nate Koenig
 * Date: 10 Nov 2009
 */

#ifndef _JOINT_WRENCH_HH_
#define _JOINT_WRENCH_HH_

#include "gazebo/math/Vector3.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class JointWrench JointWrench.hh physics/physics.hh
    /// \brief Wrench information from a joint.  These are
    /// forces and torques on parent and child Links, relative to
    /// the Joint frame immediately after rotation.
    class GZ_PHYSICS_VISIBLE JointWrench
    {
      /// \brief Operator =
      /// \param[in] _wrench Joint wrench to set from.
      /// \return *this
      public: JointWrench &operator =(const JointWrench &_wrench)
              {
                this->body1Force = _wrench.body1Force;
                this->body2Force = _wrench.body2Force;

                this->body1Torque = _wrench.body1Torque;
                this->body2Torque = _wrench.body2Torque;
                return *this;
              }

      /// \brief Operator +
      /// \param[in] _wrench Joint wrench to add
      /// \return *this
      public: inline JointWrench &operator +(const JointWrench &_wrench)
              {
                this->body1Force += _wrench.body1Force;
                this->body2Force += _wrench.body2Force;

                this->body1Torque += _wrench.body1Torque;
                this->body2Torque += _wrench.body2Torque;
                return *this;
              }

      /// \brief Operator -
      /// \param[in] _wrench Joint wrench to subtract
      /// \return *this
      public: inline JointWrench &operator -(const JointWrench &_wrench)
              {
                this->body1Force -= _wrench.body1Force;
                this->body2Force -= _wrench.body2Force;

                this->body1Torque -= _wrench.body1Torque;
                this->body2Torque -= _wrench.body2Torque;
                return *this;
              }

      /// \brief Force on the first link.
      public: math::Vector3 body1Force;

      /// \brief Force on the second link.
      public: math::Vector3 body2Force;

      /// \brief Torque on the first link.
      public: math::Vector3 body1Torque;

      /// \brief Torque on the second link.
      public: math::Vector3 body2Torque;
    };
    /// \}
  }
}
#endif
