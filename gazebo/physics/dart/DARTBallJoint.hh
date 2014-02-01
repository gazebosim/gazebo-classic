/*
 * Copyright 2014 Open Source Robotics Foundation
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

#ifndef _GAZEBO_DARTBALLJOINT_HH_
#define _GAZEBO_DARTBALLJOINT_HH_

#include "gazebo/physics/BallJoint.hh"
#include "gazebo/physics/dart/DARTJoint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief An DARTBallJoint
    class DARTBallJoint : public BallJoint<DARTJoint>
    {
      /// \brief Constructor
      /// \param[in] _parent Parent of the Joint
      public: DARTBallJoint(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~DARTBallJoint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited
      public: virtual math::Vector3 GetAnchor(int _index) const;

      // Documentation inherited
      public: virtual math::Vector3 GetGlobalAxis(int /*_index*/) const
              {return math::Vector3();}

      // Documentation inherited
      public: virtual void SetVelocity(int /*index*/, double /*angle*/) {}

      // Documentation inherited
      public: virtual double GetVelocity(int /*index*/) const {return 0;}

      // Documentation inherited
      public: virtual double GetMaxForce(int /*index*/) {return 0;}

      // Documentation inherited
      public: virtual void SetMaxForce(int /*index*/, double /*t*/) {}

      // Documentation inherited
      public: virtual math::Angle GetAngleImpl(int /*index*/) const
              {return math::Angle(0);}

      // Documentation inherited.
      protected: void SetForceImpl(int /*_index*/, double /*_torque*/)
      {
        gzerr << "Not implemented";
      }

      /// \brief
      protected: dart::dynamics::BallJoint *dtBallJoint;
    };
  }
}
#endif
