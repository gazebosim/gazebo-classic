/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _RTQL8BALLJOINT_HH_
#define _RTQL8BALLJOINT_HH_

#include "gazebo/physics/BallJoint.hh"
#include "gazebo/physics/rtql8/RTQL8Joint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief An RTQL8BallJoint
    class RTQL8BallJoint : public BallJoint<RTQL8Joint>
    {
      /// \brief Constructor
      public: RTQL8BallJoint(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~RTQL8BallJoint();

      // Documentation inherited
      public: virtual math::Vector3 GetAnchor(int _index) const;

      // Documentation inherited
      public: virtual void SetAnchor(int _index, const math::Vector3 &_anchor);

      // Documentation inherited
      public: virtual math::Vector3 GetGlobalAxis(int /*_index*/) const
              {return math::Vector3();}

      // Documentation inherited
      public: virtual void SetDamping(int _index, double _damping);

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
    };
  }
}
#endif
