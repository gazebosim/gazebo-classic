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

#ifndef _RTQL8HINGEJOINT_HH_
#define _RTQL8HINGEJOINT_HH_

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Vector3.hh"

#include "gazebo/physics/HingeJoint.hh"
#include "gazebo/physics/rtql8/RTQL8Joint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief A single axis hinge joint.
    class RTQL8HingeJoint : public HingeJoint<RTQL8Joint>
    {
      /// \brief Constructor
      /// \param[in] _parent Parent of the Joint
      public: RTQL8HingeJoint(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~RTQL8HingeJoint();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual math::Vector3 GetAnchor(int _index) const;

      // Documentation inherited
      public: virtual void SetAnchor(int _index, const math::Vector3 &_anchor);

      // Documentation inherited
      public: virtual math::Vector3 GetGlobalAxis(int _index) const;

      // Documentation inherited
      public: virtual void SetAxis(int _index, const math::Vector3 &_axis);

      // Documentation inherited
      public: virtual void SetDamping(int _index, double _damping);

      /// \brief callback to apply damping force to joint.
      public: void ApplyDamping();

      // Documentation inherited
      public: virtual math::Angle GetAngleImpl(int _index) const;

      // Documentation inherited
      public: virtual void SetVelocity(int _index, double _angle);

      // Documentation inherited
      public: virtual double GetVelocity(int _index) const;

      // Documentation inherited
      public: virtual void SetMaxForce(int _index, double _t);

      // Documentation inherited
      public: virtual double GetMaxForce(int _index);

      // Documentation inherited
      public: virtual void SetForce(int _index, double _torque);

      // Documentation inherited
      public: virtual double GetParam(int _parameter) const;

      // Documentation inherited
      public: virtual void SetParam(int _parameter, double _value);
    };
  }
}
#endif
