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

#ifndef _DARTSLIDERJOINT_HH_
#define _DARTSLIDERJOINT_HH_

#include "gazebo/physics/SliderJoint.hh"
#include "gazebo/physics/dart/DARTJoint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief A slider joint
    class DARTSliderJoint : public SliderJoint<DARTJoint>
    {
      /// \brief Constructor
      /// \param[in] _parent Pointer to the Link that is the joint' parent
      public: DARTSliderJoint(BasePtr _parent);

      /// \brief Destructor
      public: virtual ~DARTSliderJoint();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited
      public: virtual math::Vector3 GetAnchor(int _index) const;

      // Documentation inherited
      public: virtual void SetAnchor(int /*_index*/,
                                     const math::Vector3& /*_anchor*/)
      {
        // nothing to do here for DART.
      }

      // Documentation inherited
      public: virtual math::Vector3 GetGlobalAxis(int _index) const;

      // Documentation inherited
      public: virtual void SetAxis(int _index, const math::Vector3& _axis);

      // Documentation inherited
      public: virtual math::Angle GetAngleImpl(int _index) const;

      // Documentation inherited
      public: virtual void SetVelocity(int /*_index*/, double /*_vel*/)
      {
        // TODO: Do nothing because DART accept only torques (forces) of joint
        // as input.
        gzwarn << "Not implemented DARTHingeJoint::SetVelocity().\n";
      }

      // Documentation inherited
      public: virtual double GetVelocity(int _index) const;

      // Documentation inherited
      public: virtual void SetMaxForce(int _index, double _torque);

      // Documentation inherited
      public: virtual double GetMaxForce(int _index);

      // Documentation inherited
      public: virtual void SetForce(int _index, double _torque);

      // Documentation inherited.
      protected: virtual void SetForceImpl(int _index, double _effort);

      /// \brief
      protected: dart::dynamics::PrismaticJoint* dartPrismaticJoint;
    };
  }
}
#endif
