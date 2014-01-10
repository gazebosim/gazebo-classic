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

#ifndef _GAZEBO_DARTSCREWJOINT_HH_
#define _GAZEBO_DARTSCREWJOINT_HH_

#include "gazebo/physics/ScrewJoint.hh"
#include "gazebo/physics/dart/DARTJoint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief A screw joint.
    class DARTScrewJoint : public ScrewJoint<DARTJoint>
    {
      /// \brief Constructor.
      /// \param[in] _parent Pointer to the Link that is the joint' parent
      public: DARTScrewJoint(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~DARTScrewJoint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited
      public: virtual math::Vector3 GetGlobalAxis(int _index) const;

      // Documentation inherited
      public: virtual void SetAxis(int _index, const math::Vector3 &_axis);

      // Documentation inherited
      public: virtual void SetThreadPitch(int _index, double _threadPitch);

      // Documentation inherited
      public: virtual double GetThreadPitch(unsigned int _index);

      // Documentation inherited
      public: virtual math::Angle GetAngleImpl(int _index) const;

      // Documentation inherited
      public: virtual double GetVelocity(int _index) const;

      // Documentation inherited
      public: virtual void SetVelocity(int _index, double _vel);

      // Documentation inherited
      public: virtual void SetMaxForce(int _index, double _force);

      // Documentation inherited
      public: virtual double GetMaxForce(int _index);

      // Documentation inherited.
      protected: virtual void SetForceImpl(int _index, double _effort);

      /// \brief Universal joint of DART
      protected: dart::dynamics::ScrewJoint *dartScrewJoint;
    };
  }
}
#endif
