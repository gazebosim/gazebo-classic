/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include <string>

#include "gazebo/physics/ScrewJoint.hh"
#include "gazebo/physics/dart/DARTJoint.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_dart
    /// \{

    /// \brief A screw joint.
    class GZ_PHYSICS_VISIBLE DARTScrewJoint : public ScrewJoint<DARTJoint>
    {
      /// \brief Constructor.
      /// \param[in] _parent Pointer to the Link that is the joint' parent
      public: DARTScrewJoint(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~DARTScrewJoint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual math::Vector3 GetAnchor(unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetAnchor(unsigned int _index,
                  const math::Vector3 &_anchor);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited
      public: virtual math::Vector3 GetGlobalAxis(unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetAxis(unsigned int _index,
                  const math::Vector3 &_axis);

      /// \copydoc ScrewJoint::SetThreadPitch
      public: virtual void SetThreadPitch(unsigned int _index,
                  double _threadPitch);

      /// \copydoc ScrewJoint::SetThreadPitch
      public: virtual void SetThreadPitch(double _threadPitch);

      /// \copydoc ScrewJoint::GetThreadPitch
      public: virtual double GetThreadPitch(unsigned int _index);

      /// \copydoc ScrewJoint::GetThreadPitch
      public: virtual double GetThreadPitch();

      // Documentation inherited
      public: virtual double GetParam(const std::string &_key,
                                      unsigned int _index);

      // Documentation inherited
      public: virtual bool SetParam(const std::string &_key,
                                    unsigned int _index,
                                    const boost::any &_value);

      // Documentation inherited
      public: virtual math::Angle GetAngleImpl(unsigned int _index) const;

      // Documentation inherited
      public: virtual double GetVelocity(unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetVelocity(unsigned int _index, double _vel);

      // Documentation inherited.
      public: virtual math::Angle GetHighStop(unsigned int _index);

      // Documentation inherited.
      public: virtual math::Angle GetLowStop(unsigned int _index);

      // Documentation inherited.
      protected: virtual void SetForceImpl(unsigned int _index, double _effort);
    };
    /// \}
  }
}
#endif
