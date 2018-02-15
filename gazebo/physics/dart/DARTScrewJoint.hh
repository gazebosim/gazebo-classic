/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
      public: explicit DARTScrewJoint(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~DARTScrewJoint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf) override;

      // Documentation inherited.
      public: virtual void Init() override;

      // Documentation inherited
      public: virtual double GetVelocity(unsigned int _index) const override;

      // Documentation inherited
      public: virtual ignition::math::Vector3d GlobalAxis(
          const unsigned int _index) const override;

      // Documentation inherited
      public: virtual void SetAxis(const unsigned int _index,
                  const ignition::math::Vector3d &_axis) override;

      /// \copydoc ScrewJoint::SetThreadPitch
      public: virtual void SetThreadPitch(unsigned int _index,
                  double _threadPitch);

      /// \copydoc ScrewJoint::SetThreadPitch
      public: virtual void SetThreadPitch(double _threadPitch) override;

      /// \copydoc ScrewJoint::GetThreadPitch
      public: virtual double GetThreadPitch(unsigned int _index);

      /// \copydoc ScrewJoint::GetThreadPitch
      public: virtual double GetThreadPitch() override;

      // Documentation inherited
      public: virtual double GetParam(const std::string &_key,
                                      unsigned int _index) override;

      // Documentation inherited
      public: virtual bool SetParam(const std::string &_key,
                                    unsigned int _index,
                                    const boost::any &_value) override;

      public: virtual double PositionImpl(
        const unsigned int _index = 0) const override;
    };
    /// \}
  }
}
#endif
