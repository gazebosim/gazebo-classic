/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_PHYSICS_ODE_ODEGEARBOXJOINT_HH_
#define GAZEBO_PHYSICS_ODE_ODEGEARBOXJOINT_HH_
#include <string>

#include <ignition/math/Angle.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/physics/GearboxJoint.hh"
#include "gazebo/physics/ode/ODEJoint.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \class ODEGearboxJoint ODEGearboxJoint.hh physics/physics.hh
    /// \brief A double axis gearbox joint.
    class GZ_PHYSICS_VISIBLE ODEGearboxJoint : public GearboxJoint<ODEJoint>
    {
      /// \brief Constructor
      /// \param[in] _worldID ODE id of the world.
      /// \param[in] _parent Parent of the Joint
      public: ODEGearboxJoint(dWorldID _worldId, BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~ODEGearboxJoint();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual ignition::math::Vector3d Anchor(
                  const unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetAnchor(const unsigned int _index,
                                     const ignition::math::Vector3d &_anchor);

      // Documentation inherited
      public: virtual ignition::math::Vector3d GlobalAxis(
                  const unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetAxis(const unsigned int _index,
                                   const ignition::math::Vector3d &_axis);

      // Documentation inherited
      public: virtual void SetGearboxRatio(const double _gearRatio);

      // Documentation inherited
      public: virtual ignition::math::Angle AngleImpl(
                  const unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetVelocity(const unsigned int _index,
                  const double _angle);

      // Documentation inherited
      public: virtual double Velocity(const unsigned int _index) const;

      // Documentation inherited
      public: virtual double Param(const unsigned int _parameter) const;

      // Documentation inherited.
      public: virtual double Param(const std::string &_key,
                                   const unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetParam(const unsigned int _parameter,
                  const double _value);

      // Documentation inherited
      protected: virtual void SetForceImpl(const unsigned int _index,
                     const double _effort);

      /// \brief Set gearbox joint gear reference body
      /// \param[in] _body an ode body as the reference link for the gears.
      private: void SetReferenceBody(LinkPtr _body);
    };
  }
}
#endif
