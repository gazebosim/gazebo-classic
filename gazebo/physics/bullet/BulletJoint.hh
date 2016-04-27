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
/* Desc: The base Bullet joint class
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef _BULLETJOINT_HH_
#define _BULLETJOINT_HH_

#include <boost/any.hpp>
#include <string>

#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    // Forward declar private class.
    class BulletJointPrivate;

    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief Base class for all joints
    class GZ_PHYSICS_VISIBLE BulletJoint : public Joint
    {
      /// \brief Constructor
      public: BulletJoint(BasePtr _parent);

      /// \brief Destructor
      public: virtual ~BulletJoint();

      /// \brief Load a BulletJoint
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Fini();

      /// \brief Reset the joint
      public: virtual void Reset();

      /// \brief Get the body to which the joint is attached
      ///        according the _index
      /// \deprecated See JointLink
      public: LinkPtr GetJointLink(unsigned int _index) const;

      /// \brief Get the body to which the joint is attached
      ///        according the _index
      public: LinkPtr JointLink(const unsigned int _index) const;

      /// \brief Determines of the two bodies are connected by a joint
      public: bool AreConnected(LinkPtr _one, LinkPtr _two) const;

      /// \brief Detach this joint from all bodies
      public: virtual void Detach();

      /// \brief Set the anchor point
      public: virtual void SetAnchor(const unsigned int _index,
                  const ignition::math::Vector3d &_anchor);

      // Documentation inherited
      public: virtual void SetDamping(const unsigned int _index,
                  const double _damping);

      // Documentation inherited.
      public: virtual bool SetPosition(const unsigned int _index,
                  const double _position);

      // Documentation inherited.
      public: virtual void SetStiffness(const unsigned int _index,
                  const double _stiffness);

      // Documentation inherited.
      public: virtual void SetStiffnessDamping(
                  const unsigned int _index,
                  const double _stiffness,
                  const double _damping, const double _reference = 0);

      /// \brief Get the anchor point
      public: virtual ignition::math::Vector3d Anchor(
                  const unsigned int _index) const;

      /// \brief Get the force the joint applies to the first body
      /// \param index The index of the body(0 or 1)
      public: virtual ignition::math::Vector3d LinkForce(
                  const unsigned int _index) const;

      /// \brief Get the torque the joint applies to the first body
      /// \param index The index of the body(0 or 1)
      public: virtual ignition::math::Vector3d LinkTorque(
                  const unsigned int _index) const;

      // Documentation inherited.
      public: virtual bool SetParam(const std::string &_key,
                  const unsigned int _index, const boost::any &_value);

      // Documentation inherited.
      public: virtual double Param(const std::string &_key,
                  const unsigned int _index) const;

      // Documentation inherited.
      public: virtual ignition::math::Angle HighStop(
                  const unsigned int _index) const;

      // Documentation inherited.
      public: virtual ignition::math::Angle LowStop(
                  const unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetProvideFeedback(const bool _enable);

      // Documentation inherited.
      public: virtual void CacheForceTorque();

      // Documentation inherited.
      public: virtual JointWrench ForceTorque(const unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetForce(const unsigned int _index,
                  const double _force);

      // Documentation inherited.
      public: virtual double Force(const unsigned int _index) const;

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void ApplyStiffnessDamping();

      // Documentation inherited.
      public: virtual void SetAxis(const unsigned int _index,
                                   const ignition::math::Vector3d &_axis);

      /// \brief Set the force applied to this physics::Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.
      /// Force is additive (multiple calls
      /// to SetForceImpl to the same joint in the same time
      /// step will accumulate forces on that Joint).
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Force value.
      /// internal force, e.g. damping forces.  This way, Joint::appliedForce
      /// keep track of external forces only.
      protected: virtual void SetForceImpl(const unsigned int _index,
                     const double _force) = 0;

      /// \brief: Setup joint feedback datatructure.
      /// This is called after Joint::constraint is setup in Init.
      protected: void SetupJointFeedback();

      /// \brief Save external forces applied to this Joint.
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Force value.
      private: void SaveForce(const unsigned int _index, const double _force);

      /// \internal
      /// \brief Private data pointer
      protected: BulletJointPrivate *bulletJointDPtr;
    };
    /// \}
  }
}
#endif
