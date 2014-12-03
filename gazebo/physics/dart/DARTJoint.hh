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

#ifndef _GAZEBO_DARTJOINT_HH_
#define _GAZEBO_DARTJOINT_HH_

#include <boost/any.hpp>
#include <string>

#include "gazebo/common/Exception.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief DART joint interface
    class GZ_PHYSICS_VISIBLE DARTJoint : public Joint
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent of the Joint.
      public: DARTJoint(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~DARTJoint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize a joint.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void Reset();

      // Documentation inherited.
      public: virtual LinkPtr GetJointLink(unsigned int _index) const;

      // Documentation inherited.
      public: virtual bool AreConnected(LinkPtr _one, LinkPtr _two) const;

      // Documentation inherited.
      public: virtual void Attach(LinkPtr _parent, LinkPtr _child);

      // Documentation inherited.
      public: virtual void Detach();

      /// \brief Set the anchor point
      public: virtual void SetAnchor(unsigned int /*_index*/,
                                     const gazebo::math::Vector3 &/*_anchor*/);

      // Documentation inherited
      public: virtual void SetDamping(unsigned int _index, double _damping);

      // Documentation inherited.
      public: virtual void SetStiffness(unsigned int _index,
                  const double _stiffness);

      // Documentation inherited.
      public: virtual void SetStiffnessDamping(unsigned int _index,
        double _stiffness, double _damping, double _reference = 0);

      // Documentation inherited.
      public: virtual bool SetHighStop(unsigned int _index,
                  const math::Angle &_angle);

      // Documentation inherited.
      public: virtual bool SetLowStop(unsigned int _index,
                  const math::Angle &_angle);

      // Documentation inherited.
      public: virtual math::Angle GetHighStop(unsigned int _index);

      // Documentation inherited.
      public: virtual math::Angle GetLowStop(unsigned int _index);

      // Documentation inherited.
      public: virtual math::Vector3 GetLinkForce(unsigned int _index) const;

      // Documentation inherited.
      public: virtual math::Vector3 GetLinkTorque(unsigned int _index) const;

      // Documentation inherited.
      public: virtual bool SetParam(const std::string &_key,
                                        unsigned int _index,
                                        const boost::any &_value);

      // Documentation inherited.
      public: virtual double GetParam(const std::string &_key,
                                          unsigned int _index);

      // Documentation inherited.
      public: virtual JointWrench GetForceTorque(unsigned int _index);

      // Documentation inherited.
      public: virtual void SetForce(unsigned int _index, double _force);

      // Documentation inherited.
      public: virtual double GetForce(unsigned int _index);

      // Documentation inherited.
      public: virtual unsigned int GetAngleCount() const;

      // Documentation inherited.
      public: virtual void ApplyDamping();

      /// \brief Set the force applied to this physics::Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.
      /// Force is additive (multiple calls
      /// to SetForceImpl to the same joint in the same time
      /// step will accumulate forces on that Joint).
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Force value.
      protected: virtual void SetForceImpl(unsigned int _index,
                     double _force) = 0;

      /// \brief Save external forces applied to this Joint.
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Force value.
      private: void SaveForce(unsigned int _index, double _force);

      /// \brief Get DART model pointer.
      /// \return A pointer to the DART model.
      public: DARTModelPtr GetDARTModel() const;

      /// \brief Get DART joint pointer.
      /// \return A pointer to the DART joint.
      public: dart::dynamics::Joint *GetDARTJoint();

      /// \brief Save force applied by user
      /// This plus the joint feedback (joint contstraint forces) is the
      /// equivalent of simulated force torque sensor reading
      /// Allocate a 2 vector in case hinge2 joint is used.
      /// This is used by DART to store external force applied by the user.
      private: double forceApplied[MAX_JOINT_AXIS];

      /// \brief Save time at which force is applied by user
      /// This will let us know if it's time to clean up forceApplied.
      private: common::Time forceAppliedTime;

      /// \brief DARTPhysics engine pointer
      protected: DARTPhysicsPtr dartPhysicsEngine;

      /// \brief DART joint pointer
      protected: dart::dynamics::Joint *dtJoint;

      /// \brief DART child body node pointer
      protected: dart::dynamics::BodyNode *dtChildBodyNode;
    };
  }
}
#endif
