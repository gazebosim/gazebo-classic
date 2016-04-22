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
      public: LinkPtr GetJointLink(unsigned int _index) const;

      /// \brief Determines of the two bodies are connected by a joint
      public: bool AreConnected(LinkPtr _one, LinkPtr _two) const;

      /// \brief Detach this joint from all bodies
      public: virtual void Detach();

      /// \brief Set the anchor point
      public: virtual void SetAnchor(unsigned int _index,
                                     const gazebo::math::Vector3 &_anchor);

      // Documentation inherited
      public: virtual void SetDamping(unsigned int _index, double _damping);

      // Documentation inherited.
      public: virtual bool SetPosition(unsigned int _index, double _position);

      // Documentation inherited.
      public: virtual void SetStiffness(unsigned int _index,
                  const double _stiffness);

      // Documentation inherited.
      public: virtual void SetStiffnessDamping(unsigned int _index,
        double _stiffness, double _damping, double _reference = 0);

      /// \brief Get the anchor point
      public: virtual math::Vector3 GetAnchor(unsigned int _index) const;

      /// \brief Get the force the joint applies to the first body
      /// \param index The index of the body(0 or 1)
      public: virtual math::Vector3 GetLinkForce(unsigned int _index) const;

      /// \brief Get the torque the joint applies to the first body
      /// \param index The index of the body(0 or 1)
      public: virtual math::Vector3 GetLinkTorque(unsigned int _index) const;

      // Documentation inherited.
      public: virtual bool SetParam(const std::string &_key,
                                        unsigned int _index,
                                        const boost::any &_value);

      // Documentation inherited.
      public: virtual double GetParam(const std::string &_key,
                                          unsigned int _index);

      // Documentation inherited.
      public: virtual math::Angle GetHighStop(unsigned int _index);

      // Documentation inherited.
      public: virtual math::Angle GetLowStop(unsigned int _index);

      // Documentation inherited.
      public: virtual void SetProvideFeedback(bool _enable);

      // Documentation inherited.
      public: virtual void CacheForceTorque();

      // Documentation inherited.
      public: virtual JointWrench GetForceTorque(unsigned int _index);

      // Documentation inherited.
      public: virtual void SetForce(unsigned int _index, double _force);

      // Documentation inherited.
      public: virtual double GetForce(unsigned int _index);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void ApplyStiffnessDamping();

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
      protected: virtual void SetForceImpl(unsigned int _index,
                     double _force) = 0;

      /// \brief: Setup joint feedback datatructure.
      /// This is called after Joint::constraint is setup in Init.
      protected: void SetupJointFeedback();

      /// \brief Save external forces applied to this Joint.
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Force value.
      private: void SaveForce(unsigned int _index, double _force);

      /// \brief Pointer to a contraint object in Bullet.
      protected: btTypedConstraint *constraint;

      /// \brief Pointer to Bullet's btDynamicsWorld.
      protected: btDynamicsWorld *bulletWorld;

      /// \brief Feedback data for this joint
      private: btJointFeedback *feedback;

      /// \brief internal variable to keep track if ConnectJointUpdate
      /// has been called on a damping method
      private: bool stiffnessDampingInitialized;

      /// \brief Save force applied by user
      /// This plus the joint feedback (joint contstraint forces) is the
      /// equivalent of simulated force torque sensor reading
      /// Allocate a 2 vector in case hinge2 joint is used.
      /// This is used by Bullet to store external force applied by the user.
      private: double forceApplied[MAX_JOINT_AXIS];

      /// \brief Save time at which force is applied by user
      /// This will let us know if it's time to clean up forceApplied.
      private: common::Time forceAppliedTime;
    };
    /// \}
  }
}
#endif
