/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _SIMBODY_JOINT_HH_
#define _SIMBODY_JOINT_HH_

#include <boost/any.hpp>
#include <string>

#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief Base class for all joints
    class GAZEBO_VISIBLE SimbodyJoint : public Joint
    {
      /// \brief Constructor
      public: SimbodyJoint(BasePtr _parent);

      /// \brief Destructor
      public: virtual ~SimbodyJoint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Reset();

      // Documentation inherited.
      public: virtual LinkPtr GetJointLink(unsigned int _index) const;

      // Documentation inherited.
      public: virtual bool AreConnected(LinkPtr _one, LinkPtr _two) const;

      // Documentation inherited.
      public: virtual void Detach();

      // Documentation inherited.
      public: virtual void SetAnchor(unsigned int _index,
                  const gazebo::math::Vector3 &_anchor);

      // Documentation inherited.
      public: virtual void SetDamping(unsigned int _index,
                                      const double _damping);

      // Documentation inherited.
      public: virtual void SetStiffness(unsigned int _index,
                                        const double _stiffness);

      // Documentation inherited.
      public: virtual void SetStiffnessDamping(unsigned int _index,
        double _stiffness, double _damping, double _reference = 0);

      // Documentation inherited.
      public: virtual math::Vector3 GetAnchor(unsigned int _index) const;

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

      // Save current Simbody State
      public: virtual void SaveSimbodyState(const SimTK::State &_state);

      // Restore saved Simbody State
      public: virtual void RestoreSimbodyState(SimTK::State &_state);

      // Documentation inherited.
      public: virtual void SetForce(unsigned int _index, double _force);

      // Documentation inherited.
      public: virtual double GetForce(unsigned int _index);

      // Documentation inherited.
      public: virtual void SetAxis(unsigned int _index,
                                   const math::Vector3 &_axis);

      // Documentation inherited.
      public: virtual JointWrench GetForceTorque(unsigned int _index);

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

      /// \brief Save external forces applied to this Joint.
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Force value.
      private: void SaveForce(unsigned int _index, double _force);

      // Documentation inherited.
      public: virtual void CacheForceTorque();

      /// \brief Force Simbody to break a loop by using a weld constraint.
      /// This flag is needed by SimbodyPhysics::MultibodyGraphMaker, so kept
      /// public.
      public: bool mustBreakLoopHere;

      /// \brief Normally A=F, B=M. But if reversed, then B=F, A=M.
      /// parent body frame to mobilizer frame
      public: SimTK::Transform xPA;

      /// \brief child body frame to mobilizer frame
      public: SimTK::Transform xCB;

      /// \brief default mobilizer pose
      public: SimTK::Transform defxAB;

      /// \brief: Spring force element for enforcing joint stiffness.
      /// The element is assigned when constructing Simbody model in
      /// SimbodyPhysics::AddDynamicModelToSimbodySystem.
      /// \TODO: Also, consider moving this into individual joint type subclass
      /// so we can specify custom springs for special joints like ball joints.
      public: SimTK::Force::MobilityLinearSpring spring[MAX_JOINT_AXIS];

      /// \brief: for enforcing joint damping forces.
      /// Set when we build the Simbody model.
      /// \TODO: Also, consider moving this into individual joint type subclass
      /// so we can specify custom dampers for special joints like ball joints.
      public: SimTK::Force::MobilityLinearDamper damper[MAX_JOINT_AXIS];

      /// \brief: for enforcing joint stops
      /// Set when we build the Simbody model.
      /// \TODO: Also, consider moving this into individual joint type subclass
      /// so we can specify custom dampers for special joints like ball joints.
      /// Assuming this is not used for BallJoints it's ok here for now.
      public: SimTK::Force::MobilityLinearStop limitForce[MAX_JOINT_AXIS];

      /// \brief Use isValid() if we used a mobilizer
      /// Set when we build the Simbody model.
      /// How this joint was modeled in the Simbody System. We used either a
      /// mobilizer or a constraint, but not both. The type of either one is
      /// the same as the joint type above.
      public: SimTK::MobilizedBody mobod;

      /// \brief: if mobilizer, did it reverse parent&child?
      /// Set when we build the Simbody model.
      public: bool isReversed;

      /// \brief: isValid() if we used a constraint to model this joint.
      /// Set when we build the Simbody model.
      /// How this joint was modeled in the Simbody System. We used either a
      /// mobilizer or a constraint, but not both. The type of either one is the
      /// same as the joint type above.
      public: SimTK::Constraint constraint;

      // Keeps track if simbody physics has been initialized
      public: bool physicsInitialized;

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

      /// \brief Simbody Multibody System
      protected: SimTK::MultibodySystem *world;

      /// \brief keep a pointer to the simbody physics engine for convenience
      protected: SimbodyPhysicsPtr simbodyPhysics;

      /// \brief Save force applied by user
      /// This plus the joint feedback (joint contstraint forces) is the
      /// equivalent of simulated force torque sensor reading
      /// Allocate a 2 vector in case hinge2 joint is used.
      /// This is used by Bullet to store external force applied by the user.
      /// \TODO: Also, consider moving this into individual joint type subclass
      /// so we can specify custom dampers for special joints like ball joints.
      /// Assuming this is not used for BallJoints it's ok here for now.
      private: double forceApplied[MAX_JOINT_AXIS];

      /// \brief Save time at which force is applied by user
      /// This will let us know if it's time to clean up forceApplied.
      private: common::Time forceAppliedTime;
    };
    /// \}
  }
}
#endif
