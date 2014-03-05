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
/* Desc: The ODE base joint class
 * Author: Nate Koenig, Andrew Howard
 * Date: 12 Oct 2009
 */

#ifndef _ODEJOINT_HH_
#define _ODEJOINT_HH_

#include <boost/any.hpp>
#include <string>

#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/physics/Joint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief ODE joint interface
    class ODEJoint : public Joint
    {
      /// \brief internal variables used for cfm damping
      public:  enum CFMMode
      {
        /// \brief cfm damping not active
        NONE           = 0x00000000,
        /// \brief cfm damping active, joints within limits
        DAMPING_ACTIVE = 0x00000001,
        /// \brief cfm damping not active, enforcing joints limits
        JOINT_LIMIT    = 0x00000002
      };

      /// \brief Constructor.
      /// \param[in] _parent Parent of the Joint.
      public: ODEJoint(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~ODEJoint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Reset();

      // Documentation inherited.
      public: virtual LinkPtr GetJointLink(int _index) const;

      // Documentation inherited.
      public: virtual bool AreConnected(LinkPtr _one, LinkPtr _two) const;

      /// \brief Get an ODE joint parameter.
      ///
      /// The default function does nothing. This should be
      /// overriden in the child classes where appropriate.
      /// \param[in] _parameter ID of the parameter to get.
      /// \return Value of the parameter.
      public: virtual double GetParam(int _parameter) const;

      /// \brief Set an ODE joint paramter.
      ///
      /// By default this does nothing. It should be overridden in child
      /// classes where appropriate
      /// \param[in] _parameter ID of the parameter to set.
      /// \param[in] _value Value to set.
      public: virtual void SetParam(int _parameter, double _value);

      // Documentation inherited
      public: virtual void SetDamping(int _index, double _damping);

      // Documentation inherited.
      public: virtual void Attach(LinkPtr _parent, LinkPtr _child);

      // Documentation inherited.
      public: virtual void Detach();

      /// \brief Set the ERP of this joint.
      /// \param[in] _erp Error Reduction Parameter value.
      public: void SetERP(double _erp);

      /// \brief Get the ERP of this joint.
      /// \return The Error Reduction Parameter of this joint
      public: double GetERP();

      /// \brief Set the CFM of this joint.
      /// \param[in] _cfm The Constraint Force Mixing value
      public: void SetCFM(double _cfm);

      /// \brief Get the CFM of this joint
      /// \return The Constraint Force Mixing value
      public: double GetCFM();

      /// \brief Get the feedback data structure for this joint, if set
      /// \return Pointer to the joint feedback.
      public: dJointFeedback *GetFeedback();

      /// \brief simulating damping with CFM and meddling with Joint limits
      public: void CFMDamping();

      /// \brief Get access to stopCFM
      /// \return Returns joint's cfm for end stops
      public: double GetStopCFM()
      {
        return this->stopCFM;
      }

      /// \brief Get access to stopERP
      /// \return Returns joint's erp for end stops
      public: double GetStopERP()
      {
        return this->stopERP;
      }

      /// \brief internal variable to keep track of cfm damping internals
      private: int cfmDampingState[3];

      /// \brief current cfm damping for stability reasons
      private: double dStable[3];

      /// \brief internal variable to keep track if ConnectJointUpdate
      /// has been called on a damping method
      private: bool dampingInitialized;

      // Documentation inherited.
      public: virtual void SetHighStop(int _index, const math::Angle &_angle);

      // Documentation inherited.
      public: virtual void SetLowStop(int _index, const math::Angle &_angle);

      // Documentation inherited.
      public: virtual math::Angle GetHighStop(int _index);

      // Documentation inherited.
      public: virtual math::Angle GetLowStop(int _index);

      // Documentation inherited.
      public: virtual math::Vector3 GetLinkForce(unsigned int _index) const;

      // Documentation inherited.
      public: virtual math::Vector3 GetLinkTorque(unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetAttribute(Attribute _attr, int _index,
                                        double _value);

      // Documentation inherited.
      public: virtual void SetAxis(int _index, const math::Vector3 &_axis);

      // Documentation inherited.
      public: virtual void SetAttribute(const std::string &_key, int _index,
                                        const boost::any &_value);

      // Documentation inherited.
      public: virtual double GetAttribute(const std::string &_key,
                                                unsigned int _index);

      // Documentation inherited.
      public: virtual void SetProvideFeedback(bool _enable);

      // Documentation inherited.
      public: virtual JointWrench GetForceTorque(unsigned int _index);

      // Documentation inherited.
      public: virtual void SetForce(int _index, double _force);

      // Documentation inherited.
      public: virtual double GetForce(unsigned int _index);

      // Documentation inherited.
      public: virtual void ApplyDamping();

      // Documentation inherited.
      /// \brief Set the force applied to this physics::Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.
      /// Force is additive (multiple calls
      /// to SetForceImpl to the same joint in the same time
      /// step will accumulate forces on that Joint).
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Force value.
      protected: virtual void SetForceImpl(int _index, double _force) = 0;

      /// \brief Save external forces applied to this Joint.
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Force value.
      private: void SaveForce(int _index, double _force);

      /// \brief This is our ODE ID
      protected: dJointID jointId;

      /// \brief Feedback data for this joint
      private: dJointFeedback *feedback;

      /// \brief CFM for joint's limit constraint
      private: double stopCFM;

      /// \brief ERP for joint's limit constraint
      private: double stopERP;

      /// \brief Save force applied by user
      /// This plus the joint feedback (joint contstraint forces) is the
      /// equivalent of simulated force torque sensor reading
      /// Allocate a 2 vector in case hinge2 joint is used.
      /// This is used by ODE to store external force applied by the user.
      private: double forceApplied[MAX_JOINT_AXIS];

      /// \brief Save time at which force is applied by user
      /// This will let us know if it's time to clean up forceApplied.
      private: common::Time forceAppliedTime;
    };
  }
}
#endif
