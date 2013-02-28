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

      /// \brief internal variable to keep track of cfm damping internals
      private: int cfmDampingState[2];

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
      public: virtual void SetAttribute(const std::string &_key, int _index,
                                        const boost::any &_value);

      /// \brief This is our ODE ID
      protected: dJointID jointId;

      /// \brief Feedback data for this joint
      private: dJointFeedback *feedback;

      /// \brief Provide Feedback data for contact forces
      private: bool provideFeedback;

      // Documentation inherited.
      public: virtual JointWrench GetForceTorque(int _index);

      /// \brief Provide Feedback data for contact forces
      private: double stopCFM[2];
      private: double stopERP[2];

      /// \brief Get access to stopCFM
      public: double GetStopCFM(unsigned int _int)
      {
        if (_int < this->GetAngleCount())
          return this->stopCFM[_int];
        gzerr << "index out of bound when calling GetStopCFM.\n";
        return 0;
      }

      /// \brief Get access to stopERP
      public: double GetStopERP(unsigned int _int)
      {
        if (_int < this->GetAngleCount())
          return this->stopERP[_int];
        gzerr << "index out of bound when calling GetStopERP.\n";
        return 0;
      }

    };
  }
}
#endif
