/*
 * Copyright 2011 Nate Koenig
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
/* Desc: The base joint class
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef JOINT_HH
#define JOINT_HH

#include <string>

#include "common/Event.hh"
#include "math/Angle.hh"
#include "math/Vector3.hh"
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"

#include "physics/JointState.hh"
#include "physics/Base.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Base class for all joints
    class Joint : public Base
    {
      /// \brief Type of joint
      public: enum Attribute {FUDGE_FACTOR,
                              SUSPENSION_ERP,
                              SUSPENSION_CFM,
                              STOP_ERP,
                              STOP_CFM,
                              ERP,
                              CFM,
                              FMAX,
                              VEL,
                              HI_STOP,
                              LO_STOP};

      /// \brief Constructor
      public: Joint(BasePtr _parent);

      /// \brief Destructor
      public: virtual ~Joint();

      /// \brief Set pose, parent and child links of a physics::Joint
      public: void Load(LinkPtr _parent, LinkPtr _child,
                        const math::Pose &_origin);

      /// \brief Load physics::Joint from a SDF sdf::Element
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize a joint
      public: virtual void Init();

      /// \brief Update the joint
      public: void Update();

      /// \brief update the parameters using new sdf values
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Reset the joint
      public: virtual void Reset();

      /// \brief Get the joint state
      public: JointState GetState();

      /// \brief Set the joint state
      public: void SetState(const JointState &_state);

      /// \brief Set the model this joint belongs too
      public: void SetModel(ModelPtr model);

      /// \brief Get the link to which the joint is attached according
      ///        the _index
      public: virtual LinkPtr GetJointLink(int index) const = 0;

      /// \brief Determines of the two bodies are connected by a joint
      public: virtual bool AreConnected(LinkPtr one, LinkPtr two) const = 0;

      /// \brief Attach the two bodies with this joint
      public: virtual void Attach(LinkPtr parent, LinkPtr child);

      /// \brief Detach this joint from all links
      public: virtual void Detach();

      /// \brief Set the axis of rotation
      public: virtual void SetAxis(int index, const math::Vector3 &axis) = 0;

      /// \brief Set the joint damping
      public: virtual void SetDamping(int index, const double damping) = 0;

      /// \brief Connect a boost::slot the the joint update signal
      public: template<typename T>
              event::ConnectionPtr ConnectJointUpdate(T subscriber)
              { return jointUpdate.Connect(subscriber); }

      /// \brief Disconnect a boost::slot the the joint update signal
      public: void DisconnectJointUpdate(event::ConnectionPtr &c)
              { jointUpdate.Disconnect(c); }

      /// \brief Get the axis of rotation
      public: math::Vector3 GetLocalAxis(int _index) const;

      public: virtual math::Vector3 GetGlobalAxis(int _index) const = 0;

      /// \brief Set the anchor point
      public: virtual void SetAnchor(int index,
                                     const math::Vector3 &anchor) = 0;

      /// \brief Get the anchor point
      public: virtual math::Vector3 GetAnchor(int index) const = 0;

      /// \brief Set the high stop of an axis(index).
      public: virtual void SetHighStop(int index, math::Angle angle) = 0;

      /// \brief Set the low stop of an axis(index).
      public: virtual void SetLowStop(int index, math::Angle angle) = 0;

      /// \brief Get the high stop of an axis(index).
      public: virtual math::Angle GetHighStop(int index) = 0;

      /// \brief Get the low stop of an axis(index).
      public: virtual math::Angle GetLowStop(int index) = 0;

      /// \brief Set the velocity of an axis(index).
      public: virtual void SetVelocity(int index, double v) = 0;

      /// \brief Get the rotation rate of an axis(index)
      public: virtual double GetVelocity(int index) const = 0;

      /// \brief Set the force applied to this physics::Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.  E.g.  if you are using
      /// metric units, the unit for force is Newtons.  If using
      /// imperial units (sorry), then unit of force is lb-force
      /// not (lb-mass), etc.
      public: virtual void SetForce(int /*index*/, double /*f*/) {}

      /// \brief @todo: not yet implemented.
      /// Get the internal forces at a this Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.  E.g.  if you are using
      /// metric units, the unit for force is Newtons.  If using
      /// imperial units (sorry), then unit of force is lb-force
      /// not (lb-mass), etc.
      public: virtual double GetForce(int /*index*/) {return 0;}

      /// \brief Set the max allowed force of an axis(index).
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.  E.g.  if you are using
      /// metric units, the unit for force is Newtons.  If using
      /// imperial units (sorry), then unit of force is lb-force
      /// not (lb-mass), etc.
      public: virtual void SetMaxForce(int index, double t) = 0;

      /// \brief Get the max allowed force of an axis(index).
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.  E.g.  if you are using
      /// metric units, the unit for force is Newtons.  If using
      /// imperial units (sorry), then unit of force is lb-force
      /// not (lb-mass), etc.
      public: virtual double GetMaxForce(int index) = 0;

      /// \brief Get the angle of rotation of an axis(index)
      public: math::Angle GetAngle(int index) const;

      /// If the Joint is static, Gazebo stores the state of
      /// this Joint as a scalar inside the Joint class, so
      /// this call will NOT move the joint dynamically for a static Model.
      /// But if this Model is not static, then it is updated dynamically,
      /// all the conencted children Link's are moved as a result of the
      /// Joint angle setting.  Dynamic Joint angle update is accomplished
      /// by calling JointController::SetJointPosition.
      public: void SetAngle(int _index, math::Angle _angle);

      /// Get the forces applied to the center of mass of a physics::Link
      /// due to the existence of this Joint.
      /// \param index The index of the link(0 or 1)
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.  E.g.  if you are using
      /// metric units, the unit for force is Newtons.  If using
      /// imperial units (sorry), then unit of force is lb-force
      /// not (lb-mass), etc.
      public: virtual math::Vector3 GetLinkForce(unsigned int index) const = 0;

      /// \brief Get the torque applied to the center of mass of a physics::Link
      /// due to the existence of this Joint.
      /// \param index The index of the link(0 or 1)
      /// Note that the unit of torque should be consistent with the rest
      /// of the simulation scales.  E.g.  if you are using
      /// metric units, the unit for force is Newtons-Meters.  If using
      /// imperial units (sorry), then unit of force is lb-force-inches
      /// not (lb-mass-inches), etc.
      public: virtual math::Vector3 GetLinkTorque(unsigned int index) const = 0;

      /// \brief Set a parameter for the joint
      public: virtual void SetAttribute(Attribute, int index, double value) = 0;

      /// \brief Get the child link
      public: LinkPtr GetChild() const;

      /// \brief Get the child link
      public: LinkPtr GetParent() const;

      /// Fill a joint message
      public: void FillJointMsg(msgs::Joint &_msg);

      protected: virtual math::Angle GetAngleImpl(int _index) const = 0;

      private: void LoadImpl(const math::Pose &_origin);

      /// The first link this joint connects to
      protected: LinkPtr childLink;

      /// The second link this joint connects to
      protected: LinkPtr parentLink;

      protected: std::string visual;
      protected: std::string line1;
      protected: std::string line2;
      protected: bool showJoints;

      protected: ModelPtr model;

      protected: math::Vector3 anchorPos;
      protected: LinkPtr anchorLink;

      private: event::EventT<void ()> jointUpdate;
      private: event::ConnectionPtr showJointsConnection;

      // joint damping_coefficient
      protected: double damping_coefficient;

      private: math::Angle staticAngle;
    };
    /// \}
  }
}
#endif



