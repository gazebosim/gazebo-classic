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
#ifndef _GAZEBO_JOINT_HH_
#define _GAZEBO_JOINT_HH_

#include <string>
#include <vector>

#include <boost/any.hpp>

#include "gazebo/common/Event.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/math/Angle.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/physics/JointState.hh"
#include "gazebo/physics/Base.hh"
#include "gazebo/physics/JointWrench.hh"
#include "gazebo/util/system.hh"

/// \brief maximum number of axis per joint anticipated.
/// Currently, this is 2 as 3-axis joints (e.g. ball)
/// actuation, control is not there yet.
#define MAX_JOINT_AXIS 2

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class Joint Joint.hh physics/physics.hh
    /// \brief Base class for all joints
    class GAZEBO_VISIBLE Joint : public Base
    {
      /// \enum Attribute
      /// \brief Joint attribute types.
      public: enum Attribute
              {
                /// \brief Fudge factor.
                FUDGE_FACTOR,

                /// \brief Suspension error reduction parameter.
                SUSPENSION_ERP,

                /// \brief Suspension constraint force mixing.
                SUSPENSION_CFM,

                /// \brief Stop limit error reduction parameter.
                STOP_ERP,

                /// \brief Stop limit constraint force mixing.
                STOP_CFM,

                /// \brief Error reduction parameter.
                ERP,

                /// \brief Constraint force mixing.
                CFM,

                /// \brief Maximum force.
                FMAX,

                /// \brief Velocity.
                VEL,

                /// \brief High stop angle.
                HI_STOP,

                /// \brief Low stop angle.
                LO_STOP
              };

      /// \brief Constructor
      /// \param[in] Joint parent
      public: explicit Joint(BasePtr _parent);

      /// \brief Destructor
      public: virtual ~Joint();

      /// \brief Set pose, parent and child links of a physics::Joint
      /// \param[in] _parent Parent link.
      /// \param[in] _child Child link.
      /// \param[in] _pose Pose containing Joint Anchor offset from child link.
      public: void Load(LinkPtr _parent, LinkPtr _child,
                        const math::Pose &_pose);

      /// \brief Load physics::Joint from a SDF sdf::Element.
      /// \param[in] _sdf SDF values to load from.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize a joint.
      public: virtual void Init();

      /// \brief Finialize the object
      public: virtual void Fini();

      /// \brief Update the joint.
      public: void Update();

      /// \brief Update the parameters using new sdf values.
      /// \param[in] _sdf SDF values to update from.
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Reset the joint.
      public: virtual void Reset();

      /// \brief Set the joint state.
      /// \param[in] _state Joint state
      public: void SetState(const JointState &_state);

      /// \brief Set the model this joint belongs too.
      /// \param[in] _model Pointer to a model.
      public: void SetModel(ModelPtr _model);

      /// \brief Get the link to which the joint is attached according
      /// the _index.
      /// \param[in] _index Index of the link to retreive.
      /// \return Pointer to the request link. NULL if the index was
      /// invalid.
      public: virtual LinkPtr GetJointLink(unsigned int _index) const = 0;

      /// \brief Determines of the two bodies are connected by a joint.
      /// \param[in] _one First link.
      /// \param[in] _two Second link.
      /// \return True if the two links are connected by a joint.
      public: virtual bool AreConnected(LinkPtr _one, LinkPtr _two) const = 0;

      /// \brief Attach the two bodies with this joint.
      /// \param[in] _parent Parent link.
      /// \param[in] _child Child link.
      public: virtual void Attach(LinkPtr _parent, LinkPtr _child);

      /// \brief Detach this joint from all links.
      public: virtual void Detach();

      /// \brief Set the axis of rotation where axis is specified in local
      /// joint frame.
      /// \param[in] _index Index of the axis to set.
      /// \param[in] _axis Vector in local joint frame of axis direction
      ///                  (must have length greater than zero).
      public: virtual void SetAxis(unsigned int _index,
                  const math::Vector3 &_axis) = 0;

      /// \brief Set the joint damping.
      /// \param[in] _index Index of the axis to set, currently ignored, to be
      ///                   implemented.
      /// \param[in] _damping Damping value for the axis.
      public: virtual void SetDamping(unsigned int _index, double _damping) = 0;

      /// \brief Returns the current joint damping coefficient.
      /// \param[in] _index Index of the axis to get, currently ignored, to be
      ///                   implemented.
      /// \return Joint viscous damping coefficient for this joint.
      public: double GetDamping(unsigned int _index);

      /// \brief Callback to apply spring stiffness and viscous damping
      /// effects to joint.
      /// \TODO: rename to ApplySpringStiffnessDamping()
      public: virtual void ApplyStiffnessDamping();

      /// \brief Set the joint spring stiffness.
      /// \param[in] _index Index of the axis to set, currently ignored, to be
      ///                   implemented.
      /// \param[in] _stiffness Stiffness value for the axis.
      /// \param[in] _reference Spring zero load reference position.
      /// \TODO: rename to SetSpringStiffnessDamping()
      public: virtual void SetStiffnessDamping(unsigned int _index,
        double _stiffness, double _damping, double _reference = 0) = 0;

      /// \brief Set the joint spring stiffness.
      /// \param[in] _index Index of the axis to set, currently ignored, to be
      ///                   implemented.
      /// \param[in] _stiffness Spring stiffness value for the axis.
      /// \TODO: rename to SetSpringStiffness()
      public: virtual void SetStiffness(unsigned int _index,
                                        double _stiffness) = 0;

      /// \brief Returns the current joint spring stiffness coefficient.
      /// \param[in] _index Index of the axis to get, currently ignored, to be
      ///                   implemented.
      /// \return Joint spring stiffness coefficient for this joint.
      /// \TODO: rename to GetSpringStiffness()
      public: double GetStiffness(unsigned int _index);

      /// \brief Get joint spring reference position.
      /// \param[in] _index Index of the axis to get.
      /// \return Joint spring reference position
      /// (in radians for angular joints).
      public: double GetSpringReferencePosition(unsigned int _index) const;

      /// \brief Connect a boost::slot the the joint update signal.
      /// \param[in] _subscriber Callback for the connection.
      /// \return Connection pointer, which must be kept in scope.
      public: template<typename T>
              event::ConnectionPtr ConnectJointUpdate(T _subscriber)
              {return jointUpdate.Connect(_subscriber);}

      /// \brief Disconnect a boost::slot the the joint update signal.
      /// \param[in] _conn Connection to disconnect.
      public: void DisconnectJointUpdate(event::ConnectionPtr &_conn)
              {jointUpdate.Disconnect(_conn);}

      /// \brief Get the axis of rotation.
      /// \param[in] _index Index of the axis to get.
      /// \return Axis value for the provided index.
      public: math::Vector3 GetLocalAxis(unsigned int _index) const;

      /// \brief Get the axis of rotation in global cooridnate frame.
      /// \param[in] _index Index of the axis to get.
      /// \return Axis value for the provided index.
      public: virtual math::Vector3 GetGlobalAxis(
                  unsigned int _index) const = 0;

      /// \brief Set the anchor point.
      /// \param[in] _index Indx of the axis.
      /// \param[in] _anchor Anchor value.
      public: virtual void SetAnchor(unsigned int _index,
                                     const math::Vector3 &_anchor) = 0;

      /// \brief Get the anchor point.
      /// \param[in] _index Index of the axis.
      /// \return Anchor value for the axis.
      public: virtual math::Vector3 GetAnchor(unsigned int _index) const = 0;

      /// \brief Set the high stop of an axis(index).
      /// \param[in] _index Index of the axis.
      /// \param[in] _angle High stop angle.
      public: virtual bool SetHighStop(unsigned int _index,
                                       const math::Angle &_angle);

      /// \brief Set the low stop of an axis(index).
      /// \param[in] _index Index of the axis.
      /// \param[in] _angle Low stop angle.
      public: virtual bool SetLowStop(unsigned int _index,
                                      const math::Angle &_angle);

      /// \brief Get the high stop of an axis(index).
      /// This function is replaced by GetUpperLimit(unsigned int).
      /// If you are interested in getting the value of dParamHiStop*,
      /// use GetAttribute(hi_stop, _index)
      /// \param[in] _index Index of the axis.
      /// \return Angle of the high stop value.
      public: virtual math::Angle GetHighStop(unsigned int _index) = 0;

      /// \brief Get the low stop of an axis(index).
      /// This function is replaced by GetLowerLimit(unsigned int).
      /// If you are interested in getting the value of dParamHiStop*,
      /// use GetAttribute(hi_stop, _index)
      /// \param[in] _index Index of the axis.
      /// \return Angle of the low stop value.
      public: virtual math::Angle GetLowStop(unsigned int _index) = 0;

      /// \brief Get the effort limit on axis(index).
      /// \param[in] _index Index of axis, where 0=first axis and 1=second axis
      /// \return Effort limit specified in SDF
      public: virtual double GetEffortLimit(unsigned int _index);

      /// \brief Set the effort limit on a joint axis.
      /// \param[in] _index Index of the axis to set.
      /// \param[in] _effort Effort limit for the axis.
      public: virtual void SetEffortLimit(unsigned int _index, double _effort);

      /// \brief Get the velocity limit on axis(index).
      /// \param[in] _index Index of axis, where 0=first axis and 1=second axis
      /// \return Velocity limit specified in SDF
      public: virtual double GetVelocityLimit(unsigned int _index);

      /// \brief Set the velocity limit on a joint axis.
      /// \param[in] _index Index of the axis to set.
      /// \param[in] _velocity Velocity limit for the axis.
      public: virtual void SetVelocityLimit(unsigned int _index,
                                                  double _velocity);

      /// \brief Set the velocity of an axis(index).
      /// In ODE and Bullet, the SetVelocityMaximal function is used to
      /// set the velocity of the child link relative to the parent.
      /// In Simbody and DART, this function updates the velocity state,
      /// which has a recursive effect on the rest of the chain.
      /// \param[in] _index Index of the axis.
      /// \param[in] _vel Velocity.
      public: virtual void SetVelocity(unsigned int _index, double _vel) = 0;

      /// \brief Get the rotation rate of an axis(index)
      /// \param[in] _index Index of the axis.
      /// \return The rotaional velocity of the joint axis.
      public: virtual double GetVelocity(unsigned int _index) const = 0;

      /// \brief Set the force applied to this physics::Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.  Force is additive (multiple calls
      /// to SetForce to the same joint in the same time
      /// step will accumulate forces on that Joint).
      /// Forces are truncated by effortLimit before applied.
      /// \param[in] _index Index of the axis.
      /// \param[in] _effort Force value.
      public: virtual void SetForce(unsigned int _index, double _effort) = 0;

      /// \brief check if the force against velocityLimit and effortLimit,
      /// truncate if necessary.
      /// \param[in] _index Index of the axis.
      /// \param[in] _effort Force value.
      /// \return truncated effort
      public: double CheckAndTruncateForce(unsigned int _index, double _effort);

      /// \brief @todo: not yet implemented.
      /// Get external forces applied at this Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.
      /// \param[in] _index Index of the axis.
      /// \return The force applied to an axis.
      public: virtual double GetForce(unsigned int _index);

      /// \brief get internal force and torque values at a joint.
      ///
      ///   The force and torque values are returned in  a JointWrench
      ///   data structure.  Where JointWrench.body1Force contains the
      ///   force applied by the parent Link on the Joint specified in
      ///   the parent Link frame, and JointWrench.body2Force contains
      ///   the force applied by the child Link on the Joint specified
      ///   in the child Link frame.  Note that this sign convention
      ///   is opposite of the reaction forces of the Joint on the Links.
      ///
      ///   FIXME TODO: change name of this function to something like:
      ///     GetNegatedForceTorqueInLinkFrame
      ///   and make GetForceTorque call return non-negated reaction forces
      ///   in perspective Link frames.
      ///
      ///   Note that for ODE you must set
      ///     <provide_feedback>true<provide_feedback>
      ///   in the joint sdf to use this.
      ///
      /// \param[in] _index Not used right now
      /// \return The force and torque at the joint, see above for details
      /// on conventions.
      public: virtual JointWrench GetForceTorque(unsigned int _index) = 0;

      /// \brief Set the max allowed force of an axis(index) when using
      /// Joint::SetVelocity.
      /// Current implementation in Bullet and ODE is enforced using impulses,
      /// which enforces force/torque limits when calling Joint::SetVelocity.
      /// Current implementation is engine dependent. See for example
      /// ODE implementation in ODEHingeJoint::SetMaxForce.
      /// Note this functionality is not implemented in DART and Simbody.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Maximum force that can be applied to the axis.
      public: virtual void SetMaxForce(unsigned int _index, double _force)
              GAZEBO_DEPRECATED(5.0) = 0;

      /// \brief Get the max allowed force of an axis(index) when using
      /// Joint::SetVelocity.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.
      /// \param[in] _index Index of the axis.
      /// \return The maximum force.
      public: virtual double GetMaxForce(unsigned int _index)
              GAZEBO_DEPRECATED(5.0) = 0;

      /// \brief Get the angle of rotation of an axis(index)
      /// \param[in] _index Index of the axis.
      /// \return Angle of the axis.
      public: math::Angle GetAngle(unsigned int _index) const;

      /// \brief Get the angle count.
      /// \return The number of DOF for the joint.
      public: virtual unsigned int GetAngleCount() const = 0;

      /// \brief If the Joint is static, Gazebo stores the state of
      /// this Joint as a scalar inside the Joint class, so
      /// this call will NOT move the joint dynamically for a static Model.
      /// But if this Model is not static, then it is updated dynamically.
      /// The child links of this joint are updated based on position change.
      /// And all the links connected to the child link of this joint
      /// except through the parent link of this joint moves with the child
      /// link.
      /// \param[in] _index Index of the axis.
      /// \param[in] _angle Angle to set the joint to.
      public: void SetAngle(unsigned int _index, math::Angle _angle)
              GAZEBO_DEPRECATED(4.0);

      /// \brief The child links of this joint are updated based on desired
      /// position.  And all the links connected to the child link of this joint
      /// except through the parent link of this joint moves with the child
      /// link.
      /// \param[in] _index Index of the joint axis (degree of freedom).
      /// \param[in] _position Position to set the joint to.
      /// unspecified, pure kinematic teleportation.
      /// \return returns true if operation succeeds, false if it fails.
      public: virtual bool SetPosition(unsigned int _index, double _position);

      /// \brief Helper function for maximal coordinate solver SetPosition.
      /// The child links of this joint are updated based on position change.
      /// And all the links connected to the child link of this joint
      /// except through the parent link of this joint moves with the child
      /// link.
      /// \param[in] _index Index of the joint axis (degree of freedom).
      /// \param[in] _position Position to set the joint to.
      /// \return returns true if operation succeeds, false if it fails.
      protected: bool SetPositionMaximal(unsigned int _index, double _position);

      /// \brief Helper function for maximal coordinate solver SetVelocity.
      /// The velocity of the child link of this joint is updated relative
      /// to the current parent velocity.
      /// It currently does not act recursively.
      /// \param[in] _index Index of the joint axis (degree of freedom).
      /// \param[in] _velocity Velocity to set at this joint.
      /// \return returns true if operation succeeds, false if it fails.
      protected: bool SetVelocityMaximal(unsigned int _index, double _velocity);

      /// \brief Get the forces applied to the center of mass of a physics::Link
      /// due to the existence of this Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.
      /// \param[in] index The index of the link(0 or 1).
      /// \return Force applied to the link.
      public: virtual math::Vector3 GetLinkForce(unsigned int _index) const = 0;

      /// \brief Get the torque applied to the center of mass of a physics::Link
      /// due to the existence of this Joint.
      /// Note that the unit of torque should be consistent with the rest
      /// of the simulation scales.
      /// \param[in] index The index of the link(0 or 1)
      /// \return Torque applied to the link.
      public: virtual math::Vector3 GetLinkTorque(
                  unsigned int _index) const = 0;

      /// \brief Set a non-generic parameter for the joint.
      /// replaces SetAttribute(Attribute, int, double)
      /// List of parameters:
      ///  "friction"     Axis Coulomb joint friction coefficient.
      ///  "hi_stop"      Axis upper limit.
      ///  "lo_stop"      Axis lower limit.
      /// \param[in] _key String key.
      /// \param[in] _index Index of the axis.
      /// \param[in] _value Value of the attribute.
      public: virtual bool SetParam(const std::string &_key,
                                    unsigned int _index,
                                    const boost::any &_value) = 0;

      /// \brief Get a non-generic parameter for the joint.
      /// \sa SetParam(const std::string &, unsigned int, const boost::any)
      /// \param[in] _key String key.
      /// \param[in] _index Index of the axis.
      public: virtual double GetParam(const std::string &_key,
                                      unsigned int _index) = 0;

      /// \brief Get the child link
      /// \return Pointer to the child link.
      public: LinkPtr GetChild() const;

      /// \brief Get the parent link.
      /// \return Pointer to the parent link.
      public: LinkPtr GetParent() const;

      /// \brief Get the joint type as msgs::Joint::Type.
      /// \return Joint type.
      public: msgs::Joint::Type GetMsgType() const;

      /// \brief Fill a joint message.
      /// \param[out] _msg Message to fill with this joint's properties.
      public: void FillMsg(msgs::Joint &_msg);

      /// \brief Computes moment of inertia (MOI) across a specified joint axis.
      /// The ratio is given in the form of MOI_chidl / MOI_parent.
      /// If MOI_parent is zero, this funciton will return 0.
      /// The inertia ratio for each joint axis indicates the sensitivity
      /// of the joint to actuation torques.
      /// \param[in] _index axis number about which MOI ratio is computed.
      /// \return ratio of child MOI to parent MOI.
      public: double GetInertiaRatio(const unsigned int _index) const;

      /// \brief Computes moment of inertia (MOI) across an arbitrary axis
      /// specified in the world frame.
      /// The ratio is given in the form of MOI_chidl / MOI_parent.
      /// If MOI_parent is zero, this funciton will return 0.
      /// The moment of inertia ratio along constrained directions of a joint
      /// has an impact on the performance of Projected Gauss Seidel (PGS)
      /// iterative LCP methods.
      /// \param[in] _axis axis in world frame for which MOI ratio is computed.
      /// \return ratio of child MOI to parent MOI.
      public: double GetInertiaRatio(const math::Vector3 &_axis) const;

      /// \brief:  get the joint upper limit
      /// (replaces GetLowStop and GetHighStop)
      /// \param[in] _index Index of the axis.
      /// \return Lower limit of the axis.
      public: math::Angle GetLowerLimit(unsigned int _index) const;

      /// \brief:  get the joint lower limit
      /// (replacee GetLowStop and GetHighStop)
      /// \param[in] _index Index of the axis.
      /// \return Upper limit of the axis.
      public: math::Angle GetUpperLimit(unsigned int _index) const;

      /// \brief:  set the joint upper limit
      /// (replaces SetLowStop and SetHighStop)
      /// \param[in] _index Index of the axis.
      /// \param[in] _limit Lower limit of the axis.
      public: void SetLowerLimit(unsigned int _index, math::Angle _limit);

      /// \brief:  set the joint lower limit
      /// (replacee GetLowStop and GetHighStop)
      /// \param[in] _index Index of the axis.
      /// \param[in] _limit Upper limit of the axis.
      public: void SetUpperLimit(unsigned int _index, math::Angle _limit);

      /// \brief Set whether the joint should generate feedback.
      /// \param[in] _enable True to enable joint feedback.
      public: virtual void SetProvideFeedback(bool _enable);

      /// \brief Cache Joint Force Torque Values if necessary for physics engine
      public: virtual void CacheForceTorque();

      /// \brief Set joint stop stiffness.
      /// \param[in] _index joint axis index.
      /// \param[in] _stiffness joint stop stiffness coefficient.
      public: void SetStopStiffness(unsigned int _index, double _stiffness);

      /// \brief Set joint stop dissipation.
      /// \param[in] _index joint axis index.
      /// \param[in] _dissipation joint stop dissipation coefficient.
      public: void SetStopDissipation(unsigned int _index, double _dissipation);

      /// \brief Get joint stop stiffness.
      /// \param[in] _index joint axis index.
      /// \return joint stop stiffness coefficient.
      public: double GetStopStiffness(unsigned int _index) const;

      /// \brief Get joint stop dissipation.
      /// \param[in] _index joint axis index.
      /// \return joint stop dissipation coefficient.
      public: double GetStopDissipation(unsigned int _index) const;

      /// \brief Get initial Anchor Pose specified by model
      /// <joint><pose>...</pose></joint>
      /// \return Joint::anchorPose, initial joint anchor pose.
      public: math::Pose GetInitialAnchorPose() const;

      /// \brief Get pose of joint frame relative to world frame.
      /// Note that the joint frame is defined with a fixed offset from
      /// the child link frame.
      /// \return Pose of joint frame relative to world frame.
      public: math::Pose GetWorldPose() const;

      /// \brief Get anchor pose on parent link relative to world frame.
      /// When there is zero joint error, this should match the value
      /// returned by Joint::GetWorldPose() for the constrained degrees
      /// of freedom.
      /// \return Anchor pose on parent link in world frame.
      public: math::Pose GetParentWorldPose() const;

      /// \brief Get pose offset between anchor pose on child and parent,
      /// expressed in the parent link frame. This can be used to compute
      /// the bilateral constraint error.
      /// \return Pose offset between anchor pose on child and parent,
      /// in parent link frame.
      public: math::Pose GetAnchorErrorPose() const;

      /// \brief Get orientation of reference frame for specified axis,
      /// relative to world frame. The value of axisParentModelFrame
      /// is used to determine the appropriate frame.
      /// \param[in] _index joint axis index.
      /// \return Orientation of axis frame relative to world frame.
      public: math::Quaternion GetAxisFrame(unsigned int _index) const;

      /// \brief Get orientation of joint axis reference frame
      /// relative to joint frame. This should always return identity unless
      /// flag use_parent_model_frame is true in sdf 1.5 or using sdf 1.4-,
      /// i.e. bug described in issue #494 is present.
      /// In addition, if use_parent_model_frame is true, and the parent
      /// link of the joint is world, the axis is defined in the world frame.
      /// The value of axisParentModelFrame is used to determine the
      /// appropriate frame internally.
      /// \param[in] _index joint axis index.
      /// \return Orientation of axis frame relative to joint frame.
      /// If supplied _index is out of range, or use_parent_model_frame
      /// is not true, this function returns identity rotation quaternion.
      public: math::Quaternion GetAxisFrameOffset(unsigned int _index) const;

      /// \brief Returns this joint's spring potential energy,
      /// based on the reference position of the spring.
      /// If using metric system, the unit of energy will be Joules.
      /// \return this joint's spring potential energy,
      public: double GetWorldEnergyPotentialSpring(unsigned int _index) const;

      /// \brief Get the angle of an axis helper function.
      /// \param[in] _index Index of the axis.
      /// \return Angle of the axis.
      protected: virtual math::Angle GetAngleImpl(
                     unsigned int _index) const = 0;

      /// \brief internal helper to find all links connected to the child link
      /// branching out from the children of the child link and any parent
      /// of the child link other than the parent link through this joint.
      /// \param[in] _originalParentLink parent link of this joint, this
      /// is used to check for loops connecting back to the parent link.
      /// \param[in/out] _connectedLinks list of aggregated links that
      /// contains all links connected to the child link of this joint.
      /// Empty if a loop is found that connects back to the parent link.
      /// \return true if successful, false if a loop is found that connects
      /// back to the parent link.
      protected: bool FindAllConnectedLinks(const LinkPtr &_originalParentLink,
        Link_V &_connectedLinks);

      /// \brief internal function to help us compute child link pose
      /// if a joint position change is applied.
      /// \param[in] _index axis index
      /// \param[in] _position new joint position
      /// \return new child link pose at new joint position.
      protected: math::Pose ComputeChildLinkPose(unsigned int _index,
          double _position);

      /// \brief Helper function to load a joint.
      /// \param[in] _pose Pose of the anchor.
      private: void LoadImpl(const math::Pose &_pose);

      /// \brief The first link this joint connects to
      protected: LinkPtr childLink;

      /// \brief The second link this joint connects to
      protected: LinkPtr parentLink;

      /// \brief Pointer to the parent model.
      protected: ModelPtr model;

      /// \brief Anchor pose.  This is the xyz offset of the joint frame from
      /// child frame specified in the parent link frame
      protected: math::Vector3 anchorPos;

      /// \brief Anchor pose specified in SDF <joint><pose> tag.
      /// AnchorPose is the transform from child link frame to joint frame
      /// specified in the child link frame.
      /// AnchorPos is more relevant in normal usage, but sometimes,
      /// we do need this (e.g. GetForceTorque and joint visualization).
      protected: math::Pose anchorPose;

      /// \brief Anchor pose relative to parent link frame.
      protected: math::Pose parentAnchorPose;

      /// \brief Anchor link.
      protected: LinkPtr anchorLink;

      /// \brief joint viscous damping coefficient
      protected: double dissipationCoefficient[MAX_JOINT_AXIS];

      /// \brief joint stiffnessCoefficient
      protected: double stiffnessCoefficient[MAX_JOINT_AXIS];

      /// \brief joint spring reference (zero load) position
      protected: double springReferencePosition[MAX_JOINT_AXIS];

      /// \brief apply damping for adding viscous damping forces on updates
      protected: gazebo::event::ConnectionPtr applyDamping;

      /// \brief Store Joint effort limit as specified in SDF
      protected: double effortLimit[MAX_JOINT_AXIS];

      /// \brief Store Joint velocity limit as specified in SDF
      protected: double velocityLimit[MAX_JOINT_AXIS];

      /// \brief Store Joint position lower limit as specified in SDF
      protected: math::Angle lowerLimit[MAX_JOINT_AXIS];

      /// \brief Store Joint position upper limit as specified in SDF
      protected: math::Angle upperLimit[MAX_JOINT_AXIS];

      /// \brief Cache Joint force torque values in case physics engine
      /// clears them at the end of update step.
      protected: JointWrench wrench;

      /// \brief Flags that are set to true if an axis value is expressed
      /// in the parent model frame. Otherwise use the joint frame.
      /// See issue #494.
      protected: bool axisParentModelFrame[MAX_JOINT_AXIS];

      /// \brief An SDF pointer that allows us to only read the joint.sdf
      /// file once, which in turns limits disk reads.
      private: static sdf::ElementPtr sdfJoint;

      /// \brief Provide Feedback data for contact forces
      protected: bool provideFeedback;

      /// \brief Names of all the sensors attached to the link.
      private: std::vector<std::string> sensors;

      /// \brief Joint update event.
      private: event::EventT<void ()> jointUpdate;

      /// \brief Angle used when the joint is parent of a static model.
      private: math::Angle staticAngle;

      /// \brief Joint stop stiffness
      private: double stopStiffness[MAX_JOINT_AXIS];

      /// \brief Joint stop dissipation
      private: double stopDissipation[MAX_JOINT_AXIS];
    };
    /// \}
  }
}
#endif
