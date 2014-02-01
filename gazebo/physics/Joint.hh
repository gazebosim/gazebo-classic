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
/* Desc: The base joint class
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef _JOINT_HH_
#define _JOINT_HH_

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
    class Joint : public Base
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
      public: virtual LinkPtr GetJointLink(int _index) const = 0;

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
      public: virtual void SetAxis(int _index, const math::Vector3 &_axis) = 0;

      /// \brief Set the joint damping.
      /// \param[in] _index Index of the axis to set, currently ignored, to be
      ///                   implemented.
      /// \param[in] _damping Damping value for the axis.
      public: virtual void SetDamping(int _index, double _damping) = 0;

      /// \brief Returns the current joint damping coefficient.
      /// \param[in] _index Index of the axis to get, currently ignored, to be
      ///                   implemented.
      /// \return Joint viscous damping coefficient for this joint.
      public: double GetDamping(int _index);

      /// \brief Callback to apply damping force to joint.
      public: virtual void ApplyDamping();

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
      public: math::Vector3 GetLocalAxis(int _index) const;

      /// \brief Get the axis of rotation in global cooridnate frame.
      /// \param[in] _index Index of the axis to get.
      /// \return Axis value for the provided index.
      public: virtual math::Vector3 GetGlobalAxis(int _index) const = 0;

      /// \brief Set the anchor point.
      /// \param[in] _index Indx of the axis.
      /// \param[in] _anchor Anchor value.
      public: virtual void SetAnchor(int _index,
                                     const math::Vector3 &_anchor) = 0;

      /// \brief Get the anchor point.
      /// \param[in] _index Index of the axis.
      /// \return Anchor value for the axis.
      public: virtual math::Vector3 GetAnchor(int _index) const = 0;

      /// \brief Set the high stop of an axis(index).
      /// \param[in] _index Index of the axis.
      /// \param[in] _angle High stop angle.
      public: virtual void SetHighStop(int _index,
                                       const math::Angle &_angle);

      /// \brief Set the low stop of an axis(index).
      /// \param[in] _index Index of the axis.
      /// \param[in] _angle Low stop angle.
      public: virtual void SetLowStop(int _index,
                                      const math::Angle &_angle);

      /// \brief Get the high stop of an axis(index).
      /// This function is replaced by GetUpperLimit(unsigned int).
      /// If you are interested in getting the value of dParamHiStop*,
      /// use GetAttribute(hi_stop, _index)
      /// \param[in] _index Index of the axis.
      /// \return Angle of the high stop value.
      public: virtual math::Angle GetHighStop(int _index) = 0;

      /// \brief Get the low stop of an axis(index).
      /// This function is replaced by GetLowerLimit(unsigned int).
      /// If you are interested in getting the value of dParamHiStop*,
      /// use GetAttribute(hi_stop, _index)
      /// \param[in] _index Index of the axis.
      /// \return Angle of the low stop value.
      public: virtual math::Angle GetLowStop(int _index) = 0;

      /// \brief Get the effort limit on axis(index).
      /// \param[in] _index Index of axis, where 0=first axis and 1=second axis
      /// \return Effort limit specified in SDF
      public: virtual double GetEffortLimit(int _index);

      /// \brief Get the velocity limit on axis(index).
      /// \param[in] _index Index of axis, where 0=first axis and 1=second axis
      /// \return Velocity limit specified in SDF
      public: virtual double GetVelocityLimit(int _index);

      /// \brief Set the velocity of an axis(index).
      /// \param[in] _index Index of the axis.
      /// \param[in] _vel Velocity.
      public: virtual void SetVelocity(int _index, double _vel) = 0;

      /// \brief Get the rotation rate of an axis(index)
      /// \param[in] _index Index of the axis.
      /// \return The rotaional velocity of the joint axis.
      public: virtual double GetVelocity(int _index) const = 0;

      /// \brief Set the force applied to this physics::Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.  Force is additive (multiple calls
      /// to SetForce to the same joint in the same time
      /// step will accumulate forces on that Joint).
      /// Forces are truncated by effortLimit before applied.
      /// \param[in] _index Index of the axis.
      /// \param[in] _effort Force value.
      public: virtual void SetForce(int _index, double _effort) = 0;

      /// \brief check if the force against velocityLimit and effortLimit,
      /// truncate if necessary.
      /// \param[in] _index Index of the axis.
      /// \param[in] _effort Force value.
      /// \return truncated effort
      public: double CheckAndTruncateForce(int _index, double _effort);

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

      /// \brief Set the max allowed force of an axis(index).
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Maximum force that can be applied to the axis.
      public: virtual void SetMaxForce(int _index, double _force) = 0;

      /// \brief Get the max allowed force of an axis(index).
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.
      /// \param[in] _index Index of the axis.
      /// \return The maximum force.
      public: virtual double GetMaxForce(int _index) = 0;

      /// \brief Get the angle of rotation of an axis(index)
      /// \param[in] _index Index of the axis.
      /// \return Angle of the axis.
      public: math::Angle GetAngle(int _index) const;

      /// \brief Get the angle count.
      /// \return The number of DOF for the joint.
      public: virtual unsigned int GetAngleCount() const = 0;

      /// \brief If the Joint is static, Gazebo stores the state of
      /// this Joint as a scalar inside the Joint class, so
      /// this call will NOT move the joint dynamically for a static Model.
      /// But if this Model is not static, then it is updated dynamically,
      /// all the conencted children Link's are moved as a result of the
      /// Joint angle setting.  Dynamic Joint angle update is accomplished
      /// by calling JointController::SetJointPosition.
      /// \param[in] _index Index of the axis.
      /// \param[in] _angle Angle to set the joint to.
      public: void SetAngle(int _index, math::Angle _angle);

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
      /// \param[in] _key String key.
      /// \param[in] _index Index of the axis.
      /// \param[in] _value Value of the attribute.
      public: virtual void SetAttribute(const std::string &_key, int _index,
                                        const boost::any &_value) = 0;

      /// \brief Get a non-generic parameter for the joint.
      /// \param[in] _key String key.
      /// \param[in] _index Index of the axis.
      /// \param[in] _value Value of the attribute.
      public: virtual double GetAttribute(const std::string &_key,
                                                unsigned int _index) = 0;

      /// \brief Get the child link
      /// \return Pointer to the child link.
      public: LinkPtr GetChild() const;

      /// \brief Get the parent link.
      /// \return Pointer to the parent link.
      public: LinkPtr GetParent() const;

      /// \brief Fill a joint message.
      /// \param[out] _msg Message to fill with this joint's properties.
      public: void FillMsg(msgs::Joint &_msg);

      /// \brief Accessor to inertia ratio across this joint.
      /// \param[in] _index Joint axis index.
      /// \return returns the inertia ratio across specified joint axis.
      public: double GetInertiaRatio(unsigned int _index) const;

      /// \brief:  get the joint upper limit
      /// (replaces GetLowStop and GetHighStop)
      /// \param[in] _index Index of the axis.
      /// \return Upper limit of the axis.
      public: math::Angle GetLowerLimit(unsigned int _index) const;

      /// \brief:  get the joint lower limit
      /// (replacee GetLowStop and GetHighStop)
      /// \param[in] _index Index of the axis.
      /// \return Upper limit of the axis.
      public: math::Angle GetUpperLimit(unsigned int _index) const;

      /// \brief Set whether the joint should generate feedback.
      /// \param[in] _enable True to enable joint feedback.
      public: virtual void SetProvideFeedback(bool _enable);

      /// \brief Cache Joint Force Torque Values if necessary for physics engine
      public: virtual void CacheForceTorque() { }

      /// \brief Get damping coefficient of this joint
      /// \return viscous joint damping coefficient
      public: double GetDampingCoefficient() const;

      /// \brief Get initial Anchor Pose specified by model
      /// <joint><pose>...</pose></joint>
      /// \return Joint::anchorPose, initial joint anchor pose.
      public: math::Pose GetInitialAnchorPose();

      /// \brief Get the angle of an axis helper function.
      /// \param[in] _index Index of the axis.
      /// \return Angle of the axis.
      protected: virtual math::Angle GetAngleImpl(int _index) const = 0;

      /// \brief Helper function to load a joint.
      /// \param[in] _pose Pose of the anchor.
      private: void LoadImpl(const math::Pose &_pose);

      /// \brief Computes inertiaRatio for this joint during Joint::Init
      /// The inertia ratio for each joint between [1, +inf] gives a sense
      /// of how well this model will perform in iterative LCP methods.
      private: void ComputeInertiaRatio();

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

      /// \brief Anchor link.
      protected: LinkPtr anchorLink;

      /// \brief joint dampingCoefficient
      protected: double dampingCoefficient;

      /// \brief apply damping for adding viscous damping forces on updates
      protected: gazebo::event::ConnectionPtr applyDamping;

      /// \brief Store Joint effort limit as specified in SDF
      protected: double effortLimit[MAX_JOINT_AXIS];

      /// \brief Store Joint velocity limit as specified in SDF
      protected: double velocityLimit[MAX_JOINT_AXIS];

      /// \brief Store Joint inertia ratio.  This is a measure of how well
      /// this model behaves using interative LCP solvers.
      protected: double inertiaRatio[MAX_JOINT_AXIS];

      /// \brief Store Joint position lower limit as specified in SDF
      protected: math::Angle lowerLimit[MAX_JOINT_AXIS];

      /// \brief Store Joint position upper limit as specified in SDF
      protected: math::Angle upperLimit[MAX_JOINT_AXIS];

      /// \brief Cache Joint force torque values in case physics engine
      /// clears them at the end of update step.
      protected: JointWrench wrench;

      /// \brief option to use CFM damping
      protected: bool useCFMDamping;

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

      // Added in 2.1 down here to preserve ABI
      /// \brief Set the effort limit on a joint axis.
      /// \param[in] _index Index of the axis to set.
      /// \param[in] _effort Effort limit for the axis.
      public: virtual void SetEffortLimit(unsigned int _index, double _effort);
    };
    /// \}
  }
}
#endif
