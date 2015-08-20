/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
/* Desc: Link class
 * Author: Nate Koenig
 */

#ifndef _LINK_HH_
#define _LINK_HH_

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <map>
#include <vector>
#include <string>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/util/UtilTypes.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/CommonTypes.hh"

#include "gazebo/physics/LinkState.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/Inertial.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace util
  {
    class OpenALSource;
    class OpenALSink;
  }

  namespace physics
  {
    class Model;
    class Collision;
    class Battery;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class Link Link.hh physics/physics.hh
    /// \brief Link class defines a rigid body entity, containing
    /// information on inertia, visual and collision properties of
    /// a rigid body.
    class GZ_PHYSICS_VISIBLE Link : public Entity
    {
      /// \brief Constructor
      /// \param[in] _parent Parent of this link.
      public: explicit Link(EntityPtr _parent);

      /// \brief Destructor.
      public: virtual ~Link();

      /// \brief Load the body based on an SDF element.
      /// \param[in] _sdf SDF parameters.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the body.
      public: virtual void Init();

      /// \brief Finalize the body.
      public: void Fini();

      /// \brief Reset the link.
      public: void Reset();

      /// \brief Reset the velocity, acceleration, force and torque of link.
      public: void ResetPhysicsStates();

      /// \brief Update the parameters using new sdf values.
      /// \param[in] _sdf SDF values to load from.
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Update the collision.
      /// \param[in] _info Update information.
      public: void Update(const common::UpdateInfo &_info);
      using Base::Update;

      /// \brief Set the scale of the link.
      /// \param[in] _scale Scale to set the link to.
      public: void SetScale(const math::Vector3 &_scale);

      /// \brief Set whether this body is enabled.
      /// \param[in] _enable True to enable the link in the physics engine.
      public: virtual void SetEnabled(bool _enable) const = 0;

      /// \brief Get whether this body is enabled in the physics engine.
      /// \return True if the link is enabled.
      public: virtual bool GetEnabled() const = 0;

      /// \brief Set whether this entity has been selected by the user
      /// through the gui
      /// \param[in] _set True to set the link as selected.
      public: virtual bool SetSelected(bool _set);

      /// \brief Set whether gravity affects this body.
      /// \param[in] _mode True to enable gravity.
      public: virtual void SetGravityMode(bool _mode) = 0;

      /// \brief Get the gravity mode.
      /// \return True if gravity is enabled.
      public: virtual bool GetGravityMode() const = 0;

      /// \brief Set whether this body will collide with others in the
      /// model.
      /// \sa GetSelfCollide
      /// \param[in] _collide True to enable collisions.
      public: virtual void SetSelfCollide(bool _collide) = 0;

      /// \brief Set the collide mode of the body.
      /// \param[in] _mode Collision Mode,
      /// this can be: [all|none|sensors|fixed|ghost]
      /// all: collides with everything
      /// none: collides with nothing
      /// sensors: collides with everything else but other sensors
      /// fixed: collides with everything else but other fixed
      /// ghost: collides with everything else but other ghost
      public: void SetCollideMode(const std::string &_mode);

      /// \brief Get Self-Collision Flag.
      /// Two links within the same model will not collide if both have
      /// self_collide == false. \n
      /// link 1 and link2 collide = link1.self_collide || link2.self_collide
      /// Bodies connected by a joint are exempt from this, and will
      /// never collide.
      /// \return True if self collision is enabled.
      public: bool GetSelfCollide() const;

      /// \brief Set the laser retro reflectiveness.
      /// \param[in] _retro Retro value for all child collisions.
      public: void SetLaserRetro(float _retro);

      /// \brief Set the linear velocity of the body.
      /// \param[in] _vel Linear velocity.
      public: virtual void SetLinearVel(const math::Vector3 &_vel) = 0;

      /// \brief Set the angular velocity of the body.
      /// \param[in] _vel Angular velocity.
      public: virtual void SetAngularVel(const math::Vector3 &_vel) = 0;

      /// \brief Set the linear acceleration of the body.
      /// \param[in] _accel Linear acceleration.
      public: void SetLinearAccel(const math::Vector3 &_accel);

      /// \brief Set the angular acceleration of the body.
      /// \param[in] _accel Angular acceleration.
      public: void SetAngularAccel(const math::Vector3 &_accel);

      /// \brief Set the force applied to the body.
      /// \param[in] _force Force value.
      public: virtual void SetForce(const math::Vector3 &_force) = 0;

      /// \brief Set the torque applied to the body.
      /// \param[in] _torque Torque value.
      public: virtual void SetTorque(const math::Vector3 &_torque) = 0;

      /// \brief Add a force to the body.
      /// \param[in] _force Force to add.
      public: virtual void AddForce(const math::Vector3 &_force) = 0;

      /// \brief Add a force to the body, components are relative to the
      /// body's own frame of reference.
      /// \param[in] _force Force to add.
      public: virtual void AddRelativeForce(const math::Vector3 &_force) = 0;

      /// \brief Add a force to the body using a global position.
      /// \param[in] _force Force to add.
      /// \param[in] _pos Position in global coord frame to add the force.
      public: virtual void AddForceAtWorldPosition(const math::Vector3 &_force,
                  const math::Vector3 &_pos) = 0;

      /// \brief Add a force to the body at position expressed to the body's
      /// own frame of reference.
      /// \param[in] _force Force to add.
      /// \param[in] _relPos Position on the link to add the force.
      public: virtual void AddForceAtRelativePosition(
                  const math::Vector3 &_force,
                  const math::Vector3 &_relPos) = 0;

      /// \brief Add a force expressed in the link frame.
      /// \param[in] _force Direction vector expressed in the link frame. Each
      /// component corresponds to the force which will be added in that axis
      /// and the vector's magnitude corresponds to the total force.
      /// \param[in] _offset Offset position expressed in the link frame. It
      /// defaults to the link origin.
      public: virtual void AddLinkForce(const math::Vector3 &_force,
          const math::Vector3 &_offset = math::Vector3::Zero) = 0;

      /// \brief Add a torque to the body.
      /// \param[in] _torque Torque value to add to the link.
      public: virtual void AddTorque(const math::Vector3 &_torque) = 0;

      /// \brief Add a torque to the body, components are relative to the
      /// body's own frame of reference.
      /// \param[in] _torque Torque value to add.
      public: virtual void AddRelativeTorque(const math::Vector3 &_torque) = 0;

      /// \brief Get the pose of the body's center of gravity in the world
      ///        coordinate frame.
      /// \return Pose of the body's center of gravity in the world coordinate
      ///         frame.
      public: math::Pose GetWorldCoGPose() const;

      /// \brief Get the linear velocity of the origin of the link frame,
      ///        expressed in the world frame.
      /// \return Linear velocity of the link frame.
      public: virtual math::Vector3 GetWorldLinearVel() const
              {return this->GetWorldLinearVel(math::Vector3::Zero);}

      /// \brief Get the linear velocity of a point on the body in the world
      ///        frame, using an offset expressed in a body-fixed frame. If
      ///        no offset is given, the velocity at the origin of the Link
      ///        frame will be returned.
      /// \param[in] _offset Offset of the point from the origin of the Link
      ///                    frame, expressed in the body-fixed frame.
      /// \return Linear velocity of the point on the body
      public: virtual math::Vector3 GetWorldLinearVel(
                  const math::Vector3 &_offset) const = 0;

      /// \brief Get the linear velocity of a point on the body in the world
      ///        frame, using an offset expressed in an arbitrary frame.
      /// \param[in] _offset Offset from the origin of the link frame expressed
      ///                    in a frame defined by _q.
      /// \param[in] _q Describes the rotation of a reference frame relative to
      ///               the world reference frame.
      /// \return Linear velocity of the point on the body in the world frame.
      public: virtual math::Vector3 GetWorldLinearVel(
                  const math::Vector3 &_offset,
                  const math::Quaternion &_q) const = 0;

      /// \brief Get the linear velocity at the body's center of gravity in the
      ///        world frame.
      /// \return Linear velocity at the body's center of gravity in the world
      ///         frame.
      public: virtual math::Vector3 GetWorldCoGLinearVel() const = 0;

      /// \brief Get the linear velocity of the body.
      /// \return Linear velocity of the body.
      public: math::Vector3 GetRelativeLinearVel() const;

      /// \brief Get the angular velocity of the body.
      /// \return Angular velocity of the body.
      public: math::Vector3 GetRelativeAngularVel() const;

      /// \brief Get the linear acceleration of the body.
      /// \return Linear acceleration of the body.
      public: math::Vector3 GetRelativeLinearAccel() const;

      /// \brief Get the linear acceleration of the body in the world frame.
      /// \return Linear acceleration of the body in the world frame.
      public: math::Vector3 GetWorldLinearAccel() const;

      /// \brief Get the angular acceleration of the body.
      /// \return Angular acceleration of the body.
      public: math::Vector3 GetRelativeAngularAccel() const;

      /// \brief Get the angular momentum of the body CoG in the world frame,
      /// which is computed as (I * w), where
      /// I: inertia matrix in world frame
      /// w: angular velocity in world frame
      /// \return Angular momentum of the body.
      public: math::Vector3 GetWorldAngularMomentum() const;

      /// \brief Get the angular acceleration of the body in the world frame,
      /// which is computed as (I^-1 * (T - w x L)), where
      /// I: inertia matrix in world frame
      /// T: sum of external torques in world frame
      /// L: angular momentum of CoG in world frame
      /// w: angular velocity in world frame
      /// \return Angular acceleration of the body in the world frame.
      public: math::Vector3 GetWorldAngularAccel() const;

      /// \brief Get the force applied to the body.
      /// \return Force applied to the body.
      public: math::Vector3 GetRelativeForce() const;

      /// \brief Get the force applied to the body in the world frame.
      /// \return Force applied to the body in the world frame.
      public: virtual math::Vector3 GetWorldForce() const = 0;

      /// \brief Get the torque applied to the body.
      /// \return Torque applied to the body.
      public: math::Vector3 GetRelativeTorque() const;

      /// \brief Get the torque applied to the body in the world frame.
      /// \return Torque applied to the body in the world frame.
      public: virtual math::Vector3 GetWorldTorque() const = 0;

      /// \brief Get the model that this body belongs to.
      /// \return Model that this body belongs to.
      public: ModelPtr GetModel() const;

      /// \brief Get the inertia of the link.
      /// \return Inertia of the link.
      public: InertialPtr GetInertial() const {return this->inertial;}

      /// \brief Set the mass of the link.
      /// \parma[in] _inertial Inertial value for the link.
      public: void SetInertial(const InertialPtr &_inertial);

      /// \brief Get the world pose of the link inertia (cog position
      /// and Moment of Inertia frame). This differs from GetWorldCoGPose(),
      /// which returns the cog position in the link frame
      /// (not the Moment of Inertia frame).
      /// \return Inertial pose in world frame.
      public: math::Pose GetWorldInertialPose() const;

      /// \brief Get the inertia matrix in the world frame.
      /// \return Inertia matrix in world frame, returns matrix
      /// of zeros if link has no inertia.
      public: math::Matrix3 GetWorldInertiaMatrix() const;

      /// \cond
      /// This is an internal function
      /// \brief Get a collision by id.
      /// \param[in] _id Id of the collision object to find.
      /// \return Pointer to the collision, NULL if the id is invalid.
      public: CollisionPtr GetCollisionById(unsigned int _id) const;
      /// \endcond

      /// \brief Get a child collision by name
      /// \param[in] _name Name of the collision object.
      /// \return Pointer to the collision, NULL if the name was not found.
      public: CollisionPtr GetCollision(const std::string &_name);

      /// \brief Get a child collision by index
      /// \param[in] _index Index of the collision object.
      /// \return Pointer to the collision, NULL if the name was not found.
      public: CollisionPtr GetCollision(unsigned int _index) const;

      /// \brief Get all the child collisions.
      /// \return A std::vector of all the child collisions.
      public: Collision_V GetCollisions() const;

      /// \brief Get the bounding box for the link and all the child
      /// elements.
      /// \return The link's bounding box.
      public: virtual math::Box GetBoundingBox() const;

      /// \brief Set the linear damping factor.
      /// \param[in] _damping Linear damping factor.
      public: virtual void SetLinearDamping(double _damping) = 0;

      /// \brief Set the angular damping factor.
      /// \param[in] _damping Angular damping factor.
      public: virtual void SetAngularDamping(double _damping) = 0;

      /// \brief Get the linear damping factor.
      /// \return Linear damping.
      public: double GetLinearDamping() const;

      /// \brief Get the angular damping factor.
      /// \return Angular damping.
      public: double GetAngularDamping() const;

      /// \TODO Implement this function.
      /// \brief Set whether this body is in the kinematic state.
      /// \param[in] _kinematic True to make the link kinematic only.
      public: virtual void SetKinematic(const bool &_kinematic);

      /// \TODO Implement this function.
      /// \brief Get whether this body is in the kinematic state.
      /// \return True if the link is kinematic only.
      public: virtual bool GetKinematic() const {return false;}

      /// \brief Get sensor count
      ///
      /// This will return the number of sensors created by the link when it
      /// was loaded. This function is commonly used with
      /// Link::GetSensorName.
      /// \return The number of sensors created by the link.
      public: unsigned int GetSensorCount() const;

      /// \brief Get sensor name
      ///
      /// Get the name of a sensor based on an index. The index should be in
      /// the range of 0...Link::GetSensorCount().
      /// \note A Link does not manage or maintain a pointer to a
      /// sensors::Sensor. Access to a Sensor object
      /// is accomplished through the sensors::SensorManager. This was done to
      /// separate the physics engine from the sensor engine.
      /// \param[in] _index Index of the sensor name.
      /// \return The name of the sensor, or empty string if the index is out of
      /// bounds.
      public: std::string GetSensorName(unsigned int _index) const;

      /// \brief Connect to the add entity signal
      /// \param[in] _subscriber Subsciber callback function.
      /// \return Pointer to the connection, which must be kept in scope.
      public: template<typename T>
              event::ConnectionPtr ConnectEnabled(T _subscriber)
              {return enabledSignal.Connect(_subscriber);}

      /// \brief Disconnect to the add entity signal.
      /// \param[in] _conn Connection pointer to disconnect.
      public: void DisconnectEnabled(event::ConnectionPtr &_conn)
              {enabledSignal.Disconnect(_conn);}

      /// \brief Fill a link message
      /// \param[out] _msg Message to fill
      public: void FillMsg(msgs::Link &_msg);

      /// \brief Update parameters from a message
      /// \param[in] _msg Message to read.
      public: void ProcessMsg(const msgs::Link &_msg);

      /// \brief Joints that have this Link as a parent Link.
      /// \param[in] _joint Joint that is a child of this link.
      public: void AddChildJoint(JointPtr _joint);

      /// \brief Joints that have this Link as a child Link.
      /// \param[in] _joint Joint that is a parent of this link.
      public: void AddParentJoint(JointPtr _joint);

      /// \brief Remove Joints that have this Link as a child Link.
      /// \param[in] _jointName Parent Joint name.
      public: void RemoveParentJoint(const std::string &_jointName);

      /// \brief Remove Joints that have this Link as a parent Link
      /// \param[in] _jointName Child Joint name.
      public: void RemoveChildJoint(const std::string &_jointName);

      // Documentation inherited.
      public: virtual void RemoveChild(EntityPtr _child);
      using Base::RemoveChild;

      /// \brief Attach a static model to this link
      /// \param[in] _model Pointer to a static model.
      /// \param[in] _offset Pose relative to this link to place the model.
      public: void AttachStaticModel(ModelPtr &_model,
                                     const math::Pose &_offset);

      /// \brief Detach a static model from this link.
      /// \param[in] _modelName Name of an attached model to detach.
      public: void DetachStaticModel(const std::string &_modelName);

      /// \brief Detach all static models from this link.
      public: void DetachAllStaticModels();

      /// \internal
      /// \brief Called when the pose is changed. Do not call this directly.
      public: virtual void OnPoseChange();

      /// \brief Set the current link state.
      /// \param[in] _state The state to set the link to.
      public: void SetState(const LinkState &_state);

      /// \brief Update the mass matrix.
      public: virtual void UpdateMass() {}

      /// \brief Update surface parameters.
      public: virtual void UpdateSurface() {}

      /// \brief Allow the link to auto disable.
      /// \param[in] _disable If true, the link is allowed to auto disable.
      public: virtual void SetAutoDisable(bool _disable) = 0;

      /// \brief Returns a vector of children Links connected by joints.
      /// \return A vector of children Links connected by joints.
      public: Link_V GetChildJointsLinks() const;

      /// \brief Returns a vector of parent Links connected by joints.
      /// \return Vector of parent Links connected by joints.
      public: Link_V GetParentJointsLinks() const;

      /// \brief Enable/Disable link data publishing
      /// \param[in] _enable True to enable publishing, false to stop publishing
      public: void SetPublishData(bool _enable);

      /// \brief Get the parent joints.
      public: Joint_V GetParentJoints() const;

      /// \brief Get the child joints.
      public: Joint_V GetChildJoints() const;

      /// \brief Remove a collision from the link.
      /// \param[int] _name Name of the collision to remove.
      public: void RemoveCollision(const std::string &_name);

      /// \brief Returns this link's potential energy,
      /// based on position in world frame and gravity.
      /// \return this link's potential energy,
      public: double GetWorldEnergyPotential() const;

      /// \brief Returns this link's kinetic energy
      /// computed using link's CoG velocity in the inertial (world) frame.
      /// \return this link's kinetic energy
      public: double GetWorldEnergyKinetic() const;

      /// \brief Returns this link's total energy, or
      /// sum of Link::GetWorldEnergyPotential() and
      /// Link::GetWorldEnergyKinetic().
      /// \return this link's total energy
      public: double GetWorldEnergy() const;

      /// \brief Returns the visual message specified by its name
      /// \param[in] name of the visual message
      /// \return visual message
      public: msgs::Visual GetVisualMessage(const std::string &_name) const;

      /// \brief Freeze link to ground (inertial frame).
      /// \param[in] _static if true, freeze link to ground.  Otherwise
      /// unfreeze link.
      public: virtual void SetLinkStatic(bool _static) = 0;

      /// \brief Move Link given source and target frames specified in
      /// world coordinates. Assuming link's relative pose to
      /// source frame (_worldReferenceFrameSrc) remains unchanged relative
      /// to destination frame (_worldReferenceFrameDst).
      /// \param[in] _worldReferenceFrameSrc initial reference frame to
      /// which this link is attached.
      /// \param[in] _worldReferenceFrameDst final location of the
      /// reference frame specified in world coordinates.
      public: void MoveFrame(const math::Pose &_worldReferenceFrameSrc,
                        const math::Pose &_worldReferenceFrameDst);

      /// \brief Helper function to find all connected links of a link
      /// based on parent/child relations of joints. For example,
      /// if Link0 --> Link1 --> ... --> LinkN is a kinematic chain
      /// with Link0 being the base link.  Then, call by Link1:
      ///   Link1->FindAllConnectedLinksHelper(Link0, _list, true);
      /// should return true with _list containing Link1 through LinkN.
      /// In the case the _originalParentLink is part of a loop,
      /// _connectedLinks is cleared and the function returns false.
      /// \param[in] _originParentLink if this link is a child link of
      /// the search, we've found a loop.
      /// \param[in/out] _connectedLinks aggregate list of connected links.
      /// \param[in] _fistLink this is the first Link, skip over the parent
      /// link that matches the _originalParentLink.
      /// \return true if successfully found a subset of connected links
      public: bool FindAllConnectedLinksHelper(
        const LinkPtr &_originalParentLink,
        Link_V &_connectedLinks, bool _fistLink = false);

      /// \brief Get a battery by name.
      /// \param[in] _name Name of the battery to get.
      /// \return Pointer to the battery, NULL if the name is invalid.
      public: BatteryPtr Battery(const std::string &_name) const;

      /// \brief Get a battery based on an index.
      /// \return A pointer to a Battery. Null if the _index is invalid.
      public: BatteryPtr Battery(const size_t _index) const;

      /// \brief Get the number of batteries in this link.
      /// \return Size of this->batteries array.
      /// \sa Link::Battery()
      public: size_t BatteryCount() const;

      /// \brief Publish timestamped link data such as velocity.
      private: void PublishData();

      /// \brief Load a new collision helper function.
      /// \param[in] _sdf SDF element used to load the collision.
      private: void LoadCollision(sdf::ElementPtr _sdf);

      /// \brief Set the inertial properties based on the collision
      /// entities.
      private: void SetInertialFromCollisions();

      /// \brief On collision callback.
      /// \param[in] _msg Message that contains contact information.
      private: void OnCollision(ConstContactsPtr &_msg);

      /// \brief Parse visuals from SDF
      private: void ParseVisuals();

      /// \brief Helper function to see if _value is contained in _vector.
      /// \param[in] _vector a vector of boost link pointers.
      /// \param[in] _value a particular link pointer.
      /// \return true if value is in vector.
      private: bool ContainsLink(const Link_V &_vector, const LinkPtr &_value);

      /// \brief Update visual SDF's geometry size with the new scale.
      /// \param[in] _scale New scale applied to the visual
      private: void UpdateVisualGeomSDF(const math::Vector3 &_scale);

      /// \brief Update visual msgs.
      private: void UpdateVisualMsg();

      /// \brief Called when a new wrench message arrives. The wrench's force,
      /// torque and force offset are described in the link frame,
      /// \param[in] _msg The wrench message.
      private: void OnWrenchMsg(ConstWrenchPtr &_msg);

      /// \brief Process the message and add force and torque.
      /// \param[in] _msg The message to set the wrench from.
      private: void ProcessWrenchMsg(const msgs::Wrench &_msg);

      /// \brief Load a battery.
      /// \param[in] _sdf SDF parameter.
      private: void LoadBattery(const sdf::ElementPtr _sdf);

      /// \brief Inertial properties.
      protected: InertialPtr inertial;

      /// \brief Center of gravity visual elements.
      protected: std::vector<std::string> cgVisuals;

      /// \def Visuals_M
      /// \brief Map of unique ID to visual message.
      typedef std::map<uint32_t, msgs::Visual> Visuals_M;

      /// \brief Link visual elements.
      protected: Visuals_M visuals;

      /// \brief Linear acceleration.
      protected: math::Vector3 linearAccel;

      /// \brief Angular acceleration.
      protected: math::Vector3 angularAccel;

      /// \brief Offsets for the attached models.
      protected: std::vector<math::Pose> attachedModelsOffset;

      /// \brief This flag is set to true when the link is initialized.
      protected: bool initialized;

      /// \brief Event used when the link is enabled or disabled.
      private: event::EventT<void (bool)> enabledSignal;

      /// \brief This flag is used to trigger the enabled
      private: bool enabled;

      /// \brief Names of all the sensors attached to the link.
      private: std::vector<std::string> sensors;

      /// \brief All the parent joints.
      private: std::vector<JointPtr> parentJoints;

      /// \brief All the child joints.
      private: std::vector<JointPtr> childJoints;

      /// \brief All the attached models.
      private: std::vector<ModelPtr> attachedModels;

      /// \brief Link data publisher
      private: transport::PublisherPtr dataPub;

      /// \brief Link data message
      private: msgs::LinkData linkDataMsg;

      /// \brief True to publish data, false otherwise
      private: bool publishData;

      /// \brief Mutex to protect the publishData variable
      private: boost::recursive_mutex *publishDataMutex;

      /// \brief Cached list of collisions. This is here for performance.
      private: Collision_V collisions;

      /// \brief Wrench subscriber.
      private: transport::SubscriberPtr wrenchSub;

      /// \brief Vector of wrench messages to be processed.
      private: std::vector<msgs::Wrench> wrenchMsgs;

      /// \brief Mutex to protect the wrenchMsgs variable.
      private: boost::mutex wrenchMsgMutex;

      /// \brief All the attached batteries.
      private: std::vector<BatteryPtr> batteries;

#ifdef HAVE_OPENAL
      /// \brief All the audio sources
      private: std::vector<util::OpenALSourcePtr> audioSources;

      /// \brief An audio sink
      private: util::OpenALSinkPtr audioSink;

      /// \brief Subscriber to contacts with this collision. Used for audio
      /// playback.
      private: transport::SubscriberPtr audioContactsSub;
#endif
    };
    /// \}
  }
}
#endif
