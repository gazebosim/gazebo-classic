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
/* Desc: Base class for all models
 * Author: Nathan Koenig and Andrew Howard
 * Date: 8 May 2003
 */

#ifndef _MODEL_HH_
#define _MODEL_HH_

#include <string>
#include <map>
#include <vector>
#include <boost/function.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/ModelState.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/util/system.hh"

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{
  namespace physics
  {
    class Gripper;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class Model Model.hh physics/physics.hh
    /// \brief A model is a collection of links, joints, and plugins.
    class GZ_PHYSICS_VISIBLE Model : public Entity
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent object.
      public: explicit Model(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~Model();

      /// \brief Load the model.
      /// \param[in] _sdf SDF parameters to load from.
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Load all the joints.
      public: void LoadJoints();

      /// \brief Initialize the model.
      public: virtual void Init();

      /// \brief Update the model.
      public: void Update();

      /// \brief Finalize the model.
      public: virtual void Fini();

      /// \brief Update the parameters using new sdf values.
      /// \param[in] _sdf SDF values to update from.
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Get the SDF values for the model.
      /// \return The SDF value for this model.
      public: virtual const sdf::ElementPtr GetSDF();

      /// \brief Remove a child.
      /// \param[in] _child Remove a child entity.
      public: virtual void RemoveChild(EntityPtr _child);
      using Base::RemoveChild;

      /// \brief Reset the model.
      public: void Reset();

      /// \brief Reset the velocity, acceleration, force and torque of
      /// all child links.
      public: void ResetPhysicsStates();

      /// \brief Set the linear velocity of the model, and all its links.
      /// \param[in] _vel The new linear velocity.
      public: void SetLinearVel(const math::Vector3 &_vel);

      /// \brief Set the angular velocity of the model, and all its links.
      /// \param[in] _vel The new angular velocity.
      public: void SetAngularVel(const math::Vector3 &_vel);

      /// \brief Set the linear acceleration of the model, and all its
      /// links.
      /// \param[in] _vel The new linear acceleration.
      public: void SetLinearAccel(const math::Vector3 &_vel);

      /// \brief Set the angular acceleration of the model, and all its
      /// links.
      /// \param[in] _vel The new angular acceleration
      public: void SetAngularAccel(const math::Vector3 &_vel);

      /// \brief Get the linear velocity of the entity.
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body.
      public: virtual math::Vector3 GetRelativeLinearVel() const;

      /// \brief Get the linear velocity of the entity in the world frame.
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body.
      public: virtual math::Vector3 GetWorldLinearVel() const;

      /// \brief Get the angular velocity of the entity.
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body.
      public: virtual math::Vector3 GetRelativeAngularVel() const;

      /// \brief Get the angular velocity of the entity in the world frame.
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body.
      public: virtual math::Vector3 GetWorldAngularVel() const;

      /// \brief Get the linear acceleration of the entity.
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body.
      public: virtual math::Vector3 GetRelativeLinearAccel() const;

      /// \brief Get the linear acceleration of the entity in the world frame.
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body.
      public: virtual math::Vector3 GetWorldLinearAccel() const;

      /// \brief Get the angular acceleration of the entity.
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body.
      public: virtual math::Vector3 GetRelativeAngularAccel() const;

      /// \brief Get the angular acceleration of the entity in the world frame.
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body.
      public: virtual math::Vector3 GetWorldAngularAccel() const;

      /// \brief Get the size of the bounding box.
      /// \return The bounding box.
      public: virtual math::Box GetBoundingBox() const;

      /// \brief Get the number of joints.
      /// \return Get the number of joints.
      public: unsigned int GetJointCount() const;

      /// \brief Get a nested model that is a direct child of this model.
      /// \param[in] _name Name of the child model to get.
      /// \return Pointer to the model, NULL if the name is invalid.
      public: ModelPtr NestedModel(const std::string &_name) const;

      /// \brief Get all the nested models.
      /// \return a vector of Model's in this model
      public: const Model_V &NestedModels() const;

      /// \brief Construct and return a vector of Link's in this model
      /// Note this constructs the vector of Link's on the fly, could be costly
      /// \return a vector of Link's in this model
      public: const Link_V &GetLinks() const;

      /// \brief Get the joints.
      /// \return Vector of joints.
      public: const Joint_V &GetJoints() const;

      /// \brief Get a joint
      /// \param name The name of the joint, specified in the world file
      /// \return Pointer to the joint
      public: JointPtr GetJoint(const std::string &name);

      /// \cond
      /// This is an internal function
      /// \brief Get a link by id.
      /// \return Pointer to the link, NULL if the id is invalid.
      public: LinkPtr GetLinkById(unsigned int _id) const;
      /// \endcond

      /// \brief Get a link by name.
      /// \param[in] _name Name of the link to get.
      /// \return Pointer to the link, NULL if the name is invalid.
      public: LinkPtr GetLink(const std::string &_name ="canonical") const;

      /// \brief If true, all links within the model will collide by default.
      /// Two links within the same model will not collide if both have
      /// link.self_collide == false.
      /// link 1 and link2 collide = link1.self_collide || link2.self_collide
      /// Bodies connected by a joint are exempt from this, and will
      /// never collide.
      /// \return True if self-collide enabled for this model, false otherwise.
      public: bool GetSelfCollide() const;

      /// \brief Set this model's self_collide property
      /// \sa GetSelfCollide
      /// \param[in] _self_collide True if self-collisions enabled by default.
      public: void SetSelfCollide(bool _self_collide);

      /// \brief Set the gravity mode of the model.
      /// \param[in] _value False to turn gravity on for the model.
      public: void SetGravityMode(const bool &_value);

      /// \TODO This is not implemented in Link, which means this function
      /// doesn't do anything.
      /// \brief Set the collide mode of the model.
      /// \param[in] _mode The collision mode
      public: void SetCollideMode(const std::string &_mode);

      /// \brief Set the laser retro reflectiveness of the model.
      /// \param[in] _retro Retro reflectance value.
      public: void SetLaserRetro(const float _retro);

      /// \brief Fill a model message.
      /// \param[in] _msg Message to fill using this model's data.
      public: virtual void FillMsg(msgs::Model &_msg);

      /// \brief Update parameters from a model message.
      /// \param[in] _msg Message to process.
      public: void ProcessMsg(const msgs::Model &_msg);

      /// \brief Set the positions of a Joint by name.
      /// \sa JointController::SetJointPosition
      /// \param[in] _jointName Name of the joint to set.
      /// \param[in] _position Position to set the joint to.
      public: void SetJointPosition(const std::string &_jointName,
                                    double _position, int _index = 0);

      /// \brief Set the positions of a set of joints.
      /// \sa JointController::SetJointPositions.
      /// \param[in] _jointPositions Map of joint names to their positions.
      public: void SetJointPositions(
                  const std::map<std::string, double> &_jointPositions);

      /// \brief Joint Animation.
      /// \param[in] _anim Map of joint names to their position animation.
      /// \param[in] _onComplete Callback function for when the animation
      /// completes.
      public: void SetJointAnimation(
               const std::map<std::string, common::NumericAnimationPtr> &_anims,
               boost::function<void()> _onComplete = NULL);

      /// \brief Stop the current animations.
      public: virtual void StopAnimation();

      /// \brief Attach a static model to this model
      ///
      /// This function takes as input a static Model, which is a Model that
      /// has been marked as static (no physics simulation), and attaches it
      /// to this Model with a given offset.
      ///
      /// This function is useful when you want to simulate a grasp of a
      /// static object, or move a static object around using a dynamic
      /// model.
      ///
      /// If you are in doubt, do not use this function.
      ///
      /// \param[in] _model Pointer to the static model.
      /// \param[in] _offset Offset, relative to this Model, to place _model.
      public: void AttachStaticModel(ModelPtr &_model, math::Pose _offset);

      /// \brief Detach a static model from this model.
      /// \param[in] _model Name of an attached static model to remove.
      /// \sa Model::AttachStaticModel.
      public: void DetachStaticModel(const std::string &_model);

      /// \brief Set the current model state.
      /// \param[in] _state State to set the model to.
      public: void SetState(const ModelState &_state);

      /// \brief Set the scale of model.
      /// \param[in] _scale Scale to set the model to.
      public: void SetScale(const math::Vector3 &_scale);

      /// \brief Enable all the links in all the models.
      /// \param[in] _enabled True to enable all the links.
      public: void SetEnabled(bool _enabled);

      /// \brief Set the Pose of the entire Model by specifying
      /// desired Pose of a Link within the Model.  Doing so, keeps
      /// the configuration of the Model unchanged, i.e. all Joint angles
      /// are unchanged.
      /// \param[in] _pose Pose to set the link to.
      /// \param[in] _linkName Name of the link to set.
      public: void SetLinkWorldPose(const math::Pose &_pose,
                                    std::string _linkName);

      /// \brief Set the Pose of the entire Model by specifying
      /// desired Pose of a Link within the Model.  Doing so, keeps
      /// the configuration of the Model unchanged, i.e. all Joint angles
      /// are unchanged.
      /// \param[in] _pose Pose to set the link to.
      /// \param[in] _link Pointer to the link to set.
      public: void SetLinkWorldPose(const math::Pose &_pose,
                                    const LinkPtr &_link);

      /// \brief Allow the model the auto disable. This is ignored if the
      /// model has joints.
      /// \param[in] _disable If true, the model is allowed to auto disable.
      public: void SetAutoDisable(bool _disable);

      /// \brief Return the value of the SDF <allow_auto_disable> element.
      /// \return True if auto disable is allowed for this model.
      public: bool GetAutoDisable() const;

      /// \brief Load all plugins
      ///
      /// Load all plugins specified in the SDF for the model.
      public: void LoadPlugins();

      /// \brief Get the number of plugins this model has.
      /// \return Number of plugins associated with this model.
      public: unsigned int GetPluginCount() const;

      /// \brief Get the number of sensors attached to this model.
      /// This will count all the sensors attached to all the links.
      /// \return Number of sensors.
      public: unsigned int GetSensorCount() const;

      /// \brief Get a handle to the Controller for the joints in this model.
      /// \return A handle to the Controller for the joints in this model.
      public: JointControllerPtr GetJointController();

      /// \brief Get a gripper based on an index.
      /// \return A pointer to a Gripper. Null if the _index is invalid.
      public: GripperPtr GetGripper(size_t _index) const;

      /// \brief Get the number of grippers in this model.
      /// \return Size of this->grippers array.
      /// \sa Model::GetGripper()
      public: size_t GetGripperCount() const;

      /// \brief Returns the potential energy of all links
      /// and joint springs in the model.
      /// \return this link's potential energy,
      public: double GetWorldEnergyPotential() const;

      /// \brief Returns sum of the kinetic energies of all links
      /// in this model.  Computed using link's CoG velocity in
      /// the inertial (world) frame.
      /// \return this link's kinetic energy
      public: double GetWorldEnergyKinetic() const;

      /// \brief Returns this model's total energy, or
      /// sum of Model::GetWorldEnergyPotential() and
      /// Model::GetWorldEnergyKinetic().
      /// \return this link's total energy
      public: double GetWorldEnergy() const;

      /// \brief Create a joint for this model
      /// \param[in] _name name of joint
      /// \param[in] _type type of joint
      /// \param[in] _parent parent link of joint
      /// \param[in] _child child link of joint
      /// \return a JointPtr to the new joint created,
      ///         returns NULL JointPtr() if joint by name _name
      ///         already exists.
      /// \throws common::Exception When _type is not recognized
      public: gazebo::physics::JointPtr CreateJoint(
        const std::string &_name, const std::string &_type,
        physics::LinkPtr _parent, physics::LinkPtr _child);

      /// \brief Remove a joint for this model
      /// \param[in] _name name of joint
      public: void RemoveJoint(const std::string &_name);

      public: boost::shared_ptr<Model> shared_from_this()
        {
          return boost::static_pointer_cast<Model>(Entity::shared_from_this());
        }

      /// \brief Callback when the pose of the model has been changed.
      protected: virtual void OnPoseChange();

      /// \brief Load all the links.
      private: void LoadLinks();

      /// \brief Load all the nested models.
      private: void LoadModels();

      /// \brief Load a joint helper function.
      /// \param[in] _sdf SDF parameter.
      private: void LoadJoint(sdf::ElementPtr _sdf);

      /// \brief Load a plugin helper function.
      /// \param[in] _sdf SDF parameter.
      private: void LoadPlugin(sdf::ElementPtr _sdf);

      /// \brief Load a gripper helper function.
      /// \param[in] _sdf SDF parameter.
      private: void LoadGripper(sdf::ElementPtr _sdf);

      /// \brief Remove a link from the model's cached list of links.
      /// This does not delete the link.
      /// \param[in] _name Name of the link to remove.
      private: void RemoveLink(const std::string &_name);

      /// used by Model::AttachStaticModel
      protected: std::vector<ModelPtr> attachedModels;

      /// used by Model::AttachStaticModel
      protected: std::vector<math::Pose> attachedModelsOffset;

      /// \brief Publisher for joint info.
      protected: transport::PublisherPtr jointPub;

      /// \brief The canonical link of the model.
      private: LinkPtr canonicalLink;

      /// \brief All the joints in the model.
      private: Joint_V joints;

      /// \brief Cached list of links. This is here for performance.
      private: Link_V links;

      /// \brief Cached list of nested models.
      private: Model_V models;

      /// \brief All the grippers in the model.
      private: std::vector<GripperPtr> grippers;

      /// \brief All the model plugins.
      private: std::vector<ModelPluginPtr> plugins;

      /// \brief The joint animations.
      private: std::map<std::string, common::NumericAnimationPtr>
               jointAnimations;

      /// \brief Callback used when a joint animation completes.
      private: boost::function<void()> onJointAnimationComplete;

      /// \brief Mutex used during the update cycle.
      private: mutable boost::recursive_mutex updateMutex;

      /// \brief Controller for the joints.
      private: JointControllerPtr jointController;
    };
    /// \}
  }
}
#endif
