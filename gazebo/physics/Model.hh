/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#ifndef MODEL_HH
#define MODEL_HH

#include <string>
#include <map>
#include <vector>

#include "common/CommonTypes.hh"
#include "physics/PhysicsTypes.hh"

#include "physics/ModelState.hh"
#include "physics/Entity.hh"

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{
  namespace physics
  {
    class JointController;
    class Gripper;

    /// \addtogroup gazebo_physics
    /// \{

    /// \brief A model object
    class Model : public Entity
    {
      /// \brief Constructor
      /// \param parent Parent object
      public: Model(BasePtr parent);

      /// \brief Destructor
      public: virtual ~Model();

      /// \brief Load the model
      /// \param _sdf SDF parameters
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the model
      public: virtual void Init();

      /// \brief Update the model
      public: void Update();

      /// \brief Finalize the model
      public: virtual void Fini();

      /// \brief update the parameters using new sdf values
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Get the SDF values for the model
      public: virtual const sdf::ElementPtr GetSDF();

      /// \brief Remove a child
      /// \param child Remove a child entity
      public: virtual void RemoveChild(EntityPtr child);

      /// \brief Reset the model
      public: void Reset();

      /// \brief Set the linear velocity of the model
      /// \param vel The new linear velocity
      public: void SetLinearVel(const math::Vector3 &vel);

      /// \brief Set the angular velocity of the model
      /// \param vel The new angular velocity
      public: void SetAngularVel(const math::Vector3 &vel);

      /// \brief Set the linear acceleration of the model
      /// \param vel The new linear acceleration
      public: void SetLinearAccel(const math::Vector3 &vel);

      /// \brief Set the angular acceleration of the model
      /// \param vel The new angular acceleration
      public: void SetAngularAccel(const math::Vector3 &vel);

      /// \brief Get the linear velocity of the entity
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body
      public: virtual math::Vector3 GetRelativeLinearVel() const;

      /// \brief Get the linear velocity of the entity in the world frame
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body
      public: virtual math::Vector3 GetWorldLinearVel() const;

      /// \brief Get the angular velocity of the entity
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body
      public: virtual math::Vector3 GetRelativeAngularVel() const;

      /// \brief Get the angular velocity of the entity in the world frame
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body
      public: virtual math::Vector3 GetWorldAngularVel() const;

      /// \brief Get the linear acceleration of the entity
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body
      public: virtual math::Vector3 GetRelativeLinearAccel() const;

      /// \brief Get the linear acceleration of the entity in the world frame
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body
      public: virtual math::Vector3 GetWorldLinearAccel() const;

      /// \brief Get the angular acceleration of the entity
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body
      public: virtual math::Vector3 GetRelativeAngularAccel() const;

      /// \brief Get the angular acceleration of the entity in the world frame
      /// \return math::Vector3, set to 0, 0, 0 if the model has no body
      public: virtual math::Vector3 GetWorldAngularAccel() const;

      /// \brief Get the size of the bounding box
      /// \return The bounding box
      public: virtual math::Box GetBoundingBox() const;

      /// \brief Get the number of joints
      /// \return Get the number of joints
      public: unsigned int GetJointCount() const;

      /// \brief Get a joing by index
      /// \param index Index of the joint
      /// \return A pointer to the joint
      public: JointPtr GetJoint(unsigned int index) const;

      /// \brief Get a joint
      /// \param name The name of the joint, specified in the world file
      /// \return Pointer to the joint
      public: JointPtr GetJoint(const std::string &name);

      /// \brief Get a link by id
      /// \return Pointer to the link
      public: LinkPtr GetLinkById(unsigned int _id) const;

      /// \brief Get a link by name
      /// \return Pointer to the link
      public: LinkPtr GetLink(const std::string &name ="canonical") const;

      /// \brief Get a child link by index
      /// \return Point to the link
      public: LinkPtr GetLink(unsigned int _index) const;

      /// \brief Set the gravity mode of the model
      public: void SetGravityMode(const bool &v);

      /// \brief Set the collide mode of the model
      /// \param m The collision mode
      public: void SetCollideMode(const std::string &m);

      /// \brief Set the laser retro reflectiveness of the model
      /// \param retro Retro reflectance value
      public: void SetLaserRetro(const float &retro);

      /// \brief Fill a model message
      /// \param _msg Message to fill
      public: void FillModelMsg(msgs::Model &_msg);

      /// \brief Update parameters from a model message
      public: void ProcessMsg(const msgs::Model &_msg);

      /// \brief Set the positions of a set of joints
      public: void SetJointPositions(
                  const std::map<std::string, double> &_jointPositions);

      /// \brief Joint Anaimation
      public: void SetJointAnimation(
                  const std::map<std::string, common::NumericAnimationPtr> anim,
                  boost::function<void()> _onComplete = NULL);

      /// \brief Stop the current animations
      public: virtual void StopAnimation();

      /// \brief Attach a static model to this model
      public: void AttachStaticModel(ModelPtr &_model, math::Pose _offset);

      /// \brief Detach a static model from this model
      public: void DetachStaticModel(const std::string &_model);

      /// \brief Get the current model state
      public: ModelState GetState();

      /// \brief Set the current model state
      public: void SetState(const ModelState &_state);

      /// \brief Enable all the links in all the models
      public: void SetEnabled(bool _enabled);

      /// \brief Set the current model pose by specifying pose of a link
      public: void SetLinkWorldPose(const math::Pose &_pose,
                                    std::string _linkName);
      /// \brief Set the current model pose by specifying pose of a link
      public: void SetLinkWorldPose(const math::Pose &_pose,
                                    const LinkPtr &_link);

      /// \brief Allow the model the auto disable. This is ignored if the
      /// model has joints.
      /// \param _disable If true, the model is allowed to auto disable.
      public: void SetAutoDisable(bool _disable);

      protected: virtual void OnPoseChange();

      /// \brief Load a joint helper function
      /// \param _sdf SDF parameter
      private: void LoadJoint(sdf::ElementPtr _sdf);

      /// \brief Load a plugin helper function
      /// \param _sdf SDF parameter
      private: void LoadPlugin(sdf::ElementPtr _sdf);

      /// \brief Load a gripper helper function
      /// \param _sdf SDF parameter
      private: void LoadGripper(sdf::ElementPtr _sdf);

      private: LinkPtr canonicalLink;

      private: Joint_V joints;
      private: std::vector<Gripper*> grippers;

      private: std::vector<ModelPluginPtr> plugins;

      private: transport::PublisherPtr jointPub;
      private: std::map<std::string, common::NumericAnimationPtr>
               jointAnimations;

      private: boost::function<void()> onJointAnimationComplete;
      private: common::Time prevAnimationTime;

      private: boost::recursive_mutex *updateMutex;
      private: JointController *jointController;

      /// used by Model::AttachStaticModel
      protected: std::vector<ModelPtr> attachedModels;

      /// used by Model::AttachStaticModel
      protected: std::vector<math::Pose> attachedModelsOffset;
    };
    /// \}
  }
}
#endif
