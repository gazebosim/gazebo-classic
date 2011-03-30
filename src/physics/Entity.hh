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
/* Desc: Base class for all physical entities
 * Author: Nate Koenig
 * Date: 03 Apr 2007
 */

#ifndef ENTITY_HH
#define ENTITY_HH

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include "transport/Publisher.hh"

#include "common/Box.hh"
#include "common/Pose3d.hh"
#include "common/Param.hh"

#include "physics/Common.hh"

namespace gazebo
{
  namespace msgs
  {
    class Visual;
    class Pose;
  }

	namespace physics
  {
    class World;
    class Model;
  
    /// \addtogroup gazebo_server
    /// \{
    
    /// Base class for all physics objects in Gazebo
    class Entity : public Common
    {
      /// \brief Constructor
      /// \param parent Parent of the entity.
      public: Entity(Common *parent = NULL);
    
      /// \brief Destructor
      public: virtual ~Entity();
  
      /// \brief Load
      public: virtual void Load(common::XMLConfigNode *node);
   
      public: void SetName(const std::string &name);
   
      /// \brief Set whether this entity is static: immovable
      /// \param s Bool, true = static
      public: void SetStatic(const bool &s);
    
      /// \brief Return whether this entity is static
      /// \return bool True = static
      public: bool IsStatic() const;
  
      /// \brief Return the bounding box for the entity 
      public: common::Box GetBoundingBox() const;
  
      /// \brief Get the absolute pose of the entity
      public: virtual common::Pose3d GetWorldPose() const;
  
      /// \brief Get the pose of the entity relative to its parent
      public: common::Pose3d GetRelativePose() const;
  
      /// \brief Get the pose relative to the model this entity belongs to
      public: common::Pose3d GetModelRelativePose() const;
  
      /// \brief Set the pose of the entity relative to its parent
      public: void SetRelativePose(const common::Pose3d &pose, bool notify = true);
  
      /// \brief Set the abs pose of the entity
      public: void SetWorldPose(const common::Pose3d &pose, bool notify=true);
  
      /// \brief Set the position of the entity relative to its parent
      public: void SetRelativePosition(const common::Vector3 &pos);
  
      /// \brief Set the rotation of the entity relative to its parent
      public: void SetRelativeRotation(const common::Quatern &rot);
  
  
      /// \brief Get the linear velocity of the entity
      public: virtual common::Vector3 GetRelativeLinearVel() const
              {return common::Vector3();}
  
      /// \brief Get the linear velocity of the entity in the world frame
      public: virtual common::Vector3 GetWorldLinearVel() const
              {return common::Vector3();}
  
  
      /// \brief Get the angular velocity of the entity
      public: virtual common::Vector3 GetRelativeAngularVel() const
              {return common::Vector3();}
  
      /// \brief Get the angular velocity of the entity in the world frame
      public: virtual common::Vector3 GetWorldAngularVel() const
              {return common::Vector3();}
  
  
      /// \brief Get the linear acceleration of the entity
      public: virtual common::Vector3 GetRelativeLinearAccel() const
              {return common::Vector3();}
  
      /// \brief Get the linear acceleration of the entity in the world frame
      public: virtual common::Vector3 GetWorldLinearAccel() const
              {return common::Vector3();}
  
  
      /// \brief Get the angular acceleration of the entity 
      public: virtual common::Vector3 GetRelativeAngularAccel() const
              {return common::Vector3();}
  
      /// \brief Get the angular acceleration of the entity in the world frame
      public: virtual common::Vector3 GetWorldAngularAccel() const
              {return common::Vector3();}
  
      /// Register a visual
      public: void RegisterVisual();
  
      /// \brief Get the parent model, if one exists
      /// \return Pointer to a model, or NULL if no parent model exists
      public: Model *GetParentModel() const;

      /// \brief This function is called when the entity's (or one of its parents)
      ///        pose of the parent has changed
      protected: virtual void OnPoseChange() {}
  
      /// \brief Handle a change of pose
      private: void PoseChange(bool notify = true);
  
      // is this an static entity
      protected: common::ParamT<bool> *staticP;
    
      /// \brief Visual stuff
      protected: msgs::Visual *visualMsg;
      protected: msgs::Pose *poseMsg;
  
      private: common::Pose3d relativePose;
      private: transport::PublisherPtr pose_pub;
      protected: transport::PublisherPtr vis_pub;
    };
    
    /// \}
  }
}
#endif
