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

#include <string>

#include "msgs/msgs.h"

#include "transport/TransportTypes.hh"
#include "common/CommonTypes.hh"
#include "math/MathTypes.hh"
#include "math/Box.hh"

#include "math/Pose.hh"
#include "physics/PhysicsTypes.hh"

#include "physics/Base.hh"

namespace gazebo
{
	namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Base class for all physics objects in Gazebo
    class Entity : public Base
    {
      /// \brief Constructor
      /// \param parent Parent of the entity.
      public: Entity(BasePtr parent);
    
      /// \brief Destructor
      public: virtual ~Entity();
  
      /// \brief Load
      /// \param node Pointer to an configuration node
      public: virtual void Load( sdf::ElementPtr &_sdf );
  
      /// \brief Set the name of the entity 
      /// \param name The new name
      public: virtual void SetName(const std::string &name);
   
      /// \brief Set whether this entity is static: immovable
      /// \param s Bool, true = static
      public: void SetStatic(const bool &s);
    
      /// \brief Return whether this entity is static
      /// \return bool True = static
      public: bool IsStatic() const;

      /// \brief Set the initial pose
      /// \param p The initial pose
      public: void SetInitialRelativePose( const math::Pose &p );

      /// \brief Return the bounding box for the entity 
      public: virtual math::Box GetBoundingBox() const;
  
      /// \brief Get the absolute pose of the entity
      public: virtual math::Pose GetWorldPose() const;
  
      /// \brief Get the pose of the entity relative to its parent
      public: math::Pose GetRelativePose() const;
  
      /// \brief Set the pose of the entity relative to its parent
      /// \param pose The new pose
      /// \param notify True = tell children of the pose change
      public: void SetRelativePose(const math::Pose &pose, bool notify = true);
  
      /// \brief Set the world pose of the entity
      /// \param pose The new world pose
      /// \param notify True = tell children of the pose change
      public: void SetWorldPose(const math::Pose &pose, bool notify=true);
  
      /// \brief Get the linear velocity of the entity
      /// \return A math::Vector3 for the linear velocity
      public: virtual math::Vector3 GetRelativeLinearVel() const
              {return math::Vector3();}
  
      /// \brief Get the linear velocity of the entity in the world frame
      /// \return A math::Vector3 for the linear velocity
      public: virtual math::Vector3 GetWorldLinearVel() const
              {return math::Vector3();}
  
      /// \brief Get the angular velocity of the entity
      /// \return A math::Vector3 for the velocity
      public: virtual math::Vector3 GetRelativeAngularVel() const
              {return math::Vector3();}
  
      /// \brief Get the angular velocity of the entity in the world frame
      /// \return A math::Vector3 for the velocity
      public: virtual math::Vector3 GetWorldAngularVel() const
              {return math::Vector3();}
  
      /// \brief Get the linear acceleration of the entity
      /// \return A math::Vector3 for the acceleration
      public: virtual math::Vector3 GetRelativeLinearAccel() const
              {return math::Vector3();}
  
      /// \brief Get the linear acceleration of the entity in the world frame
      /// \return A math::Vector3 for the acceleration
      public: virtual math::Vector3 GetWorldLinearAccel() const
              {return math::Vector3();}
  
  
      /// \brief Get the angular acceleration of the entity 
      /// \return A math::Vector3 for the acceleration
      public: virtual math::Vector3 GetRelativeAngularAccel() const
              {return math::Vector3();}
  
      /// \brief Get the angular acceleration of the entity in the world frame
      /// \return A math::Vector3 for the acceleration
      public: virtual math::Vector3 GetWorldAngularAccel() const
              {return math::Vector3();}

      /// \brief Set to true if this entity is a canonical link for a model.
      /// \param _value True if the link is canonical.
      public: void SetCanonicalLink(bool _value);

      /// \brief A helper function that checks if this is a canonical body
      public: bool IsCanonicalLink() const;

      private: void PublishPose();
 
      /// \brief Get the parent model, if one exists
      /// \return Pointer to a model, or NULL if no parent model exists
      public: ModelPtr GetParentModel() const;

      /// \brief This function is called when the entity's (or one of its parents)
      ///        pose of the parent has changed
      protected: virtual void OnPoseChange() {}
  
      /// \brief Handle a change of pose
      private: void UpdatePhysicsPose(bool update_children = true);
  
      /// A helper that prevents numerous dynamic_casts
      private: EntityPtr parentEntity;

      private: bool isStatic;

      /// \brief Only used by Links. Included here for performance.
      private: bool isCanonicalLink;
  
      /// The initial pose of the entity
      private: math::Pose initialRelativePose;
      private: math::Pose worldPose;

      protected: transport::NodePtr node;
      protected: transport::PublisherPtr posePub;
      protected: transport::PublisherPtr visPub;

      protected: msgs::Visual *visualMsg;
      protected: msgs::Pose *poseMsg;
    };
    
    /// \}
  }
}
#endif
