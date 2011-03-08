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
 * SVN: $Id$
 */

#ifndef ENTITY_HH
#define ENTITY_HH

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include "Publisher.hh"
#include "Box.hh"
#include "Common.hh"
#include "Pose3d.hh"
#include "Param.hh"

namespace gazebo
{
  namespace msgs
  {
    class Visual;
  }

  /// \addtogroup gazebo_server
  /// \{
  
  /// Base class for all objects in Gazebo
  /*
   * Facilitates meshing of physics engine with rendering engine
   */
  class Entity : public Common
  {

    /// \brief Constructor
    /// \param parent Parent of the entity.
    public: Entity(Common *parent = NULL);
  
    /// \brief Destructor
    public: virtual ~Entity();

    /// \brief Load
    public: virtual void Load(XMLConfigNode *node);
 
    public: void SetName(const std::string &name);
 
    /// \brief Set whether this entity is static: immovable
    /// \param s Bool, true = static
    public: void SetStatic(const bool &s);
  
    /// \brief Return whether this entity is static
    /// \return bool True = static
    public: bool IsStatic() const;

    /// \brief Return the bounding box for the entity 
    public: Box GetBoundingBox() const;

    /// \brief Get the absolute pose of the entity
    public: virtual Pose3d GetWorldPose() const;

    /// \brief Get the pose of the entity relative to its parent
    public: Pose3d GetRelativePose() const;

    /// \brief Get the pose relative to the model this entity belongs to
    public: Pose3d GetModelRelativePose() const;

    /// \brief Set the pose of the entity relative to its parent
    public: void SetRelativePose(const Pose3d &pose, bool notify = true);

    /// \brief Set the abs pose of the entity
    public: void SetWorldPose(const Pose3d &pose, bool notify=true);

    /// \brief Set the position of the entity relative to its parent
    public: void SetRelativePosition(const Vector3 &pos);

    /// \brief Set the rotation of the entity relative to its parent
    public: void SetRelativeRotation(const Quatern &rot);


    /// \brief Get the linear velocity of the entity
    public: virtual Vector3 GetRelativeLinearVel() const
            {return Vector3();}

    /// \brief Get the linear velocity of the entity in the world frame
    public: virtual Vector3 GetWorldLinearVel() const
            {return Vector3();}


    /// \brief Get the angular velocity of the entity
    public: virtual Vector3 GetRelativeAngularVel() const
            {return Vector3();}

    /// \brief Get the angular velocity of the entity in the world frame
    public: virtual Vector3 GetWorldAngularVel() const
            {return Vector3();}


    /// \brief Get the linear acceleration of the entity
    public: virtual Vector3 GetRelativeLinearAccel() const
            {return Vector3();}

    /// \brief Get the linear acceleration of the entity in the world frame
    public: virtual Vector3 GetWorldLinearAccel() const
            {return Vector3();}


    /// \brief Get the angular acceleration of the entity 
    public: virtual Vector3 GetRelativeAngularAccel() const
            {return Vector3();}

    /// \brief Get the angular acceleration of the entity in the world frame
    public: virtual Vector3 GetWorldAngularAccel() const
            {return Vector3();}

    /// Register a visual
    public: void RegisterVisual();

    /// \brief This function is called when the entity's (or one of its parents)
    ///        pose of the parent has changed
    protected: virtual void OnPoseChange() {}

    /// \brief Handle a change of pose
    private: void PoseChange(bool notify = true);

    // is this an static entity
    protected: ParamT<bool> *staticP;
  
    /// \brief Visual stuff
    protected: msgs::Visual *visualMsg;

    private: Pose3d relativePose;
    private: PublisherPtr pose_pub;
    protected: PublisherPtr vis_pub;
  };
  
  /// \}
}

#endif
