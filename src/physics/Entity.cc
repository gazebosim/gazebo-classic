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
/* Desc: External interfaces for Gazebo
 * Author: Nate Koenig
 * Date: 03 Apr 2007
 */

#include "msgs/msgs.h"

#include "common/Console.hh"

#include "transport/Publisher.hh"
#include "transport/Transport.hh"
#include "transport/Node.hh"

#include "physics/Geom.hh"
#include "physics/Model.hh"
#include "physics/World.hh"
#include "physics/Link.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/Entity.hh"

using namespace gazebo;
using namespace physics;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Entity::Entity(BasePtr parent)
  : Base(parent)
{
  this->isCanonicalLink = false;
  this->node = transport::NodePtr(new transport::Node());
  this->AddType(ENTITY);

  this->visualMsg = new msgs::Visual;
  this->poseMsg = new msgs::Pose;

  this->visualMsg->set_mesh_type( msgs::Visual::UNKNOWN );

  if (this->parent && this->parent->HasType(ENTITY))
  {
    this->parentEntity = boost::shared_dynamic_cast<Entity>(this->parent);
    this->SetStatic( this->parentEntity->IsStatic() );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Entity::~Entity()
{
  Base_V::iterator iter;

  // TODO: put this back in
  //this->GetWorld()->GetPhysicsEngine()->RemoveEntity(this);

  // Tell all renderers that I'm gone
  this->visualMsg->set_action( msgs::Visual::DELETE );
  this->visPub->Publish(*this->visualMsg);
  delete this->visualMsg;
  this->visualMsg = NULL;

  delete this->poseMsg;
  this->poseMsg = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Load
void Entity::Load(sdf::ElementPtr &_sdf)
{
  Base::Load(_sdf);
  this->node->Init(this->GetWorld()->GetName());
  this->posePub = this->node->Advertise<msgs::Pose>("~/pose", 10);
  this->visPub = this->node->Advertise<msgs::Visual>("~/visual", 10);

  this->visualMsg->mutable_header()->set_str_id(this->GetCompleteScopedName());

  if (_sdf->HasElement("origin"))
  {
     this->SetRelativePose( _sdf->GetElement("origin")->GetValuePose("pose") );
  }

  if (this->parent)
    this->visualMsg->set_parent_id( this->parent->GetCompleteScopedName() );

  this->visPub->Publish(*this->visualMsg);

  msgs::Init( *this->poseMsg, this->GetCompleteScopedName() );
}
 
////////////////////////////////////////////////////////////////////////////////
// Set the name of the entity
void Entity::SetName(const std::string &name)
{
  // TODO: if an entitie's name is changed, then the old visual is never
  // removed. Should add in functionality to modify/update the visual
  Base::SetName(name);
}

////////////////////////////////////////////////////////////////////////////////
// Set whether this entity is static: immovable
void Entity::SetStatic(const bool &s)
{
  Base_V::iterator iter;

  this->isStatic = s ;

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    EntityPtr e = boost::shared_dynamic_cast<Entity>(*iter);
    if (e)
      e->SetStatic(s);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Return whether this entity is static
bool Entity::IsStatic() const
{
  return this->isStatic;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the initial pose
void Entity::SetInitialPose(const math::Pose &p )
{
  this->initialPose = p;
}


////////////////////////////////////////////////////////////////////////////////
/// Return the bounding box for the entity 
math::Box Entity::GetBoundingBox() const
{
  return math::Box(math::Vector3(0,0,0), math::Vector3(1,1,1));
}

////////////////////////////////////////////////////////////////////////////////
/// Get the absolute pose of the entity
math::Pose Entity::GetWorldPose() const
{
  if (this->parent && this->parentEntity)
    return this->GetRelativePose() + this->parentEntity->GetWorldPose();
  else
    return this->GetRelativePose();
}

////////////////////////////////////////////////////////////////////////////////
// Get the pose of the entity relative to its parent
math::Pose Entity::GetRelativePose() const
{
  return this->relativePose;
}

// Set to true if this entity is a canonical link for a model.
void Entity::SetCanonicalLink(bool _value)
{
  this->isCanonicalLink = _value;
}

// Am I a canonical Link for my Model parent?
bool Entity::IsCanonicalLink() const
{
  return this->isCanonicalLink;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the pose of the entity relative to its parent
void Entity::SetRelativePose(const math::Pose &pose, bool notify)
{
  // debugging
  // if (this->GetCompleteScopedName() == "root::model_1::link_1")
  // {
  //   if (abs(this->relativePose.pos.x - pose.pos.x) > 0.00001)
  //   {
  //     gzdbg << "setting relative pose name [" << this->GetName()
  //           << "] old [" << this->relativePose
  //           << "] new [" << pose
  //           << "]\n";
  //     gzdbg << "diff [" << abs(this->relativePose.pos.x - pose.pos.x) << "]\n";
  //     printf("ok\n");
  //   }
  // }

  //if (pose != this->relativePose)
  {
    if (this->HasType(MODEL))
    {
      // lock physics

      // set relative pose of this model
      this->relativePose = pose;
      this->relativePose.Correct();

      // update all children pose, moving them with the model.
      // this happens when OnPoseChange uses GetWorldPose and
      // finds each link's world pose has changed due to parent
      // relative pose change above.
      this->UpdatePhysicsPose(true);

      // unlock physics
    }
    else if (this->IsCanonicalLink())
    {
      // lock physics

      // set relative pose of the parent model
      ModelPtr parentModel = boost::shared_static_cast<Model>(this->parent);
      parentModel->relativePose = parentModel->relativePose +
           (pose - this->GetRelativePose());

      // let the visualizer know that the parent pose has changed
      // given the canonical link relative pose is the same, this
      // is needed to show movement in visualizer
      parentModel->PublishPose();

      // loop through non-canonical bodies, so their worldPose remain unchanged
      for  (Base_V::iterator iter = parentModel->children.begin();
            iter != parentModel->children.end(); iter++)
      {
        if ((*iter)->HasType(ENTITY))
        {
          EntityPtr ent = boost::shared_static_cast<Entity>(*iter);
          if (!ent->IsCanonicalLink())
          {
            ent->relativePose = ent->relativePose +
               (pose - this->GetRelativePose());
          }
        }
      }
      // push changes for this canonical link back to the physics engine
      //this->UpdatePhysicsPose(false);

      // unlock physics
    }
    else
    {
      this->relativePose = pose;
      this->relativePose.Correct();
      if (notify) 
        this->UpdatePhysicsPose(true);
    }

    // do we really need to do this still?
    //if (notify) this->UpdatePhysicsPose(false);

    this->PublishPose();
  }
}

void Entity::PublishPose()
{
  msgs::Set( this->poseMsg, this->GetRelativePose());
  this->posePub->Publish( *this->poseMsg);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the pose relative to the model this entity belongs to
math::Pose Entity::GetModelRelativePose() const
{
  if (this->HasType(MODEL) || !this->parent)
    return math::Pose();

  return this->GetRelativePose() + this->parentEntity->GetModelRelativePose();
}

////////////////////////////////////////////////////////////////////////////////
// Set the abs pose of the entity
void Entity::SetWorldPose(const math::Pose &pose, bool notify)
{
  if (this->parent && this->parent->HasType(ENTITY))
  {
    math::Pose relative_pose(pose - this->parentEntity->GetWorldPose());
    this->SetRelativePose(relative_pose, notify);
  }
  else
  {
    // no parent, relative pose is the absolute world pose
    this->SetRelativePose(pose, notify);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the position of the entity relative to its parent
void Entity::SetRelativePosition(const math::Vector3 &pos)
{
  this->SetRelativePose( math::Pose( pos, this->GetRelativePose().rot), true );
}

////////////////////////////////////////////////////////////////////////////////
/// Set the rotation of the entity relative to its parent
void Entity::SetRelativeRotation(const math::Quaternion &rot)
{
  this->SetRelativePose( math::Pose( this->GetRelativePose().pos, rot), true );
}

////////////////////////////////////////////////////////////////////////////////
// Handle a change of pose
void Entity::UpdatePhysicsPose(bool update_children)
{
  this->OnPoseChange();

  if (update_children)
  {
    for  (Base_V::iterator iter = this->children.begin();
          iter != this->children.end(); iter++)
    {
      if ((*iter)->HasType(ENTITY))
        boost::shared_static_cast<Entity>(*iter)->OnPoseChange();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the parent model, if one exists
ModelPtr Entity::GetParentModel() const
{
  BasePtr p = this->parent;

  while (p && p->HasType(MODEL))
    p = p->GetParent();

  return boost::shared_dynamic_cast<Model>(p);
}
