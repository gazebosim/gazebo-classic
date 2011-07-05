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
#include "physics/Body.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/Entity.hh"

using namespace gazebo;
using namespace physics;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Entity::Entity(BasePtr parent)
  : Base(parent)
{
  this->node = transport::NodePtr(new transport::Node());
  this->AddType(ENTITY);

  this->visualMsg = new msgs::Visual;
  this->poseMsg = new msgs::Pose;

  this->visualMsg->set_mesh_type( msgs::Visual::UNKNOWN );

  if (this->parent && this->parent->HasType(ENTITY))
  {
    this->parentEntity = boost::shared_dynamic_cast<Entity>(this->parent);
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
  this->posePub = this->node->Advertise<msgs::Pose>("~/pose");
  this->visPub = this->node->Advertise<msgs::Visual>("~/visual");

  this->visualMsg->mutable_header()->set_str_id(this->GetCompleteScopedName());

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

////////////////////////////////////////////////////////////////////////////////
/// Set the pose of the entity relative to its parent
void Entity::SetRelativePose(const math::Pose &pose, bool notify)
{
  this->relativePose = pose;
  this->relativePose.Correct();
  this->PoseChange(notify);
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
    // TODO: I took out the first if clause because it would cause other
    // bodies in a model to penetrate the ground since they were not the 
    // canonical body
    // if this is the canonical body of a model, then
    // we want to SetWorldPose of the parent model
    // by doing some backwards transform
    /*if (this->parent->HasType(MODEL) && 
        boost::shared_static_cast<Model>(this->parent)->GetBody("canonical")->GetId() == this->GetId())
    {
      // abs pose of the model + relative pose of cb = abs pose of cb 
      // so to get abs pose of the model, we need
      // start with abs pose of cb, inverse rotate by relative pose of cb
      //  then, inverse translate by relative pose of cb
      math::Pose model_abs_pose;
      math::Pose cb_rel_pose = this->GetRelativePose();
      // anti-rotate cb-abs by cb-rel = model-abs
      model_abs_pose.rot = pose.rot * this->GetRelativePose().rot.GetInverse();
      // rotate cb-rel pos by cb-rel rot to get pos offset
      //math::Vector3 pos_offset = cb->GetRelativePose().rot.GetInverse() * cb->GetRelativePose().pos;
      // finally, model-abs pos is cb-abs pos - pos_offset
      //model_abs_pose.pos = cb->GetWorldPose().pos - pos_offset;
      model_abs_pose.pos = pose.pos - model_abs_pose.rot * this->GetRelativePose().pos;
      //model_abs_pose.pos = pose.pos - this->GetRelativePose().pos;
      // set abs pose of parent model without propagating
      // changes to children
      boost::shared_static_cast<Entity>(this->parent)->SetWorldPose(model_abs_pose,false);
      // that should be all, as relative pose of a canonical model
      // should not change
    }
    else
    */
    {
      // this is not a canonical Body of a model
      // simply update it's own RelativePose
      math::Pose relative_pose = pose - this->parentEntity->GetWorldPose();
      // relative pose is the pose relative to the parent
      // if this is called from MoveCallback, notify is false
      // FIXME: if this is called by user, and notify is true
      //        use may end up updating the entire model trying
      //        to set abs pose of a body?  no, the body has no
      //        children
      this->SetRelativePose(relative_pose, notify);
    }
  }
  else if (this->HasType(MODEL))
  {
    // race condition with MoveCallback from canonical body calling SetWorldPose
    // we need to stop canonicalBody from calling SetWorldPose in MoveCallback
    // so this user request is not overwritten
    // if this is a model with no parent,
    // then set own relative pose as incoming
    // pose and notify all children
    //  this has to propagate through before MoveCallback
    //  triggers another SetWorldPose() and overwrites this change
    this->SetRelativePose(pose, notify);
    //then, set abs pose of the canonical body
    //Body* cb = ((Model*)this)->GetBody("canonical");
  }
  else
  {
    gzerr << "No parent and not a model, strange\n";
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
void Entity::PoseChange(bool notify)
{
  msgs::Pose msg;
  msgs::Set( this->poseMsg, this->GetRelativePose());
  this->posePub->Publish( *this->poseMsg);

  if (notify)
  {
    this->OnPoseChange();

    Base_V::iterator iter;
    for  (iter = this->children.begin(); iter != this->children.end(); iter++)
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
