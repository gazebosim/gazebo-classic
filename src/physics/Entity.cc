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
 * SVN: $Id$
 */

#include "common/Messages.hh"
#include "physics/Geom.hh"
#include "Model.hh"
#include "Body.hh"
#include "common/GazeboError.hh"
#include "common/Global.hh"
#include "PhysicsEngine.hh"
#include "Entity.hh"
#include "transport/TopicManager.hh"

using namespace gazebo;
using namespace physics;


////////////////////////////////////////////////////////////////////////////////
// Constructor
Entity::Entity(Common *parent)
: Common(parent)
{
  this->pose_pub = transport::TopicManager::Instance()->Advertise<msgs::Pose>("/gazebo/pose");
  this->vis_pub = transport::TopicManager::Instance()->Advertise<msgs::Visual>("/gazebo/visual");
  this->AddType(ENTITY);

  common::Param::Begin(&this->parameters);
  this->staticP = new common::ParamT<bool>("static",false,0);
  this->staticP->Callback( &Entity::SetStatic, this );
  common::Param::End();
 
  this->visualMsg = new msgs::Visual();

  if (this->parent && this->parent->HasType(ENTITY))
  {
    Entity *ep = (Entity*)(this->parent);
    this->SetStatic(ep->IsStatic());
  }

  // NATY: put functionality back in
  //this->visualNode->SetOwner(this);
}

////////////////////////////////////////////////////////////////////////////////
/// Load
void Entity::Load(common::XMLConfigNode *node)
{
  Common::Load(node);
  this->RegisterVisual();
}
 

void Entity::SetName(const std::string &name)
{
  // TODO: if an entitie's name is changed, then the old visual is never
  // removed. Should add in functionality to modify/update the visual
  Common::SetName(name);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Entity::~Entity()
{
  // remove all connected joints for a Body before delteing it
   /*gazebo::Model* parent_model = dynamic_cast<gazebo::Model*>(this->parent);
   if (parent_model)
     parent_model->DeleteConnectedJoints(this);
     */

  delete this->staticP;

  std::vector<Common*>::iterator iter;

  this->GetWorld()->GetPhysicsEngine()->RemoveEntity(this);

  /*for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if (*iter && (*iter)->HasType(ENTITY))
    {
      Entity *child = (Entity*)(*iter);
      child->SetParent(NULL);
      this->GetWorld()->GetPhysicsEngine()->RemoveEntity(child);

      if (child->HasType(MODEL))
      {
        Model *m = (Model*)child;
        m->Detach();
      }
      //delete *iter;
    }
  }*/

  this->visualMsg->set_action( msgs::Visual::DELETE );
  this->vis_pub->Publish(*this->visualMsg);
  delete this->visualMsg;
  this->visualMsg = NULL;
}


////////////////////////////////////////////////////////////////////////////////
// Set whether this entity is static: immovable
void Entity::SetStatic(const bool &s)
{
  std::vector< Common *>::iterator iter;
  Body *body = NULL;

  this->staticP->SetValue( s );

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if (! (*iter)->HasType(ENTITY))
      continue;

    Entity *ent = (Entity*)(*iter);
    if (!ent || ent->IsStatic())
      continue;

    ent->SetStatic(s);
    if (ent->HasType(BODY))
    {
      body = (Body*)ent;
      body->SetEnabled(!s);
    }
  }

}

////////////////////////////////////////////////////////////////////////////////
// Return whether this entity is static
bool Entity::IsStatic() const
{
  return this->staticP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Return the bounding box for the entity 
Box Entity::GetBoundingBox() const
{
  return Box(common::Vector3(0,0,0), common::Vector3(1,1,1));
}

////////////////////////////////////////////////////////////////////////////////
/// Get the absolute pose of the entity
common::Pose3d Entity::GetWorldPose() const
{
  if (this->parent && this->parent->HasType(ENTITY))
  {
    Entity *ent = (Entity*)this->parent;
    return this->GetRelativePose() + ent->GetWorldPose();
  }
  else
    return this->GetRelativePose();
}

////////////////////////////////////////////////////////////////////////////////
// Get the pose of the entity relative to its parent
common::Pose3d Entity::GetRelativePose() const
{
  return this->relativePose;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the pose of the entity relative to its parent
void Entity::SetRelativePose(const common::Pose3d &pose, bool notify)
{
  this->relativePose = pose;
  this->relativePose.Correct();
  this->PoseChange(notify);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the pose relative to the model this entity belongs to
common::Pose3d Entity::GetModelRelativePose() const
{
  if (this->HasType(MODEL) || !this->parent)
    return common::Pose3d();

  return this->GetRelativePose() + 
         ((Entity*)this->parent)->GetModelRelativePose();
}

////////////////////////////////////////////////////////////////////////////////
// Set the abs pose of the entity
void Entity::SetWorldPose(const common::Pose3d &pose, bool notify)
{

  if (this->parent && this->parent->HasType(ENTITY))
  {
    // NATY: I took out the first if clause because it would cause other
    // bodies in a model to penetrate the ground since they were not the 
    // canonical body
    // if this is the canonical body of a model, then
    // we want to SetWorldPose of the parent model
    // by doing some backwards transform
    /*if (this->parent->HasType(MODEL) && 
        ((Model*)this->parent)->GetCanonicalBody() == (Body*)this)
    {
      // abs pose of the model + relative pose of cb = abs pose of cb 
      // so to get abs pose of the model, we need
      // start with abs pose of cb, inverse rotate by relative pose of cb
      //  then, inverse translate by relative pose of cb
      common::Pose3d model_abs_pose;
      common::Pose3d cb_rel_pose = this->GetRelativePose();
      // anti-rotate cb-abs by cb-rel = model-abs
      model_abs_pose.rot = pose.rot * this->GetRelativePose().rot.GetInverse();
      // rotate cb-rel pos by cb-rel rot to get pos offset
      //common::Vector3 pos_offset = cb->GetRelativePose().rot.GetInverse() * cb->GetRelativePose().pos;
      // finally, model-abs pos is cb-abs pos - pos_offset
      //model_abs_pose.pos = cb->GetWorldPose().pos - pos_offset;
      model_abs_pose.pos = pose.pos - model_abs_pose.rot * this->GetRelativePose().pos;
      //model_abs_pose.pos = pose.pos - this->GetRelativePose().pos;
      // set abs pose of parent model without propagating
      // changes to children
      ((Entity*)this->parent)->SetWorldPose(model_abs_pose,false);
      // that should be all, as relative pose of a canonical model
      // should not change
    }
    else
      */

    {
      // this is not a canonical Body of a model
      // simply update it's own RelativePose
      common::Pose3d relative_pose = pose - ((Entity*)this->parent)->GetWorldPose();
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
    //Body* cb = ((Model*)this)->GetCanonicalBody();
  }
  else
  {
    std::cerr << "No parent and not a model, strange\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the position of the entity relative to its parent
void Entity::SetRelativePosition(const common::Vector3 &pos)
{
  this->SetRelativePose( common::Pose3d( pos, this->GetRelativePose().rot), true );
}

////////////////////////////////////////////////////////////////////////////////
/// Set the rotation of the entity relative to its parent
void Entity::SetRelativeRotation(const common::Quatern &rot)
{
  this->SetRelativePose( common::Pose3d( this->GetRelativePose().pos, rot), true );
}

////////////////////////////////////////////////////////////////////////////////
// Handle a change of pose
void Entity::PoseChange(bool notify)
{
  msgs::Pose msg;
  Message::Init(msg, this->GetCompleteScopedName() );
  Message::Set( &msg, this->GetRelativePose());
  this->pose_pub->Publish(msg);

  if (notify)
  {
    this->OnPoseChange();

    std::vector<Common*>::iterator iter;
    for  (iter = this->children.begin(); iter != this->children.end(); iter++)
    {
      if ((*iter)->HasType(ENTITY))
        ((Entity*)*iter)->OnPoseChange();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Register a visual
void Entity::RegisterVisual()
{
  this->visualMsg->mutable_header()->set_str_id(this->GetCompleteScopedName());

  if (this->parent)
    this->visualMsg->set_parent_id( this->parent->GetCompleteScopedName() );

  this->vis_pub->Publish(*this->visualMsg);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the parent model, if one exists
Model *Entity::GetParentModel() const
{
  Common *p = this->parent;

  while (p && p->HasType(MODEL))
    p = p->GetParent();

  return (Model*)p;
}
