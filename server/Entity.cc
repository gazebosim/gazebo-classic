/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Desc: External interfaces for Gazebo
 * Author: Nate Koenig
 * Date: 03 Apr 2007
 * SVN: $Id$
 */

#include "Messages.hh"
#include "Geom.hh"
#include "Model.hh"
#include "Body.hh"
#include "GazeboError.hh"
#include "Global.hh"
#include "World.hh"
#include "PhysicsEngine.hh"
#include "Entity.hh"
#include "Simulator.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Entity::Entity(Common *parent)
: Common(parent)
{
  this->AddType(ENTITY);

  Param::Begin(&this->parameters);
  this->staticP = new ParamT<bool>("static",false,0);
  this->staticP->Callback( &Entity::SetStatic, this );
  Param::End();
 
  this->visualMsg = new VisualMsg();
  //this->visualMsg->id = this->GetName();

  if (this->parent && this->parent->HasType(ENTITY))
  {
    Entity *ep = (Entity*)(this->parent);
    this->SetStatic(ep->IsStatic());
  }
  /*else
  {
    this->visualMsg = new VisualMsg();
  }

  Simulator::Instance()->SendMessage( *this->visualMsg );
  */

  // NATY: put functionality back in
  //this->visualNode->SetOwner(this);
}

////////////////////////////////////////////////////////////////////////////////
/// Load
void Entity::Load(XMLConfigNode *node)
{
  Common::Load(node);
  this->RegisterVisual();
}
 

void Entity::SetName(const std::string &name)
{
  // TODO: if an entitie's name is changed, then the old visual is never
  // removed. Should add in functionality to modify/update the visual
  Common::SetName(name);
  //this->visualMsg->id = this->GetCompleteScopedName();
  //Simulator::Instance()->SendMessage( *this->visualMsg );
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

  this->visualMsg->action = VisualMsg::DELETE;
  Simulator::Instance()->SendMessage( *this->visualMsg );
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
  return Box(Vector3(0,0,0), Vector3(1,1,1));
}

////////////////////////////////////////////////////////////////////////////////
/// Get the absolute pose of the entity
Pose3d Entity::GetWorldPose() const
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
Pose3d Entity::GetRelativePose() const
{
  return this->relativePose;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the pose of the entity relative to its parent
void Entity::SetRelativePose(const Pose3d &pose, bool notify)
{
  this->relativePose = pose;
  this->relativePose.Correct();
  this->PoseChange(notify);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the pose relative to the model this entity belongs to
Pose3d Entity::GetModelRelativePose() const
{
  if (this->HasType(MODEL) || !this->parent)
    return Pose3d();

  return this->GetRelativePose() + 
         ((Entity*)this->parent)->GetModelRelativePose();
}

////////////////////////////////////////////////////////////////////////////////
// Set the abs pose of the entity
void Entity::SetWorldPose(const Pose3d &pose, bool notify)
{
  if (this->parent && this->parent->HasType(ENTITY))
  {
    // if this is the canonical body of a model, then
    // we want to SetWorldPose of the parent model
    // by doing some backwards transform
    if (this->parent->HasType(MODEL) && 
        ((Model*)this->parent)->GetCanonicalBody() == (Body*)this)
    {
      // abs pose of the model + relative pose of cb = abs pose of cb 
      // so to get abs pose of the model, we need
      // start with abs pose of cb, inverse rotate by relative pose of cb
      //  then, inverse translate by relative pose of cb
      Pose3d model_abs_pose;
      Pose3d cb_rel_pose = this->GetRelativePose();
      // anti-rotate cb-abs by cb-rel = model-abs
      model_abs_pose.rot = pose.rot * this->GetRelativePose().rot.GetInverse();
      // rotate cb-rel pos by cb-rel rot to get pos offset
      //Vector3 pos_offset = cb->GetRelativePose().rot.GetInverse() * cb->GetRelativePose().pos;
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
    {
      // this is not a canonical Body of a model
      // simply update it's own RelativePose
      Pose3d relative_pose = pose - ((Entity*)this->parent)->GetWorldPose();
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
void Entity::SetRelativePosition(const Vector3 &pos)
{
  this->SetRelativePose( Pose3d( pos, this->GetRelativePose().rot), true );
}

////////////////////////////////////////////////////////////////////////////////
/// Set the rotation of the entity relative to its parent
void Entity::SetRelativeRotation(const Quatern &rot)
{
  this->SetRelativePose( Pose3d( this->GetRelativePose().pos, rot), true );
}

////////////////////////////////////////////////////////////////////////////////
// Handle a change of pose
void Entity::PoseChange(bool notify)
{
  PoseMsg msg;
  msg.id = this->GetCompleteScopedName();
  //std::cout << "Pose[" << msg.id << "]\n";
  msg.pose = this->GetRelativePose();
  Simulator::Instance()->SendMessage( msg );

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
  this->visualMsg->id = this->GetCompleteScopedName();
  if (this->parent)
    this->visualMsg->parentId = this->parent->GetCompleteScopedName();
  Simulator::Instance()->SendMessage( *this->visualMsg );
}
