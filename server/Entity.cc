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

#include "Geom.hh"
#include "Model.hh"
#include "Body.hh"
#include "GazeboError.hh"
#include "Global.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "World.hh"
#include "PhysicsEngine.hh"
#include "Entity.hh"
#include "Simulator.hh"

using namespace gazebo;


////////////////////////////////////////////////////////////////////////////////
// Constructor
Entity::Entity(Entity *parent)
: Common(), parent(parent), visualNode(0)
{
  this->type = DEFAULT;

  Param::Begin(&this->parameters);
  this->staticP = new ParamT<bool>("static",false,0);
  Param::End();
 
  this->selected = false;

  std::ostringstream visname;
  visname << "Entity_" << this->GetId() << "_VISUAL";

  if (this->parent)
  {
    this->parent->AddChild(this);

    if (Simulator::Instance()->GetRenderEngineEnabled())
    {
      this->visualNode = OgreCreator::Instance()->CreateVisual(
          visname.str(), this->parent->GetVisualNode(), this);
    }
    this->SetStatic(parent->IsStatic());
  }
  else
  {
    if (Simulator::Instance()->GetRenderEngineEnabled())
    {
      this->visualNode = OgreCreator::Instance()->CreateVisual( 
          visname.str(), NULL, this );
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Entity::~Entity()
{
  // remove self as a child of the parent
  if (this->parent)
    this->parent->RemoveChild(this);

  // remove all connected joints for a Body before delteing it
   /*gazebo::Model* parent_model = dynamic_cast<gazebo::Model*>(this->parent);
   if (parent_model)
     parent_model->DeleteConnectedJoints(this);
     */

  this->SetParent(NULL);

  delete this->staticP;

  std::vector<Entity*>::iterator iter;

  World::Instance()->GetPhysicsEngine()->RemoveEntity(this);

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if (*iter)
    {
      (*iter)->SetParent(NULL);

      World::Instance()->GetPhysicsEngine()->RemoveEntity(*iter);

      if (*iter && (*iter)->GetType() == Entity::MODEL)
      {
        Model *m = (Model*)*iter;
        m->Detach();
      }

      delete *iter;
    }
  }

  if (Simulator::Instance()->GetRenderEngineEnabled())
  {
    if (this->visualNode)
    {
      OgreCreator::Instance()->DeleteVisual(this->visualNode);
    }
  }

  this->visualNode = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Return the ID of the parent
int Entity::GetParentId() const
{
  return this->parent == NULL ? 0 : this->parent->GetId();
}

////////////////////////////////////////////////////////////////////////////////
// Set the parent
void Entity::SetParent(Entity *_parent)
{
  if (this->parent)
  {
    this->parent->visualNode->sceneNode->removeChild(this->visualNode->sceneNode);
    this->visualNode->parentNode = NULL;
  }

  if (_parent)
  {
    _parent->visualNode->sceneNode->addChild(this->visualNode->sceneNode);
    this->visualNode->parentNode = _parent->visualNode->sceneNode;
  }

  this->parent = _parent;
}

////////////////////////////////////////////////////////////////////////////////
// Get the parent
Entity *Entity::GetParent() const
{
  return this->parent;
}

////////////////////////////////////////////////////////////////////////////////
// Add a child to this entity
void Entity::AddChild(Entity *child)
{
  if (child == NULL)
    gzthrow("Cannot add a null child to an entity");

  // Add this child to our list
  this->children.push_back(child);
}

////////////////////////////////////////////////////////////////////////////////
/// Remove a child from this entity
void Entity::RemoveChild(Entity *child)
{
  std::vector<Entity*>::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if ((*iter)->GetName() == child->GetName())
    {
      this->children.erase(iter);
      break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Get all children
const std::vector< Entity* > &Entity::GetChildren() const
{
  return this->children;
}

////////////////////////////////////////////////////////////////////////////////
///  Get the number of children
unsigned int Entity::GetChildCount() const
{
  return this->children.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a child by index
Entity *Entity::GetChild(unsigned int i)
{
  if (i < this->children.size())
    return this->children[i];
  
  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a child by name
Entity *Entity::GetChild(const std::string &name )
{
  std::string fullName = this->GetCompleteScopedName() + "::" + name;
  return World::Instance()->GetEntityByName(fullName);
}
 
////////////////////////////////////////////////////////////////////////////////
// Return this entitie's sceneNode
OgreVisual *Entity::GetVisualNode() const
{
  return this->visualNode;
}

////////////////////////////////////////////////////////////////////////////////
// Set the scene node
void Entity::SetVisualNode(OgreVisual *visualNode)
{
  this->visualNode = visualNode;
}

////////////////////////////////////////////////////////////////////////////////
// Set whether this entity is static: immovable
void Entity::SetStatic(const bool &s)
{
  std::vector< Entity *>::iterator iter;
  Body *body = NULL;

  this->staticP->SetValue( s );

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if ( (*iter)->IsStatic())
      continue;

    (*iter)->SetStatic(s);
    if (*iter && (*iter)->GetType() == Entity::BODY)
    {
      body = (Body*)*iter;
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
/// Set whether this entity has been selected by the user through the gui
bool Entity::SetSelected( bool s )
{
  std::vector< Entity *>::iterator iter;

  this->selected = s;

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
    (*iter)->SetSelected(s);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// True if the entity is selected by the user
bool Entity::IsSelected() const
{
  return this->selected;
}

////////////////////////////////////////////////////////////////////////////////
/// Returns true if the entities are the same. Checks only the name
bool Entity::operator==(const Entity &ent) const 
{
  return ent.GetName() == this->GetName();
}

////////////////////////////////////////////////////////////////////////////////
/// Return the name of this entity with the model scope
/// model1::...::modelN::entityName
std::string Entity::GetScopedName() const
{
  Entity *p = this->parent;
  std::string scopedName = this->GetName();

  while (p)
  {
    if (p && p->GetType() == Entity::MODEL)
    {
      Model *m = (Model*)p;
      scopedName.insert(0, m->GetName()+"::");
    }
    p = p->GetParent();
  }

  return scopedName;
}

////////////////////////////////////////////////////////////////////////////////
/// Return the name of this entity with the model scope
/// model1::...::modelN::entityName
std::string Entity::GetCompleteScopedName() const
{
  Entity *p = this->parent;
  std::string scopedName = this->GetName();

  while (p)
  {
    scopedName.insert(0, p->GetName()+"::");
    p = p->GetParent();
  }

  return scopedName;
}

// TODO: THIS IS A HACK AND SHOULD BE REMOVED
std::string Entity::GetCompleteScopedAltName() const
{
  gzthrow("This is bad\n");
  /*Entity *p = this->parent;
  std::string scopedName = this->GetAltName();

  while (p)
  {
    scopedName.insert(0, p->GetName()+"::");
    p = p->GetParent();
  }

  return scopedName;
  */
  return std::string();
}


////////////////////////////////////////////////////////////////////////////////
/// Get the absolute pose of the entity
Pose3d Entity::GetWorldPose() const
{
  if (this->parent)
  {
    //std::cout << " GetWorldPose for model " << this->GetName()
    //          << " relative " << this->GetRelativePose()
    //          << " parent-abs " << this->parent->GetWorldPose() << std::endl;
    return this->GetRelativePose() + this->parent->GetWorldPose();
  }
  else
  {
    //std::cout << " GetWorldPose for model " << this->GetName()
    //          << " relative " << this->GetRelativePose() << std::endl;
    return this->GetRelativePose();
  }
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
  if (this->type == MODEL || !this->parent)
    return Pose3d();

  return this->GetRelativePose() + this->parent->GetModelRelativePose();
}

////////////////////////////////////////////////////////////////////////////////
// Set the abs pose of the entity
void Entity::SetWorldPose(const Pose3d &pose, bool notify)
{
  if (this->parent)
  {
    // if this is the canonical body of a model, then
    // we want to SetWorldPose of the parent model
    // by doing some backwards transform
    if (this->parent->type == MODEL &&
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
      this->parent->SetWorldPose(model_abs_pose,false);
      // that should be all, as relative pose of a canonical model
      // should not change
    }
    else
    {
      // this is not a canonical Body of a model
      // simply update it's own RelativePose
      Pose3d relative_pose = pose - this->parent->GetWorldPose();
      // relative pose is the pose relative to the parent
      // if this is called from MoveCallback, notify is false
      // FIXME: if this is called by user, and notify is true
      //        use may end up updating the entire model trying
      //        to set abs pose of a body?  no, the body has no
      //        children
      this->SetRelativePose(relative_pose, notify);
    }
  }
  else if (this->type == MODEL)
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
  if (Simulator::Instance()->GetRenderEngineEnabled())
  {
    if (Simulator::Instance()->GetState() == Simulator::RUN &&
        !this->IsStatic())
      this->visualNode->SetDirty(true, this->GetRelativePose());
    else
      this->visualNode->SetPose(this->GetRelativePose());
  }

  if (notify)
  {
    this->OnPoseChange();

    std::vector<Entity*>::iterator iter;
    for  (iter = this->children.begin(); iter != this->children.end(); iter++)
      (*iter)->OnPoseChange();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the parent model, if one exists
Model *Entity::GetParentModel() const
{
  Entity *p = this->parent;

  while (p && p->type != MODEL)
    p = p->GetParent();

  return (Model*)p;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the type of this entity
void Entity::SetType(Entity::Type type)
{
  this->type = type;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the type of this entity
Entity::Type Entity::GetType() const
{
  return this->type;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the type as a string
std::string Entity::GetTypeString() const
{
  static std::string typenames[] = {"default", "model", "body", "geom", "light"};
  return typenames[this->type];
}


////////////////////////////////////////////////////////////////////////////////
void Entity::Print(std::string prefix)
{
  std::vector<Entity*>::iterator iter;
  std::cout << prefix << "Name[" << this->GetName() << "] Alt[";
  for (unsigned int i=0; i < this->altNames.size(); i++)
    std::cout << this->altNames[i] << ", ";
  std::cout << "]\n";

  prefix += "  ";
  for (iter = this->children.begin(); iter != this->children.end(); iter++)
    (*iter)->Print(prefix);
}

////////////////////////////////////////////////////////////////////////////////
/// A helper routine to make objects static. This is called at the very end of initialize at the model level
void Entity::MakeStatic()
{
  if (this->visualNode)
    this->visualNode->MakeStatic();
}
