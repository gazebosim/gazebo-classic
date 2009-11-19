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
  Param::Begin(&this->parameters);
  this->staticP = new ParamT<bool>("static",false,0);
  //this->staticP->Callback( &Entity::SetStatic, this);
  Param::End();
 
  this->selected = false;

  std::ostringstream visname;
  visname << "Entity_" << this->GetId() << "_VISUAL";

  if (this->parent)
  {
    this->parent->AddChild(this);

    if (Simulator::Instance()->GetRenderEngineEnabled())
      this->visualNode = OgreCreator::Instance()->CreateVisual(
          visname.str(), this->parent->GetVisualNode(), this);
    this->SetStatic(parent->IsStatic());
  }
  else
  {
    if (Simulator::Instance()->GetRenderEngineEnabled())
      this->visualNode = OgreCreator::Instance()->CreateVisual( 
          visname.str(), NULL, this );
  }

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Entity::~Entity()
{
  delete this->staticP;

  World::Instance()->GetPhysicsEngine()->RemoveEntity(this);

  if (Simulator::Instance()->GetRenderEngineEnabled())
    if (this->visualNode)
      OgreCreator::Instance()->DeleteVisual(this->visualNode);
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
void Entity::SetParent(Entity *parent)
{
  this->parent = parent;
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
// Get all children
std::vector< Entity* > &Entity::GetChildren() 
{
  return this->children;
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
    (*iter)->SetStatic(s);
    body = dynamic_cast<Body*>(*iter);
    if (body)
      body->SetEnabled(!s);
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
  Body *body = NULL;

  this->selected = s;

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    (*iter)->SetSelected(s);
    body = dynamic_cast<Body*>(*iter);
  }

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
/// Return true if the entity is a geom
bool Entity::IsGeom() const
{
  return dynamic_cast<const Geom*>(this) != NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Return true if the entity is a body
bool Entity::IsBody() const
{
  return dynamic_cast<const Body*>(this) != NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Return true if the entity is a model
bool Entity::IsModel() const
{
  return dynamic_cast<const Model*>(this) != NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Return the name of this entity with the model scope
/// model1::...::modelN::entityName
std::string Entity::GetScopedName()
{
  Entity *p = this->parent;
  std::string scopedName = this->GetName();

  while (p)
  {
    Model *m = dynamic_cast<Model*>(p);
    if (m)
      scopedName.insert(0, m->GetName()+"::");
    p = p->GetParent();
  }

  return scopedName;
}

////////////////////////////////////////////////////////////////////////////////
/// Return the name of this entity with the model scope
/// model1::...::modelN::entityName
std::string Entity::GetCompleteScopedName()
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

////////////////////////////////////////////////////////////////////////////////
/// Get the absolute pose of the entity
Pose3d Entity::GetAbsPose() const
{
  if (this->parent)
    return this->relativePose + this->parent->GetAbsPose();
  else
    return this->relativePose;
}

////////////////////////////////////////////////////////////////////////////////
// Get the pose of the entity relative to its parent
Pose3d Entity::GetRelativePose() const
{
  return this->relativePose;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the pose relative to the model this entity belongs to
Pose3d Entity::GetModelRelativePose() const
{
  if (this->IsModel() || !this->parent)
    return Pose3d();

  return this->relativePose + this->parent->GetModelRelativePose();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the pose of the entity relative to its parent
void Entity::SetRelativePose(const Pose3d &pose, bool notify)
{
  this->relativePose = pose;
  this->PoseChange(notify);
}

////////////////////////////////////////////////////////////////////////////////
// Set the abs pose of the entity
void Entity::SetAbsPose(const Pose3d &pose, bool notify)
{
  Pose3d p = pose;
  if (this->parent)
    p -= this->parent->GetAbsPose();

  this->SetRelativePose(p, notify);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the position of the entity relative to its parent
void Entity::SetRelativePosition(const Vector3 &pos)
{
  this->relativePose.pos = pos;
  this->PoseChange();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the rotation of the entity relative to its parent
void Entity::SetRelativeRotation(const Quatern &rot)
{
  this->relativePose.rot = rot;
  this->PoseChange();
}

////////////////////////////////////////////////////////////////////////////////
// Handle a change of pose
void Entity::PoseChange(bool notify)
{
  if (Simulator::Instance()->GetState() == Simulator::RUN)// || this->IsStatic())
    this->visualNode->SetDirty(true, this->relativePose);
  else
    this->visualNode->SetPose(this->relativePose);

  if (notify)
  {
    this->OnPoseChange();

    std::vector<Entity*>::iterator iter;
    for  (iter = this->children.begin(); iter != this->children.end(); iter++)
      (*iter)->OnPoseChange();
  }
}
