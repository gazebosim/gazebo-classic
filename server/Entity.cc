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

////////////////////////////////////////////////////////////////////////////////
/// Get the absolute pose of the entity
Pose3d Entity::GetAbsPose() const
{
  if (this->parent)
  {
    //std::cout << " GetAbsPose for model " << this->GetName()
    //          << " relative " << this->GetRelativePose()
    //          << " parent-abs " << this->parent->GetAbsPose() << std::endl;
    return this->GetRelativePose() + this->parent->GetAbsPose();
  }
  else
  {
    //std::cout << " GetAbsPose for model " << this->GetName()
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
