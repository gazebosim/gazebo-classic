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

/* Desc: Base class shared by all classes in Gazebo.
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
 * SVN: $Id$
 */

#include "World.hh"
#include "Common.hh"
#include "Model.hh"
#include "World.hh"
#include "GazeboMessage.hh"
#include "GazeboError.hh"

using namespace gazebo;

unsigned int Common::idCounter = 0;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Common::Common(Common *parent)
 : parent(parent)
{
  this->world = NULL;

  this->AddType(COMMON);

  if (this->parent)
    this->world = this->parent->GetWorld();

  this->id = ++idCounter;

  Param::Begin(&this->parameters);
  this->nameP = new ParamT<std::string>("name","noname",1);
  Param::End();

  this->saveable = true;

  if (this->parent)
    this->parent->AddChild(this);

  this->showInGui = true;
  this->selected = false;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Common::~Common()
{
  // remove self as a child of the parent
  if (this->parent)
    this->parent->RemoveChild(this);

  this->SetParent(NULL);

  std::vector<Common*>::iterator iter;

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if (*iter)
    {
      (*iter)->SetParent(NULL);
      delete *iter;
    }
  }

  delete this->nameP;
}

////////////////////////////////////////////////////////////////////////////////
/// Load 
void Common::Load(XMLConfigNode *node)
{
  this->nameP->Load(node);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the name of the entity
void Common::SetName(const std::string &name)
{
  this->nameP->SetValue( name );
}
  
////////////////////////////////////////////////////////////////////////////////
/// Return the name of the entity
std::string Common::GetName() const
{
  return this->nameP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the count of the parameters
unsigned int Common::GetParamCount() const
{
  return this->parameters.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a param by index
Param *Common::GetParam(unsigned int index) const
{
  if (index < this->parameters.size())
    return this->parameters[index];
  else
    gzerr(0) << "Invalid index[" << index << "]\n";
  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a parameter by name
Param *Common::GetParam(const std::string &key) const
{
  std::vector<Param*>::const_iterator iter;
  Param *result = NULL;

  for (iter = this->parameters.begin(); iter != this->parameters.end(); iter++)
  {
    if ((*iter)->GetKey() == key)
    {
      result = *iter;
      break;
    }
  }

  if (result == NULL)
    gzerr(0) << "Unable to find Param using key[" << key << "]\n";

  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Set a parameter by name
void Common::SetParam(const std::string &key, const std::string &value)
{
  std::vector<Param*>::const_iterator iter;
  Param *result = NULL;

  for (iter = this->parameters.begin(); iter != this->parameters.end(); iter++)
  {
    if ((*iter)->GetKey() == key)
    {
      result = *iter;
      break;
    }
  }

  if (result == NULL)
    gzerr(0) << "Unable to find Param using key[" << key << "]\n";
  else
    result->SetFromString( value, true );
}
 
////////////////////////////////////////////////////////////////////////////////
/// Return the ID of this entity. This id is unique
int Common::GetId() const
{
  return this->id;
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether the object should be "saved", when the user
///        selects to save the world to xml
void Common::SetSaveable(bool v)
{
  this->saveable = v;
}

////////////////////////////////////////////////////////////////////////////////
/// Get whether the object should be "saved", when the user
/// selects to save the world to xml
bool Common::GetSaveable() const
{
  return this->saveable;
}

////////////////////////////////////////////////////////////////////////////////
// Return the ID of the parent
int Common::GetParentId() const
{
  return this->parent == NULL ? 0 : this->parent->GetId();
}

////////////////////////////////////////////////////////////////////////////////
// Set the parent
void Common::SetParent(Common *parent)
{
  this->parent = parent;
}

////////////////////////////////////////////////////////////////////////////////
// Get the parent
Common *Common::GetParent() const
{
  return this->parent;
}

////////////////////////////////////////////////////////////////////////////////
// Add a child to this entity
void Common::AddChild(Common *child)
{
  if (child == NULL)
    gzthrow("Cannot add a null child to an entity");

  // Add this child to our list
  this->children.push_back(child);
}

////////////////////////////////////////////////////////////////////////////////
/// Remove a child from this entity
void Common::RemoveChild(Common *child)
{
  std::vector<Common*>::iterator iter;
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
///  Get the number of children
unsigned int Common::GetChildCount() const
{
  return this->children.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Add a type specifier
void Common::AddType( EntityType t )
{
  this->type.push_back(t);
}

////////////////////////////////////////////////////////////////////////////////
/// Get a child by index
Common *Common::GetChild(unsigned int i) const
{
  if (i < this->children.size())
    return this->children[i];
  
  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a child by name
Common *Common::GetChild(const std::string &name )
{
  std::string fullName = this->GetCompleteScopedName() + "::" + name;
  return this->GetByName(fullName);
}

////////////////////////////////////////////////////////////////////////////////
// Get by name helper
Common *Common::GetByName(const std::string &name)
{
  if (this->GetCompleteScopedName() == name)
    return this;

  Common *result = NULL;
  std::vector<Common*>::const_iterator iter;

  for (iter =  this->children.begin(); 
       iter != this->children.end() && result ==NULL; iter++)
    result = (*iter)->GetByName(name);

  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Return the name of this entity with the model scope
/// model1::...::modelN::entityName
std::string Common::GetScopedName() const
{
  Common *p = this->parent;
  std::string scopedName = this->GetName();

  while (p)
  {
    if (p && p->HasType(MODEL))
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
std::string Common::GetCompleteScopedName() const
{
  Common *p = this->parent;
  std::string scopedName = this->GetName();

  while (p)
  {
    scopedName.insert(0, p->GetName()+"::");
    p = p->GetParent();
  }

  return scopedName;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the type
bool Common::HasType(const EntityType &t) const
{
  return std::binary_search(this->type.begin(), this->type.end(), t);

  /*for (unsigned int i=0; i < this->type.size(); i++)
    if (this->type[i] == t)
      return true;

  return false;
  */
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of types
unsigned int Common::GetTypeCount() const
{
  return this->type.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a type by index
EntityType Common::GetType(unsigned int index) const
{
  if (index < this->type.size())
    return this->type[index];

  gzthrow("Invalid type index");
  return COMMON;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the leaf type (last type set)
EntityType Common::GetLeafType() const
{
  return this->type.back();
}

////////////////////////////////////////////////////////////////////////////////
void Common::Print(std::string prefix)
{
  std::vector<Common*>::iterator iter;
  std::cout << prefix << this->GetName() << "\n";

  prefix += "  ";
  for (iter = this->children.begin(); iter != this->children.end(); iter++)
    (*iter)->Print(prefix);
}

////////////////////////////////////////////////////////////////////////////////
/// True == show parameters in the gui
bool Common::GetShowInGui() const
{
  return this->showInGui;
}

////////////////////////////////////////////////////////////////////////////////
/// True == show parameters in the gui
void Common::SetShowInGui(bool v)
{
  this->showInGui = v;
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether this entity has been selected by the user through the gui
bool Common::SetSelected( bool s )
{
  std::vector< Common *>::iterator iter;

  this->selected = s;

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
    (*iter)->SetSelected(s);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// True if the entity is selected by the user
bool Common::IsSelected() const
{
  return this->selected;
}


////////////////////////////////////////////////////////////////////////////////
/// Get the parent model, if one exists
Model *Common::GetParentModel() const
{
  Common *p = this->parent;

  while (p && p->HasType(MODEL))
    p = p->GetParent();

  return (Model*)p;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the world this object belongs to. This will also set the world for all 
/// children
void Common::SetWorld(World *newWorld)
{
  this->world = newWorld;

  for (unsigned int i=0; i < this->children.size(); i++)
    this->children[i]->SetWorld(this->world);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the world this object is in
World *Common::GetWorld() const
{
  return this->world;
}

////////////////////////////////////////////////////////////////////////////////
/// Returns true if the entities are the same. Checks only the name
bool Common::operator==(const Common &ent) const 
{
  return ent.GetName() == this->GetName();
}
