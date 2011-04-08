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

/* Desc: Base class shared by all classes in Gazebo.
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
 */


#include "common/Console.hh"
#include "physics/World.hh"
#include "physics/Base.hh"

using namespace gazebo;
using namespace physics;

unsigned int Base::idCounter = 0;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Base::Base(BasePtr parent)
 : parent(parent)
{
  this->AddType(BASE);
  this->id = ++idCounter;
  this->saveable = true;
  this->selected = false;

  common::Param::Begin(&this->parameters);
  this->nameP = new common::ParamT<std::string>("name","noname",1);
  common::Param::End();

}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Base::~Base()
{
  gzmsg << "DELETEING BASE[" << this->GetName() << "]\n";

  // remove self as a child of the parent
  if (this->parent)
    this->parent->RemoveChild(this->id);

  this->SetParent(BasePtr());

  for (Base_V::iterator iter = this->children.begin(); 
       iter != this->children.end(); iter++)
  {
    if (*iter)
      (*iter)->SetParent(BasePtr());
  }
  this->children.clear();

  delete this->nameP;
}

////////////////////////////////////////////////////////////////////////////////
/// Load 
void Base::Load(common::XMLConfigNode *node)
{
  this->nameP->Load(node);

  gzmsg << "Load Base[" << this->GetName() << "]";

  if (this->parent)
  {
    gzmsg << "   Has parent\n";
    this->world = this->parent->GetWorld();
    this->parent->AddChild(shared_from_this());
  }
  else
    gzmsg << "   No parent\n";
}

////////////////////////////////////////////////////////////////////////////////
// Save function
void Base::Save(const std::string &prefix, std::ostream &stream)
{
}

////////////////////////////////////////////////////////////////////////////////
/// Finialize the object
void Base::Fini()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Set the name of the entity
void Base::SetName(const std::string &name)
{
  this->nameP->SetDefaultValue(name);
  this->nameP->SetValue( name );
}
  
////////////////////////////////////////////////////////////////////////////////
/// Return the name of the entity
std::string Base::GetName() const
{
  return **this->nameP;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the count of the parameters
unsigned int Base::GetParamCount() const
{
  return this->parameters.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a param by index
common::Param *Base::GetParam(unsigned int index) const
{
  if (index < this->parameters.size())
    return this->parameters[index];
  else
    gzerr << "Invalid index[" << index << "]\n";
  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a parameter by name
common::Param *Base::GetParam(const std::string &key) const
{
  common::Param_V::const_iterator iter;
  common::Param *result = NULL;

  for (iter = this->parameters.begin(); iter != this->parameters.end(); iter++)
  {
    if ((*iter)->GetKey() == key)
    {
      result = *iter;
      break;
    }
  }

  if (result == NULL)
    gzerr << "Unable to find Param using key[" << key << "]\n";

  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Return the ID of this entity. This id is unique
int Base::GetId() const
{
  return this->id;
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether the object should be "saved", when the user
///        selects to save the world to xml
void Base::SetSaveable(bool v)
{
  this->saveable = v;
}

////////////////////////////////////////////////////////////////////////////////
/// Get whether the object should be "saved", when the user
/// selects to save the world to xml
bool Base::GetSaveable() const
{
  return this->saveable;
}

////////////////////////////////////////////////////////////////////////////////
// Return the ID of the parent
int Base::GetParentId() const
{
  return this->parent == NULL ? 0 : this->parent->GetId();
}

////////////////////////////////////////////////////////////////////////////////
// Set the parent
void Base::SetParent(BasePtr parent)
{
  this->parent = parent;
}

////////////////////////////////////////////////////////////////////////////////
// Get the parent
BasePtr Base::GetParent() const
{
  return this->parent;
}

////////////////////////////////////////////////////////////////////////////////
// Add a child to this entity
void Base::AddChild(BasePtr child)
{
  if (child == NULL)
    gzthrow("Cannot add a null child to an entity");

  // Add this child to our list
  this->children.push_back(child);
}

////////////////////////////////////////////////////////////////////////////////
/// Remove a child from this entity
void Base::RemoveChild(unsigned int _id)
{
  Base_V::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if ((*iter)->GetId() == _id)
    {
      this->children.erase(iter);
      break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
///  Get the number of children
unsigned int Base::GetChildCount() const
{
  return this->children.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Add a type specifier
void Base::AddType( Base::EntityType t )
{
  this->type.push_back(t);
}

////////////////////////////////////////////////////////////////////////////////
/// Get a child by index
BasePtr Base::GetChild(unsigned int i) const
{
  if (i < this->children.size())
    return this->children[i];
  
  return BasePtr();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a child by name
BasePtr Base::GetChild(const std::string &name )
{
  std::string fullName = this->GetCompleteScopedName() + "::" + name;
  return this->GetByName(fullName);
}

////////////////////////////////////////////////////////////////////////////////
// Get by name helper
BasePtr Base::GetByName(const std::string &name)
{
  if (this->GetCompleteScopedName() == name)
    return shared_from_this();

  BasePtr result;
  Base_V::const_iterator iter;

  for (iter =  this->children.begin(); 
       iter != this->children.end() && result ==NULL; iter++)
    result = (*iter)->GetByName(name);

  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Return the name of this entity with the model scope
/// model1::...::modelN::entityName
std::string Base::GetScopedName() const
{
  BasePtr p = this->parent;
  std::string scopedName = this->GetName();

  while (p)
  {
    scopedName.insert(0, p->GetName()+"::");
    p = p->GetParent();
  }

  return scopedName;
}

////////////////////////////////////////////////////////////////////////////////
/// Return the name of this entity with the model scope
/// model1::...::modelN::entityName
std::string Base::GetCompleteScopedName() const
{
  BasePtr p = this->parent;
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
bool Base::HasType(const Base::EntityType &t) const
{
  return std::binary_search(this->type.begin(), this->type.end(), t);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of types
unsigned int Base::GetTypeCount() const
{
  return this->type.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a type by index
Base::EntityType Base::GetType(unsigned int index) const
{
  if (index < this->type.size())
    return this->type[index];

  gzthrow("Invalid type index");
  return BASE;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the leaf type (last type set)
Base::EntityType Base::GetLeafType() const
{
  return this->type.back();
}

////////////////////////////////////////////////////////////////////////////////
void Base::Print(std::string prefix)
{
  Base_V::iterator iter;
  gzmsg << prefix << this->GetName() << "\n";

  prefix += "  ";
  for (iter = this->children.begin(); iter != this->children.end(); iter++)
    (*iter)->Print(prefix);
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether this entity has been selected by the user through the gui
bool Base::SetSelected( bool s )
{
  this->selected = s;

  Base_V::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); iter++)
    (*iter)->SetSelected(s);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// True if the entity is selected by the user
bool Base::IsSelected() const
{
  return this->selected;
}

////////////////////////////////////////////////////////////////////////////////
/// Returns true if the entities are the same. Checks only the name
bool Base::operator==(const Base &ent) const 
{
  return ent.GetName() == this->GetName();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the world this object belongs to. This will also set the world for all 
/// children
void Base::SetWorld(WorldPtr newWorld)
{
  this->world = newWorld;

  Base_V::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    (*iter)->SetWorld(this->world);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the world this object is in
WorldPtr Base::GetWorld() const
{
  return this->world;
}
