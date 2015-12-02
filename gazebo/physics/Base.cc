/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
// We also include this winsock2 trick in Base.hh but it is used last,
// so we need it again here.
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/World.hh"

#include "gazebo/physics/BasePrivate.hh"
#include "gazebo/physics/Base.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Base::Base(BasePtr _parent)
: parent(_parent)
{
  this->dataPtr->type = BASE;
  this->dataPtr->id = physics::getUniqueId();
  this->dataPtr->saveable = true;
  this->dataPtr->selected = false;

  this->dataPtr->sdf.reset(new sdf::Element);
  this->dataPtr->sdf->AddAttribute("name", "string", "__default__", true);
  this->dataPtr->name = "__default__";

  if (this->dataPtr->parent)
  {
    this->dataPtr->world = this->dataPtr->parent->GetWorld();
  }
}

//////////////////////////////////////////////////
Base::~Base()
{
  // remove self as a child of the parent
  if (this->dataPtr->parent)
    this->dataPtr->parent->RemoveChild(this->dataPtr->id);

  this->SetParent(BasePtr());

  for (Base_V::iterator iter = this->dataPtr->children.begin();
       iter != this->dataPtr->children.end(); ++iter)
  {
    if (*iter)
      (*iter)->SetParent(BasePtr());
  }
  this->dataPtr->children.clear();
  if (this->dataPtr->sdf)
    this->dataPtr->sdf->Reset();
  this->dataPtr->sdf.reset();
}

//////////////////////////////////////////////////
void Base::Load(sdf::ElementPtr _sdf)
{
  if (_sdf)
    this->dataPtr->sdf = _sdf;

  GZ_ASSERT(this->dataPtr->sdf != NULL, "this->dataPtr->sdf is NULL");

  if (this->dataPtr->sdf->HasAttribute("name"))
    this->dataPtr->name = this->dataPtr->sdf->Get<std::string>("name");
  else
    this->dataPtr->name.clear();

  if (this->dataPtr->parent)
  {
    this->dataPtr->world = this->dataPtr->parent->GetWorld();
    this->dataPtr->parent->AddChild(shared_from_this());
  }

  this->ComputeScopedName();
}

//////////////////////////////////////////////////
void Base::UpdateParameters(sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_sdf != NULL, "_sdf parameter is NULL");
  GZ_ASSERT(this->dataPtr->sdf != NULL, "Base sdf member is NULL");
  this->dataPtr->sdf->Copy(_sdf);
}

//////////////////////////////////////////////////
void Base::Fini()
{
  Base_V::iterator iter;

  for (iter = this->dataPtr->children.begin(); iter != this->dataPtr->children.end(); ++iter)
    if (*iter)
      (*iter)->Fini();

  this->dataPtr->children.clear();

  this->dataPtr->world.reset();
  this->dataPtr->parent.reset();
}

//////////////////////////////////////////////////
void Base::Reset()
{
}

//////////////////////////////////////////////////
void Base::Reset(Base::EntityType _resetType)
{
  Base_V::iterator iter;
  for (iter = this->dataPtr->children.begin(); iter != this->dataPtr->children.end(); ++iter)
  {
    if ((*iter)->HasType(_resetType))
      (*iter)->Reset();

    (*iter)->Reset(_resetType);
  }
}

//////////////////////////////////////////////////
void Base::SetName(const std::string &_name)
{
  GZ_ASSERT(this->dataPtr->sdf != NULL, "Base sdf member is NULL");
  GZ_ASSERT(this->dataPtr->sdf->GetAttribute("name"),
      "Base sdf missing name attribute");
  this->dataPtr->sdf->GetAttribute("name")->Set(_name);
  this->dataPtr->name = _name;
  this->ComputeScopedName();
}

//////////////////////////////////////////////////
std::string Base::GetName() const
{
  return this->Name();
}

//////////////////////////////////////////////////
std::string Base::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
uint32_t Base::GetId() const
{
  return this->Id();
}

//////////////////////////////////////////////////
uint32_t Base::Id() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
void Base::SetSaveable(const bool _v)
{
  this->dataPtr->saveable = _v;
}

//////////////////////////////////////////////////
bool Base::GetSaveable() const
{
  return this->Saveable();
}

//////////////////////////////////////////////////
bool Base::Saveable() const
{
  return this->dataPtr->saveable;
}

//////////////////////////////////////////////////
int Base::GetParentId() const
{
  return this->ParentId();
}

//////////////////////////////////////////////////
int Base::ParentId() const
{
  return this->dataPtr->parent == NULL ? 0 : this->dataPtr->parent->Id();
}

//////////////////////////////////////////////////
void Base::SetParent(BasePtr _parent)
{
  this->dataPtr->parent = _parent;
}

//////////////////////////////////////////////////
BasePtr Base::GetParent() const
{
  return this->Parent();
}

//////////////////////////////////////////////////
BasePtr Base::Parent() const
{
  return this->dataPtr->parent;
}

//////////////////////////////////////////////////
void Base::AddChild(BasePtr _child)
{
  if (_child == NULL)
    gzthrow("Cannot add a null _child to an entity");

  // Add this _child to our list
  this->dataPtr->children.push_back(_child);
}

//////////////////////////////////////////////////
void Base::RemoveChild(unsigned int _id)
{
  Base_V::iterator iter;
  for (iter = this->dataPtr->children.begin(); iter != this->dataPtr->children.end(); ++iter)
  {
    if ((*iter)->GetId() == _id)
    {
      (*iter)->Fini();
      this->dataPtr->children.erase(iter);
      break;
    }
  }
}

//////////////////////////////////////////////////
unsigned int Base::GetChildCount() const
{
  return this->ChildCount();
}

//////////////////////////////////////////////////
unsigned int Base::ChildCount() const
{
  return this->dataPtr->children.size();
}

//////////////////////////////////////////////////
void Base::AddType(Base::EntityType _t)
{
  this->dataPtr->type = this->dataPtr->type | (unsigned int)_t;
}

//////////////////////////////////////////////////
BasePtr Base::GetChild(unsigned int _i) const
{
  return this->Child(_i);
}

//////////////////////////////////////////////////
BasePtr Base::Child(const unsigned int _i) const
{
  if (_i < this->dataPtr->children.size())
    return this->dataPtr->children[_i];

  return BasePtr();
}

//////////////////////////////////////////////////
BasePtr Base::GetChild(const std::string &_name)
{
  return this->Child(_name);
}

//////////////////////////////////////////////////
BasePtr Base::Child(const std::string &_name) const
{
  std::string fullName = this->ScopedName() + "::" + _name;
  return this->BaseByName(fullName);
}

//////////////////////////////////////////////////
void Base::RemoveChild(const std::string &_name)
{
  Base_V::iterator iter;

  for (iter = this->dataPtr->children.begin(); iter != this->dataPtr->children.end(); ++iter)
  {
    if ((*iter)->GetScopedName() == _name)
      break;
  }

  if (iter != this->dataPtr->children.end())
  {
    (*iter)->Fini();
    this->dataPtr->children.erase(iter);
  }
}

//////////////////////////////////////////////////
void Base::RemoveChildren()
{
  this->dataPtr->children.clear();
}

//////////////////////////////////////////////////
BasePtr Base::GetById(unsigned int _id) const
{
  return this->BaseById(_id);
}

//////////////////////////////////////////////////
BasePtr Base::BaseById(const unsigned int _id) const
{
  BasePtr result;
  Base_V::const_iterator biter;

  for (biter = this->dataPtr->children.begin();
       biter != this->dataPtr->children.end(); ++biter)
  {
    if ((*biter)->GetId() == _id)
    {
      result = *biter;
      break;
    }
  }

  return result;
}

//////////////////////////////////////////////////
BasePtr Base::GetByName(const std::string &_name)
{
  return this->BaseByName(_name);
}

//////////////////////////////////////////////////
BasePtr Base::BaseByName(const std::string &_name) const
{
  if (this->ScopedName() == _name || this->Name() == _name)
    return shared_from_this();

  BasePtr result;

  for (auto iter = this->dataPtr->children.begin();
       iter != this->dataPtr->children.end() && result == NULL; ++iter)
  {
    result = (*iter)->BaseByName(_name);
  }

  return result;
}

//////////////////////////////////////////////////
std::string Base::GetScopedName(bool _prependWorldName) const
{
  return this->ScopedName(_prependWorldName);
}

//////////////////////////////////////////////////
std::string Base::ScopedName(bool _prependWorldName) const
{
  if (_prependWorldName && this->dataPtr->world)
    return this->dataPtr->world->GetName() + "::" + this->dataPtr->scopedName;
  else
    return this->dataPtr->scopedName;
}

//////////////////////////////////////////////////
void Base::ComputeScopedName()
{
  BasePtr p = this->dataPtr->parent;
  this->dataPtr->scopedName = this->GetName();

  while (p)
  {
    if (p->GetParent())
      this->dataPtr->scopedName.insert(0, p->GetName()+"::");
    p = p->GetParent();
  }
}

//////////////////////////////////////////////////
bool Base::HasType(const Base::EntityType &_t) const
{
  return ((unsigned int)(_t & this->dataPtr->type) == (unsigned int)_t);
}

//////////////////////////////////////////////////
unsigned int Base::GetType() const
{
  return this->Type();
}

//////////////////////////////////////////////////
unsigned int Base::Type() const
{
  return this->dataPtr->type;
}

//////////////////////////////////////////////////
void Base::Print(const std::string &_prefix)
{
  Base_V::iterator iter;
  gzmsg << _prefix << this->GetName() << "\n";

  for (iter = this->dataPtr->children.begin(); iter != this->dataPtr->children.end(); ++iter)
    (*iter)->Print(_prefix + "  ");
}

//////////////////////////////////////////////////
bool Base::SetSelected(const bool _s)
{
  this->dataPtr->selected = _s;

  Base_V::iterator iter;
  for (iter = this->dataPtr->children.begin(); iter != this->dataPtr->children.end(); ++iter)
    (*iter)->SetSelected(_s);

  return true;
}

//////////////////////////////////////////////////
bool Base::IsSelected() const
{
  return this->dataPtr->selected;
}

//////////////////////////////////////////////////
bool Base::operator ==(const Base &ent) const
{
  return ent.GetName() == this->GetName();
}

//////////////////////////////////////////////////
void Base::SetWorld(const WorldPtr &_newWorld)
{
  this->dataPtr->world = _newWorld;

  Base_V::iterator iter;
  for (iter = this->dataPtr->children.begin(); iter != this->dataPtr->children.end(); ++iter)
  {
    (*iter)->SetWorld(this->dataPtr->world);
  }
}

//////////////////////////////////////////////////
const WorldPtr &Base::GetWorld() const
{
  return this->World();
}

//////////////////////////////////////////////////
const WorldPtr &Base::World() const
{
  return this->dataPtr->world;
}

//////////////////////////////////////////////////
const sdf::ElementPtr Base::SDF() const
{
  GZ_ASSERT(this->dataPtr->sdf != NULL, "Base sdf member is NULL");
  this->dataPtr->sdf->Update();
  return this->dataPtr->sdf;
}
