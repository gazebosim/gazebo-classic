/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include "gazebo/util/IntrospectionManager.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Base.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Base::Base(BasePtr _parent)
: parent(_parent)
{
  this->type = BASE;
  this->id = physics::getUniqueId();
  this->typeStr = "base";
  this->saveable = true;
  this->selected = false;

  this->sdf.reset(new sdf::Element);
  this->sdf->AddAttribute("name", "string", "__default__", true);
  this->name = "__default__";

  if (this->parent)
  {
    this->world = this->parent->GetWorld();
  }
}

//////////////////////////////////////////////////
Base::~Base()
{
  this->Fini();
}

//////////////////////////////////////////////////
void Base::Load(sdf::ElementPtr _sdf)
{
  if (_sdf)
    this->sdf = _sdf;

  GZ_ASSERT(this->sdf != NULL, "this->sdf is NULL");

  if (this->sdf->HasAttribute("name"))
    this->name = this->sdf->Get<std::string>("name");
  else
    this->name.clear();

  if (this->parent)
  {
    this->world = this->parent->GetWorld();
    this->parent->AddChild(shared_from_this());
  }

  this->ComputeScopedName();

  this->RegisterIntrospectionItems();
}

//////////////////////////////////////////////////
void Base::UpdateParameters(sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_sdf != NULL, "_sdf parameter is NULL");
  GZ_ASSERT(this->sdf != NULL, "Base sdf member is NULL");
  this->sdf->Copy(_sdf);
}

//////////////////////////////////////////////////
void Base::Fini()
{
  this->UnregisterIntrospectionItems();

  // Remove self as a child of the parent
  if (this->parent)
  {
    auto temp = this->parent;
    this->parent.reset();

    temp->RemoveChild(this->id);
  }

  // Also destroy all children.
  while (!this->children.empty())
  {
    auto child = this->children.front();
    this->RemoveChild(child);
  }
  this->children.clear();

  this->sdf.reset();

  this->world.reset();
}

//////////////////////////////////////////////////
void Base::Reset()
{
}

//////////////////////////////////////////////////
void Base::Reset(Base::EntityType _resetType)
{
  Base_V::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(_resetType))
      (*iter)->Reset();

    (*iter)->Reset(_resetType);
  }
}

//////////////////////////////////////////////////
void Base::SetName(const std::string &_name)
{
  GZ_ASSERT(this->sdf != NULL, "Base sdf member is NULL");
  GZ_ASSERT(this->sdf->GetAttribute("name"), "Base sdf missing name attribute");
  this->sdf->GetAttribute("name")->Set(_name);
  this->name = _name;
  this->ComputeScopedName();
}

//////////////////////////////////////////////////
std::string Base::GetName() const
{
  return this->name;
}

//////////////////////////////////////////////////
uint32_t Base::GetId() const
{
  return this->id;
}

//////////////////////////////////////////////////
void Base::SetSaveable(bool _v)
{
  this->saveable = _v;
}

//////////////////////////////////////////////////
bool Base::GetSaveable() const
{
  return this->saveable;
}

//////////////////////////////////////////////////
int Base::GetParentId() const
{
  return this->parent == NULL ? 0 : this->parent->GetId();
}

//////////////////////////////////////////////////
void Base::SetParent(BasePtr _parent)
{
  this->parent = _parent;
}

//////////////////////////////////////////////////
BasePtr Base::GetParent() const
{
  return this->parent;
}

//////////////////////////////////////////////////
void Base::AddChild(BasePtr _child)
{
  if (_child == NULL)
    gzthrow("Cannot add a null _child to an entity");

  // Add this _child to our list
  if (std::find(this->children.begin(), this->children.end(), _child)
      == this->children.end())
  {
    this->children.push_back(_child);
  }
}

//////////////////////////////////////////////////
void Base::RemoveChild(unsigned int _id)
{
  this->RemoveChild(this->GetById(_id));
}

//////////////////////////////////////////////////
unsigned int Base::GetChildCount() const
{
  return this->children.size();
}

//////////////////////////////////////////////////
void Base::AddType(Base::EntityType _t)
{
  this->type = this->type | (unsigned int)_t;

  if (this->type & MODEL)
    this->typeStr = "model";
  else if (this->type & LINK)
    this->typeStr = "link";
  else if (this->type & COLLISION)
    this->typeStr = "collision";
  else if (this->type & ACTOR)
    this->typeStr = "actor";
  else if (this->type & LIGHT)
    this->typeStr = "light";
  else if (this->type & VISUAL)
    this->typeStr = "visual";
  else if (this->type & JOINT)
    this->typeStr = "joint";
  else if (this->type & SHAPE)
    this->typeStr = "shape";
}

//////////////////////////////////////////////////
BasePtr Base::GetChild(unsigned int _i) const
{
  if (_i < this->children.size())
    return this->children[_i];

  return BasePtr();
}

//////////////////////////////////////////////////
BasePtr Base::GetChild(const std::string &_name)
{
  std::string fullName = this->GetScopedName() + "::" + _name;
  return this->GetByName(fullName);
}

//////////////////////////////////////////////////
void Base::RemoveChild(const std::string &_name)
{
  this->RemoveChild(this->GetByName(_name));
}

//////////////////////////////////////////////////
void Base::RemoveChild(physics::BasePtr _child)
{
  if (!_child)
    return;

  // Fini
  _child->SetParent(nullptr);
  _child->Fini();

  // Remove from vector if still there
  this->children.erase(std::remove(this->children.begin(),
                                   this->children.end(), _child),
                                   this->children.end());
}

//////////////////////////////////////////////////
void Base::RemoveChildren()
{
  this->children.clear();
}

//////////////////////////////////////////////////
BasePtr Base::GetById(unsigned int _id) const
{
  BasePtr result;
  Base_V::const_iterator biter;

  for (biter = this->children.begin(); biter != this->children.end(); ++biter)
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
  if (this->GetScopedName() == _name || this->GetName() == _name)
    return shared_from_this();

  BasePtr result;
  Base_V::const_iterator iter;

  for (iter = this->children.begin();
      iter != this->children.end() && result == NULL; ++iter)
    result = (*iter)->GetByName(_name);

  return result;
}

//////////////////////////////////////////////////
std::string Base::GetScopedName(bool _prependWorldName) const
{
  if (_prependWorldName && this->world)
    return this->world->GetName() + "::" + this->scopedName;
  else
    return this->scopedName;
}

//////////////////////////////////////////////////
common::URI Base::URI() const
{
  common::URI uri;

  uri.SetScheme("data");

  BasePtr p = this->parent;
  while (p)
  {
    if (p->GetParent())
    {
      uri.Path().PushFront(p->GetName());
      uri.Path().PushFront(p->TypeStr());
    }

    p = p->GetParent();
  }

  uri.Path().PushBack(this->TypeStr());
  uri.Path().PushBack(this->GetName());
  uri.Path().PushFront(this->world->GetName());
  uri.Path().PushFront("world");

  return uri;
}

/////////////////////////////////////////////////
void Base::RegisterIntrospectionItems()
{
  // nothing for now
}

/////////////////////////////////////////////////
void Base::UnregisterIntrospectionItems()
{
  for (auto &item : this->introspectionItems)
    util::IntrospectionManager::Instance()->Unregister(item.Str());

  this->introspectionItems.clear();
}

//////////////////////////////////////////////////
void Base::ComputeScopedName()
{
  BasePtr p = this->parent;
  this->scopedName = this->GetName();

  while (p)
  {
    if (p->GetParent())
      this->scopedName.insert(0, p->GetName()+"::");
    p = p->GetParent();
  }
}

//////////////////////////////////////////////////
bool Base::HasType(const Base::EntityType &_t) const
{
  return ((unsigned int)(_t & this->type) == (unsigned int)_t);
}

//////////////////////////////////////////////////
unsigned int Base::GetType() const
{
  return this->type;
}

//////////////////////////////////////////////////
std::string Base::TypeStr() const
{
  return this->typeStr;
}

//////////////////////////////////////////////////
void Base::Print(const std::string &_prefix)
{
  Base_V::iterator iter;
  gzmsg << _prefix << this->GetName() << "\n";

  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
    (*iter)->Print(_prefix + "  ");
}

//////////////////////////////////////////////////
bool Base::SetSelected(bool _s)
{
  this->selected = _s;

  Base_V::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
    (*iter)->SetSelected(_s);

  return true;
}

//////////////////////////////////////////////////
bool Base::IsSelected() const
{
  return this->selected;
}

//////////////////////////////////////////////////
bool Base::operator ==(const Base &ent) const
{
  return ent.GetName() == this->GetName();
}

//////////////////////////////////////////////////
void Base::SetWorld(const WorldPtr &_newWorld)
{
  this->world = _newWorld;

  Base_V::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    (*iter)->SetWorld(this->world);
  }
}

//////////////////////////////////////////////////
const WorldPtr &Base::GetWorld() const
{
  return this->world;
}

//////////////////////////////////////////////////
const sdf::ElementPtr Base::GetSDF()
{
  GZ_ASSERT(this->sdf != NULL, "Base sdf member is NULL");
  this->sdf->Update();
  return this->sdf;
}
