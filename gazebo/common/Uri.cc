/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <memory>
#include <string>
#include <vector>
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Uri.hh"

using namespace gazebo;
using namespace common;

namespace gazebo
{
  namespace common
  {
    /// \internal
    /// \brief UriEntityPart private data.
    class UriEntityPartPrivate
    {
      /// \brief ToDo.
      public: std::string type;

      /// \brief ToDo.
      public: std::string name;

      /// \brief ToDo.
      public: std::shared_ptr<UriEntityPart> children = nullptr;
    };

    /// \internal
    /// \brief UriParts private data.
    class UriPartsPrivate
    {
      /// \brief ToDo.
      public: std::string world;

      /// \brief ToDo.
      public: UriEntityPart entity;

      /// \brief ToDo.
      public: std::vector<std::string> parameters;
    };
  }
}

//////////////////////////////////////////////////
UriEntityPart::UriEntityPart()
  : dataPtr(new UriEntityPartPrivate())
{
}

//////////////////////////////////////////////////
UriEntityPart::~UriEntityPart()
{
}

//////////////////////////////////////////////////
std::string UriEntityPart::Type() const
{
  return this->dataPtr->type;
}

//////////////////////////////////////////////////
std::string UriEntityPart::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
std::shared_ptr<UriEntityPart> UriEntityPart::Children() const
{
  return this->dataPtr->children;
}

//////////////////////////////////////////////////
void UriEntityPart::SetType(const std::string &_type)
{
  this->dataPtr->type = _type;
}

//////////////////////////////////////////////////
void UriEntityPart::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
void UriEntityPart::SetChildren(const std::shared_ptr<UriEntityPart> &_children)
{
  this->dataPtr->children = _children;
}

//////////////////////////////////////////////////
UriEntityPart &UriEntityPart::operator=(const UriEntityPart &_p)
{
  this->SetName(_p.Name());
  this->SetType(_p.Type());
  this->dataPtr->children.reset();

  // Deep copy.
  auto parent = this;
  auto p = _p.Children();
  while (p)
  {
    // Allocate a new child.
    auto newChild = std::make_shared<UriEntityPart>();
    newChild->SetName(p->Name());
    newChild->SetType(p->Type());
    parent->SetChildren(newChild);

    p = p->Children();
    parent = newChild.get();
  }

  return *this;
}

//////////////////////////////////////////////////
UriParts::UriParts()
  : dataPtr(new UriPartsPrivate())
{
}

//////////////////////////////////////////////////
UriParts::~UriParts()
{
}

//////////////////////////////////////////////////
std::string UriParts::World() const
{
  return this->dataPtr->world;
}

//////////////////////////////////////////////////
UriEntityPart &UriParts::Entity() const
{
  return this->dataPtr->entity;
}

//////////////////////////////////////////////////
std::vector<std::string> &UriParts::Parameters() const
{
  return this->dataPtr->parameters;
}

//////////////////////////////////////////////////
void UriParts::SetWorld(const std::string &_world)
{
  this->dataPtr->world = _world;
}

//////////////////////////////////////////////////
void UriParts::SetEntity(const UriEntityPart &_entity)
{
  this->dataPtr->entity = _entity;
}

//////////////////////////////////////////////////
void UriParts::SetParameters(const std::vector<std::string> &_params)
{
  this->dataPtr->parameters = _params;
}

//////////////////////////////////////////////////
UriParts &UriParts::operator=(const UriParts &_p)
{
  this->SetWorld(_p.World());
  this->SetEntity(_p.Entity());
  this->SetParameters(_p.Parameters());

  return *this;
}

//////////////////////////////////////////////////
Uri::~Uri()
{
}

//////////////////////////////////////////////////
bool Uri::Parse(const std::string &_uri, UriParts &_parts)
{
  std::string world;
  size_t next;
  UriEntityPart entity;

  std::cout << "About to parse world" << std::endl;

  if (!ParseWorld(_uri, world, next))
    return false;

  std::cout << "World looks good! [" << world << "]" << std::endl;

  if (!ParseEntity(_uri, next, entity))
    return false;

  // ToDo: Parse parameters.

  _parts.SetWorld(world);
  _parts.SetEntity(entity);
  return true;
}

//////////////////////////////////////////////////
bool Uri::ParseWorld(const std::string &_uri, std::string &_world,
    size_t &_next)
{
  // Sanity check: Make sure that there are no white spaces.
  if (_uri.find(" ") != std::string::npos)
    return false;

  const std::string kDelimWorld = "/world/";
  auto start = _uri.find(kDelimWorld);
  if (start != 0)
    return false;

  std::cout << "/world/ was found" << std::endl;

  auto from = kDelimWorld.size();
  auto to = _uri.find("/", from);
  if (to == std::string::npos)
    return false;

  _world = _uri.substr(from, to - from);
  _next = to + 1;
  return true;
}

//////////////////////////////////////////////////
bool Uri::ParseEntity(const std::string &_uri, const size_t &_from,
    UriEntityPart &_entity)
{
  size_t from = _from;
  size_t next;
  UriEntityPart *entity = &_entity;

  while (true)
  {
    if (!ParseOneEntity(_uri, from, *entity, next))
      return false;

    if (next == _uri.size())
    {
      std::cout << "We're done" << std::endl;
      return true;
    }

    std::cout << "Good pair!" << std::endl;
    std::cout << "  Type: [" << entity->Type() << "]" << std::endl;
    std::cout << "  Name: [" << entity->Name() << "]" << std::endl;

    from = next;
    entity->Children().reset(new UriEntityPart());
    if (!entity->Children())
      std::cerr << "entity is NULL" << std::endl;
    entity = entity->Children().get();

  }

  return true;
}

//////////////////////////////////////////////////
bool Uri::ParseOneEntity(const std::string &_uri, const size_t &_from,
    UriEntityPart &_entity, size_t &_next)
{
  std::cout << "ParseOneEntity()" << std::endl;
  std::cout << "  uri: " << _uri.substr(_from) << std::endl;
  auto next = _uri.find("/", _from);
  if (next == std::string::npos)
    return false;

  std::cout << "Setting type (" << _uri.substr(_from, next - _from) << ")" << std::endl;
  _entity.SetType(_uri.substr(_from, next - _from));

  std::cout << "type: [" << _entity.Type() << "]" << std::endl;

  next += 1;
  auto to = _uri.find("/", next);
  if (to == std::string::npos)
  {
    std::cout << "End / not found" << std::endl;
    if (next == _uri.size())
    {
      // No entity name after the "/".
      std::cout << "1" << std::endl;
      return false;
    }
    else
    {
      _entity.SetName(_uri.substr(next, _uri.size() - next + 1));
      _next = _uri.size();
      std::cout << "2" << std::endl;
      std::cout << "Name: [" << _entity.Name() << "]" << std::endl;
      return true;
    }
  }
  else
  {
    _entity.SetName(_uri.substr(next, to - next));
    _next = to + 1;
    std::cout << "3" << std::endl;
    return true;
  }
}
