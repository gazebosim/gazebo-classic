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
    class UriEntityPrivate
    {
      /// \brief URI entity type.
      public: std::string type;

      /// \brief URI entity name.
      public: std::string name;
    };

    /// \internal
    /// \brief UriNestedEntity private data.
    class UriNestedEntityPrivate
    {
      /// \brief Nested URI entities.
      public: std::vector<UriEntity> entities;
    };

    /// \internal
    /// \brief UriParts private data.
    class UriPartsPrivate
    {
      /// \brief World name.
      public: std::string world;

      /// \brief URI nested entity.
      public: UriNestedEntity entity;

      /// \brief Parameter.
      public: std::string parameter;
    };

    /// \internal
    /// \brief Uri private data.
    class UriPrivate
    {
      /// \brief The individual parts of the URI.
      public: UriParts parts;

      /// \brief The string representation of the URI.
      public: std::string canonicalUri;

      /// \brief True when the object is storing a valid URI.
      public: bool correct = false;
    };
  }
}

//////////////////////////////////////////////////
UriEntity::UriEntity()
  : dataPtr(new UriEntityPrivate())
{
}

//////////////////////////////////////////////////
UriEntity::UriEntity(const std::string &_type, const std::string &_name)
  : UriEntity()
{
  this->SetType(_type);
  this->SetName(_name);
}

//////////////////////////////////////////////////
UriEntity::UriEntity(const UriEntity &_entity)
  : UriEntity()
{
  *this = _entity;
}

//////////////////////////////////////////////////
UriEntity::~UriEntity()
{
}

//////////////////////////////////////////////////
std::string UriEntity::Type() const
{
  return this->dataPtr->type;
}

//////////////////////////////////////////////////
std::string UriEntity::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void UriEntity::SetType(const std::string &_type)
{
  this->Validate(_type);
  this->dataPtr->type = _type;
}

//////////////////////////////////////////////////
void UriEntity::SetName(const std::string &_name)
{
  this->Validate(_name);
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
UriEntity &UriEntity::operator=(const UriEntity &_p)
{
  this->SetType(_p.Type());
  this->SetName(_p.Name());

  return *this;
}

//////////////////////////////////////////////////
void UriEntity::Validate(const std::string &_identifier)
{
  if (_identifier.find_first_of(" ?=&") != std::string::npos)
    gzthrow("Invalid URI entity identifier");
}

//////////////////////////////////////////////////
UriNestedEntity::UriNestedEntity()
  : dataPtr(new UriNestedEntityPrivate())
{
}

//////////////////////////////////////////////////
UriNestedEntity::UriNestedEntity(const UriNestedEntity &_entity)
  : UriNestedEntity()
{
  *this = _entity;
}

//////////////////////////////////////////////////
UriNestedEntity::~UriNestedEntity()
{
}

//////////////////////////////////////////////////
UriEntity UriNestedEntity::Parent() const
{
  if (this->dataPtr->entities.empty())
    gzthrow("Empty nested entity");

  return this->dataPtr->entities.front();
}

//////////////////////////////////////////////////
UriEntity UriNestedEntity::Leaf() const
{
  if (this->dataPtr->entities.empty())
    gzthrow("Empty nested entity");

 return this->dataPtr->entities.back();
}

//////////////////////////////////////////////////
UriEntity UriNestedEntity::Entity(const unsigned int &_index) const
{
  if (_index >= this->dataPtr->entities.size())
    gzthrow("Incorrect index accessing a nested entity");

  return this->dataPtr->entities.at(_index);
}

//////////////////////////////////////////////////
unsigned int UriNestedEntity::EntityCount() const
{
  return this->dataPtr->entities.size();
}

//////////////////////////////////////////////////
void UriNestedEntity::AddEntity(const UriEntity &_entity)
{
  this->dataPtr->entities.push_back(_entity);
}

//////////////////////////////////////////////////
void UriNestedEntity::AddParentEntity(const UriEntity &_entity)
{
  this->dataPtr->entities.insert(this->dataPtr->entities.begin(), _entity);
}

//////////////////////////////////////////////////
void UriNestedEntity::Clear()
{
  this->dataPtr->entities.clear();
}

//////////////////////////////////////////////////
UriNestedEntity &UriNestedEntity::operator=(const UriNestedEntity &_p)
{
  this->dataPtr->entities.clear();
  for (auto i = 0u; i < _p.EntityCount(); ++i)
    this->AddEntity(_p.Entity(i));

  return *this;
}

//////////////////////////////////////////////////
UriParts::UriParts()
  : dataPtr(new UriPartsPrivate())
{
}

//////////////////////////////////////////////////
UriParts::UriParts(const std::string &_uri)
  : UriParts()
{
  this->Parse(_uri);
}


//////////////////////////////////////////////////
UriParts::UriParts(const UriParts &_parts)
  : UriParts()
{
  *this = _parts;
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
UriNestedEntity &UriParts::Entity() const
{
  return this->dataPtr->entity;
}

//////////////////////////////////////////////////
std::string UriParts::Parameter() const
{
  return this->dataPtr->parameter;
}

//////////////////////////////////////////////////
void UriParts::SetWorld(const std::string &_world)
{
  this->dataPtr->world = _world;
}

//////////////////////////////////////////////////
void UriParts::SetEntity(const UriNestedEntity &_entity)
{
  this->dataPtr->entity = _entity;
}

//////////////////////////////////////////////////
void UriParts::SetParameter(const std::string &_param)
{
  this->dataPtr->parameter = _param;
}

//////////////////////////////////////////////////
UriParts &UriParts::operator=(const UriParts &_p)
{
  this->SetWorld(_p.World());
  this->SetEntity(_p.Entity());
  this->SetParameter(_p.Parameter());

  return *this;
}

//////////////////////////////////////////////////
void UriParts::Parse(const std::string &_uri)
{
  size_t paramBegin;
  std::string uri = _uri;

  // Remove trailing '/'
  if (uri.back() == '/')
    uri.pop_back();

  this->ParseEntity(uri, paramBegin);
  this->ParseParameter(uri, paramBegin);
}

//////////////////////////////////////////////////
void UriParts::ParseEntity(const std::string &_uri, size_t &_next)
{
  size_t from = 0;

  // No entity.
  if ((from >= _uri.size()) || (_uri.at(from) == '?'))
    gzthrow("Unable to parse URI. Empty world");;

  this->dataPtr->entity.Clear();

  bool first = true;
  while (true)
  {
    UriEntity entity;
    if (!UriParts::ParseOneEntity(_uri, from, entity, _next))
      gzthrow("Unable to parse entity");

    // Set the world.
    if (first)
    {
      if (entity.Type() != "world")
        gzthrow("Unable to parse world keyword");

      this->SetWorld(entity.Name());
      first = false;
    }
    else
      this->dataPtr->entity.AddEntity(entity);

    from = _next;

    if ((_next >= _uri.size()) || (_uri.at(_next) == '?'))
      return;

    // The URI doesn't have parameter and ends with "/".
    if ((_uri.at(_next) == '/') && (_next + 1 >= _uri.size()))
    {
      _next += 1;
      return;
    }
  }
}

//////////////////////////////////////////////////
bool UriParts::ParseOneEntity(const std::string &_uri, const size_t &_from,
    UriEntity &_entity, size_t &_next)
{
  auto next = _uri.find("/", _from + 1);
  if (next == std::string::npos)
    return false;

  auto type = _uri.substr(_from + 1, next - _from - 1);
  if (type.find_first_of(" ?&=") != std::string::npos)
    return false;
 _entity.SetType(type);

  next += 1;
  auto to = _uri.find_first_of("/?", next);
  if (to == std::string::npos)
  {
    if (next == _uri.size())
    {
      // No name after the type.
      return false;
    }
    else
    {
      // Check whether invalid characters ' ', '?', '&', or '=' are found.
      auto name = _uri.substr(next, _uri.size() - next);
      if (name.find_first_of(" ?&=") != std::string::npos)
        return false;

      _entity.SetName(name);
      _next = _uri.size();
      return true;
    }
  }
  else
  {
    // Check whether invalid characters ' ', '?', '&', or '=' are found.
    auto name = _uri.substr(next, to - next);
    if (name.find_first_of(" ?&=") != std::string::npos)
      return false;

    _entity.SetName(name);
    _next = to;
    return true;
  }
}

//////////////////////////////////////////////////
void UriParts::ParseParameter(const std::string &_uri, const size_t &_from)
{
  size_t from = _from;
  this->SetParameter("");

  // No parameter.
  if (_from >= _uri.size())
    return;

  // The first character of the parameter list has to be a '?'.
  if (_uri.at(from) != '?')
    gzthrow("Unable to parse parameter ('?' not found)");

  from += 1;

  auto to = _uri.find("=", from);
  if ((to == std::string::npos) || (to == _uri.size() - 1))
    gzthrow("Unable to parse parameter ('=' not found)");

  // The parameter follows this convention: "p=value1".
  // The name of the parameter (left from the '=') and the value (right from the
  // '=') cannot contain ' ', '?', '=' or '&'.
  auto left = _uri.substr(from, to - from);
  if (left.find_first_of(" ?&=") != std::string::npos)
    gzthrow("Unable to parse parameter (invalid parameter name)");

  from = to + 1;
  std::string right = _uri.substr(from);
  if (right.find_first_of(" ?&=") != std::string::npos)
    gzthrow("Unable to parse parameter (invalid parameter value)");

  this->SetParameter(right);
}

//////////////////////////////////////////////////
Uri::Uri(const std::string &_uri)
  : dataPtr(new UriPrivate())
{
  this->dataPtr->parts.Parse(_uri);
}

//////////////////////////////////////////////////
Uri::Uri(const Uri &_uri)
  : Uri(_uri.CanonicalUri())
{
}

//////////////////////////////////////////////////
Uri::Uri(const UriParts &_parts)
  : dataPtr(new UriPrivate())
{
  this->dataPtr->parts = _parts;
}

//////////////////////////////////////////////////
Uri::~Uri()
{
}

//////////////////////////////////////////////////
UriParts &Uri::Parts() const
{
  return this->dataPtr->parts;
}

//////////////////////////////////////////////////
std::string Uri::CanonicalUri(const std::string &_param) const
{
  // Add the world part.
  std::string canonicalUri = "/world/" + this->Parts().World();

  // Add the nested entity part.
  auto nestedEntity = this->Parts().Entity();
  for (auto i = 0u; i < nestedEntity.EntityCount(); ++i)
  {
    UriEntity entity = nestedEntity.Entity(i);
    canonicalUri += "/" + entity.Type() + "/" + entity.Name();
  }

  if (!_param.empty())
  {
    if (_param.find_first_of(" ?&=") != std::string::npos)
      gzthrow("Incorrect parameter (' ', '?', '&', '=' not allowed");

    // Add the parameter part.
    canonicalUri += "?p=" + _param;
  }
  else
  {
    std::string param = this->Parts().Parameter();
    if (!param.empty())
      canonicalUri += "?p=" + param;
  }

  return canonicalUri;
}
