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

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Uri.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
//Uri::Uri(const UriParts &/*_parts*/)
//{
//if (_parts.world.empty())
//  gzthrow("Empty world name");//
//if (this->kAllowedEntities.find(_parts.entity) ==
//    this->kAllowedEntities.end())
//{
//  gzthrow("Illegal entity name");
//}

//}

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

  if (!ParseWorld(_uri, world, next))
    return false;

  if (!ParseEntity(_uri, next, entity))
    return false;

  // ToDo: Parse parameters.

  _parts.world = world;
  _parts.entity = entity;
  return true;
}

//////////////////////////////////////////////////
bool Uri::ParseWorld(const std::string &_uri, std::string &_world,
    size_t &_next)
{
  const std::string kDelimWorld = "/world/";
  auto start = _uri.find(kDelimWorld);
  if (start != 0)
    return false;

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
      return true;

    from = next;
    entity->children.reset(new UriEntityPart());
    entity = entity->children.get();
  }

  return true;
}

//////////////////////////////////////////////////
bool Uri::ParseOneEntity(const std::string &_uri, const size_t &_from,
    UriEntityPart &_entity, size_t &_next)
{
  auto next = _uri.find("/", _from);
  if (next == std::string::npos)
    return false;

  _entity.type = _uri.substr(_from, next - _from);

  next += 1;
  auto to = _uri.find("/", next);
  if (to == std::string::npos)
  {
    if (next == _uri.size() - 1)
    {
      // No entity name after the "/".
      return false;
    }
    else
    {
      _entity.name = _uri.substr(next, _uri.size() - next + 1);
      _next = _uri.size();
      return true;
    }
  }
  else
  {
    _entity.name = _uri.substr(next, to - next);
    _next = to + 1;
    return true;
  }
}
