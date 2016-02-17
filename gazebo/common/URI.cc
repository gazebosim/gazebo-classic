/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include <algorithm>
#include <sstream>
#include <iterator>
#include "gazebo/common/URI.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
URI::URI()
{
}

/////////////////////////////////////////////////
URI::URI(const URI &_uri)
{
  *this = _uri;
}

//////////////////////////////////////////////////
std::string URI::Str() const
{
  std::string result = this->scheme.empty() ? "" : this->scheme + "://";
  result += this->path.Str() + this->query.Str();
  return result;
}

/////////////////////////////////////////////////
std::string URI::Scheme() const
{
  return this->scheme;
}

/////////////////////////////////////////////////
void URI::SetScheme(const std::string &_scheme)
{
  this->scheme = _scheme;
}

/////////////////////////////////////////////////
URIPath &URI::Path()
{
  return this->path;
}

/////////////////////////////////////////////////
URIQuery &URI::Query()
{
  return this->query;
}

/////////////////////////////////////////////////
URI &URI::operator=(const URI &_uri)
{
  this->scheme = _uri.scheme;
  this->path = _uri.path;
  this->query = _uri.query;
  return *this;
}

/////////////////////////////////////////////////
void URIPath::PushFront(const std::string &_part)
{
  this->path.push_front(_part);
}

/////////////////////////////////////////////////
void URIPath::PushBack(const std::string &_part)
{
  this->path.push_back(_part);
}

/////////////////////////////////////////////////
const URIPath URIPath::operator/(const std::string &_part) const
{
  URIPath result = *this;
  result /= _part;
  return result;
}

/////////////////////////////////////////////////
const URIPath &URIPath::operator/=(const std::string &_part)
{
  this->path.push_back(_part);
  return *this;
}

/////////////////////////////////////////////////
std::string URIPath::Str(const std::string &_delim) const
{
  std::string result;
  for (auto const &part : this->path)
  {
    if (!result.empty())
      result += _delim;
    result += part;
  }

  return result;
}

/////////////////////////////////////////////////
URIPath &URIPath::operator=(const URIPath &_path)
{
  this->path = _path.path;
  return *this;
}

/////////////////////////////////////////////////
const URIQuery URIQuery::Insert(const std::string &_key,
                                    const std::string &_value) const
{
  URIQuery result = *this;
  result.values.insert(std::make_pair(_key, _value));
  return result;
}

/////////////////////////////////////////////////
URIQuery &URIQuery::operator=(const URIQuery &_query)
{
  this->values = _query.values;
  return *this;
}

/////////////////////////////////////////////////
std::string URIQuery::Str(const std::string &_delim) const
{
  if (this->values.empty())
      return "";

  std::string result = "?";
  for (auto const &value : this->values)
  {
    if (result != "?")
      result += _delim;
    result += value.first + "=" + value.second;
  }

  return result;
}

/////////////////////////////////////////////////
void URI::Clear()
{
  this->scheme.clear();
  this->path.Clear();
  this->query.Clear();
}

/////////////////////////////////////////////////
void URIPath::Clear()
{
  this->path.clear();
}

/////////////////////////////////////////////////
void URIQuery::Clear()
{
  this->values.clear();
}

/////////////////////////////////////////////////
bool URI::operator==(const URI &_uri) const
{
  return this->scheme == _uri.scheme && this->path == _uri.path &&
         this->query == _uri.query;
}

/////////////////////////////////////////////////
bool URIPath::operator==(const URIPath &_path) const
{
 return this->path == _path.path;
}

/////////////////////////////////////////////////
bool URIQuery::operator==(const URIQuery &_query) const
{
 return this->values == _query.values;
}
