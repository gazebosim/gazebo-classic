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
#include <list>
#include <map>
#include <string>

#include "gazebo/common/Exception.hh"
#include "gazebo/common/URI.hh"
#include "gazebo/common/Tokenizer.hh"

using namespace gazebo;
using namespace common;

namespace gazebo
{
  namespace common
  {
    /// \internal
    /// \brief URIPath private data.
    class URIPathPrivate
    {
      /// \brief The parts of the path.
      public: std::list<std::string> path;
    };

    /// \internal
    /// \brief URIQuery private data.
    class URIQueryPrivate
    {
      /// \brief The key/value tuples that compose the query.
      public: std::map<std::string, std::string> values;
    };

    /// \internal
    /// \brief URI private data.
    class URIPrivate
    {
      /// \brief The URI scheme.
      public: std::string scheme;

      /// \brief Path component.
      public: URIPath path;

      /// \brief Query component.
      public: URIQuery query;
    };
  }
}

/////////////////////////////////////////////////
URIPath::URIPath()
  : dataPtr(new URIPathPrivate())
{
}

/////////////////////////////////////////////////
URIPath::~URIPath()
{
}

/////////////////////////////////////////////////
URIPath::URIPath(const std::string &_str)
  : URIPath()
{
  if (!this->Load(_str))
    gzthrow("Invalid URIPath");
}

/////////////////////////////////////////////////
URIPath::URIPath(const URIPath &_path)
  : URIPath()
{
  *this = _path;
}

/////////////////////////////////////////////////
bool URIPath::Load(const std::string &_str)
{
  URIPath newPath;
  if (!this->Valid(_str, newPath))
    return false;

  *this = newPath;
  return true;
}

/////////////////////////////////////////////////
void URIPath::PushFront(const std::string &_part)
{
  this->dataPtr->path.push_front(_part);
}

/////////////////////////////////////////////////
void URIPath::PushBack(const std::string &_part)
{
  this->dataPtr->path.push_back(_part);
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
  this->dataPtr->path.push_back(_part);
  return *this;
}

/////////////////////////////////////////////////
std::string URIPath::Str(const std::string &_delim) const
{
  std::string result;
  for (auto const &part : this->dataPtr->path)
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
  this->dataPtr->path = _path.dataPtr->path;
  return *this;
}

/////////////////////////////////////////////////
void URIPath::Clear()
{
  this->dataPtr->path.clear();
}

/////////////////////////////////////////////////
bool URIPath::operator==(const URIPath &_path) const
{
  return this->dataPtr->path == _path.dataPtr->path;
}

/////////////////////////////////////////////////
bool URIPath::Valid(const std::string &_str, URIPath &_path)
{
  size_t slashCount = std::count(_str.begin(), _str.end(), '/');
  if ((_str.empty()) ||
      (slashCount == _str.size()) ||
      (_str.find_first_of(" ?=&") != std::string::npos))
  {
    return false;
  }

  _path.Clear();

  Tokenizer tokenizer(_str);
  for (auto part : tokenizer.Split("/"))
    _path.PushBack(part);

  return true;
}

/////////////////////////////////////////////////
URIQuery::URIQuery()
  : dataPtr(new URIQueryPrivate())
{
}

/////////////////////////////////////////////////
URIQuery::URIQuery(const std::string &_str)
  : URIQuery()
{
  if (!this->Load(_str))
    gzthrow("Invalid URIQuery");
}

/////////////////////////////////////////////////
URIQuery::URIQuery(const URIQuery &_query)
  : URIQuery()
{
  *this = _query;
}

/////////////////////////////////////////////////
URIQuery::~URIQuery()
{
}

/////////////////////////////////////////////////
bool URIQuery::Load(const std::string &_str)
{
  URIQuery newQuery;
  if (!this->Valid(_str, newQuery))
    return false;

  *this = newQuery;
  return true;
}

/////////////////////////////////////////////////
void URIQuery::Insert(const std::string &_key, const std::string &_value)
{
  this->dataPtr->values.insert(std::make_pair(_key, _value));
}

/////////////////////////////////////////////////
URIQuery &URIQuery::operator=(const URIQuery &_query)
{
  this->dataPtr->values = _query.dataPtr->values;
  return *this;
}

/////////////////////////////////////////////////
std::string URIQuery::Str(const std::string &_delim) const
{
  if (this->dataPtr->values.empty())
      return "";

  std::string result = "?";
  for (auto const &value : this->dataPtr->values)
  {
    if (result != "?")
      result += _delim;
    result += value.first + "=" + value.second;
  }

  return result;
}

/////////////////////////////////////////////////
void URIQuery::Clear()
{
  this->dataPtr->values.clear();
}

/////////////////////////////////////////////////
bool URIQuery::operator==(const URIQuery &_query) const
{
  return this->Str() == _query.Str();
}

/////////////////////////////////////////////////
bool URIQuery::Valid(const std::string &_str, URIQuery &_query)
{
  _query.Clear();

  if (_str.empty())
    return true;

  if ((std::count(_str.begin(), _str.end(), '?') != 1u) ||
      (_str.find("?") != 0u) ||
      (_str.find_first_of(" ") != std::string::npos))
  {
    return false;
  }

  Tokenizer tokenizer(_str.substr(1));
  auto queries = tokenizer.Split("&");
  for (auto query : queries)
  {
    Tokenizer tk(query);
    auto values = tk.Split("=");
    if (values.size() != 2u)
      return false;

    _query.Insert(values.at(0), values.at(1));
  }
  return true;
}

/////////////////////////////////////////////////
URI::URI()
  : dataPtr(new URIPrivate())
{
}

/////////////////////////////////////////////////
URI::URI(const std::string &_str)
  : URI()
{
  if (!this->Load(_str))
    gzthrow("Invalid URI");
}

/////////////////////////////////////////////////
URI::URI(const URI &_uri)
  : URI()
{
  *this = _uri;
}

/////////////////////////////////////////////////
URI::~URI()
{
}

/////////////////////////////////////////////////
bool URI::Load(const std::string &_str)
{
  URI newUri;
  if (!this->Valid(_str, newUri))
    return false;

  *this = newUri;
  return true;
}

//////////////////////////////////////////////////
std::string URI::Str() const
{
  std::string result =
    this->dataPtr->scheme.empty() ? "" : this->dataPtr->scheme + "://";
  result += this->dataPtr->path.Str() + this->dataPtr->query.Str();
  return result;
}

/////////////////////////////////////////////////
std::string URI::Scheme() const
{
  return this->dataPtr->scheme;
}

/////////////////////////////////////////////////
void URI::SetScheme(const std::string &_scheme)
{
  this->dataPtr->scheme = _scheme;
}

/////////////////////////////////////////////////
URIPath &URI::Path()
{
  return this->dataPtr->path;
}

/////////////////////////////////////////////////
URIQuery &URI::Query()
{
  return this->dataPtr->query;
}

/////////////////////////////////////////////////
void URI::Clear()
{
  this->dataPtr->scheme.clear();
  this->dataPtr->path.Clear();
  this->dataPtr->query.Clear();
}

/////////////////////////////////////////////////
bool URI::operator==(const URI &_uri) const
{
  return this->dataPtr->scheme == _uri.dataPtr->scheme &&
         this->dataPtr->path == _uri.dataPtr->path &&
         this->dataPtr->query == _uri.dataPtr->query;
}

/////////////////////////////////////////////////
URI &URI::operator=(const URI &_uri)
{
  this->dataPtr->scheme = _uri.dataPtr->scheme;
  this->dataPtr->path = _uri.dataPtr->path;
  this->dataPtr->query = _uri.dataPtr->query;
  return *this;
}

/////////////////////////////////////////////////
bool URI::Valid(const std::string &_str, URI &_uri)
{
  // Validate scheme.
  const std::string kSchemeDelim = "://";
  auto schemeDelimPos = _str.find(kSchemeDelim);
  if ((_str.empty()) ||
      (schemeDelimPos == std::string::npos) ||
      (schemeDelimPos == 0u))
  {
    return false;
  }

  auto from = schemeDelimPos + kSchemeDelim.size();
  std::string scheme = _str.substr(0, schemeDelimPos);
  std::string path = _str.substr(from);
  std::string query;

  auto to = _str.find("?", from);
  if (to != std::string::npos)
  {
    // Update path.
    path = _str.substr(from, to - from);

    // Update the query.
    query = _str.substr(to);
  }

  URIPath newPath;
  URIQuery newQuery;

  // Validate the path.
  if (!URIPath::Valid(path, newPath))
    return false;

  // Validate the query.
  if (!URIQuery::Valid(query, newQuery))
    return false;

  _uri.Clear();
  _uri.SetScheme(scheme);
  _uri.Path() = newPath;
  _uri.Query() = newQuery;
  return true;
}
