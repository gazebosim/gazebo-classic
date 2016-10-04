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

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/URI.hh"

using namespace gazebo;
using namespace common;

static const std::string kSchemeDelim = "://";

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
  if (!this->Parse(_str))
  {
    gzwarn << "Unable to parse URIPath [" << _str << "]. Ignoring."
           << std::endl;
  }
}

/////////////////////////////////////////////////
URIPath::URIPath(const URIPath &_path)
  : URIPath()
{
  *this = _path;
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
bool URIPath::Valid() const
{
  return this->Valid(this->Str());
}

/////////////////////////////////////////////////
bool URIPath::Valid(const std::string &_str)
{
  size_t slashCount = std::count(_str.begin(), _str.end(), '/');
  if ((_str.empty()) ||
      (slashCount == _str.size()) ||
      (_str.find_first_of(" ?=&") != std::string::npos))
  {
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
bool URIPath::Parse(const std::string &_str)
{
  if (!this->Valid(_str))
    return false;

  this->Clear();

  for (auto part : common::split(_str, "/"))
    this->PushBack(part);

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
  if (!this->Parse(_str))
  {
    gzwarn << "Unable to parse URIQuery [" << _str << "]. Ignoring."
           << std::endl;
  }
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
bool URIQuery::Valid() const
{
  return this->Valid(this->Str());
}

/////////////////////////////////////////////////
bool URIQuery::Valid(const std::string &_str)
{
  if (_str.empty())
    return true;

  if ((std::count(_str.begin(), _str.end(), '?') != 1u) ||
      (_str.find("?") != 0u) ||
      (_str.find_first_of(" ") != std::string::npos))
  {
    return false;
  }

  for (auto const &query : common::split(_str.substr(1), "&"))
  {
    if (common::split(query, "=").size() != 2u)
      return false;
  }

  return true;
}

/////////////////////////////////////////////////
bool URIQuery::Parse(const std::string &_str)
{
  if (!this->Valid(_str))
    return false;

  this->Clear();

  if (!_str.empty())
  {
    for (auto query : common::split(_str.substr(1), "&"))
    {
      auto values = common::split(query, "=");
      this->Insert(values.at(0), values.at(1));
    }
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
  if (!this->Parse(_str))
    gzwarn << "Unable to parse URI [" << _str << "]. Ignoring." << std::endl;
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
const URIPath &URI::Path() const
{
  return this->dataPtr->path;
}

/////////////////////////////////////////////////
URIQuery &URI::Query()
{
  return this->dataPtr->query;
}

/////////////////////////////////////////////////
const URIQuery &URI::Query() const
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
bool URI::Valid() const
{
  return this->Valid(this->Str());
}

/////////////////////////////////////////////////
bool URI::Valid(const std::string &_str)
{
  // Validate scheme.
  auto schemeDelimPos = _str.find(kSchemeDelim);
  if ((_str.empty()) ||
      (schemeDelimPos == std::string::npos) ||
      (schemeDelimPos == 0u))
  {
    return false;
  }

  auto from = schemeDelimPos + kSchemeDelim.size();
  std::string localPath = _str.substr(from);
  std::string localQuery;

  auto to = _str.find("?", from);
  if (to != std::string::npos)
  {
    // Update path.
    localPath = _str.substr(from, to - from);

    // Update the query.
    localQuery = _str.substr(to);
  }

  // Validate the path and query.
  return URIPath::Valid(localPath) && URIQuery::Valid(localQuery);
}

/////////////////////////////////////////////////
bool URI::Parse(const std::string &_str)
{
  if (!this->Valid(_str))
    return false;

  auto schemeDelimPos = _str.find(kSchemeDelim);
  auto from = schemeDelimPos + kSchemeDelim.size();
  std::string localScheme = _str.substr(0, schemeDelimPos);
  std::string localPath = _str.substr(from);
  std::string localQuery;

  auto to = _str.find("?", from);
  if (to != std::string::npos)
  {
    // Update path.
    localPath = _str.substr(from, to - from);

    // Update the query.
    localQuery = _str.substr(to);
  }

  this->Clear();
  this->SetScheme(localScheme);

  return this->dataPtr->path.Parse(localPath) &&
         this->dataPtr->query.Parse(localQuery);
}
