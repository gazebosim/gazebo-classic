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
#ifndef _GAZEBO_COMMON_URI_HH_
#define _GAZEBO_COMMON_URI_HH_

#include <string>
#include <list>
#include <map>

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \brief The path component of a URI
    class GZ_COMMON_VISIBLE URIPath
    {
      /// \brief Constructor
      public: URIPath() = default;

      /// \brief Destructor
      public: virtual ~URIPath() = default;

      /// \brief Remove all parts of the path
      public: void Clear();

      /// \brief Push a new part onto the front of this path.
      /// \param[in] _part Path part to push
      public: void PushFront(const std::string &_part);

      /// \brief Push a new part onto the back of this path.
      /// \param[in] _part Path part to push
      /// \sa operator/
      public: void PushBack(const std::string &_part);

      public: const URIPath &operator/=(const std::string &_part);

      /// \brief Get the current path with the _part added to the end.
      /// \return A new Path that consists of "this / _part"
      /// \sa PushBack
      public: const URIPath operator/(const std::string &_part) const;

      /// \brief Return true if the two paths match.
      public: bool operator==(const URIPath &_path) const;

      /// \brief Get the path as a string.
      /// \return The path as a string, with each path part separated by
      /// _delim.
      public: std::string Str(const std::string &_delim = "/") const;

      public: URIPath &operator=(const URIPath &_path);

      /// \brief The parts of the path
      private: std::list<std::string> path;
    };

    /// \brief The query component of a URI
    class GZ_COMMON_VISIBLE URIQuery
    {
      /// \brief Constructor
      public: URIQuery() = default;

      /// \brief Destructor
      public: virtual ~URIQuery() = default;

      /// \brief Remove all values of the query
      public: void Clear();

      /// \brief Get this query with a new _key=_value pair added.
      /// \return This query with the additional _key = _value pair
      public: const URIQuery &Insert(const std::string &_key,
                                     const std::string &_value);

      public: URIQuery &operator=(const URIQuery &_query);

      /// \brief Return true if the two queries contain the same values.
      public: bool operator==(const URIQuery &_query) const;

      /// \brief Get the query as a string.
      /// \return The query as a string, with each key,value pair separated by
      /// _delim.
      public: std::string Str(const std::string &_delim = "&") const;

      private: std::map<std::string, std::string> values;
    };

    /// \brief A complete URI
    class GZ_COMMON_VISIBLE URI
    {
      /// \brief Default constructor
      public: URI();

      /// \brief Copy constructor
      public: URI(const URI &_uri);

      /// \brief Get the URI as a string, which has the form:
      ///
      /// scheme://path?query
      ///
      /// \return The full URI as a string
      public: std::string Str() const;

      /// \brief Remove all components of the URI
      public: void Clear();

      /// \brief Get the URI's scheme
      /// \return The scheme
      public: std::string Scheme() const;

      /// \brief Set the URI's scheme
      /// \param[in] _scheme New scheme.
      public: void SetScheme(const std::string &_scheme);

      /// \brief Get a mutable version of the path component
      /// \return A reference to the path
      public: URIPath &Path();

      /// \brief Get a mutable version of the query component
      /// \return A reference to the query
      public: URIQuery &Query();

      public: URI &operator=(const URI &_uri);

      /// \brief Return true if the two URIs match.
      public: bool operator==(const URI &_uri) const;

      /// \brief The URI scheme
      private: std::string scheme;

      /// \brief Path component
      private: URIPath path;

      /// \brief Query component
      private: URIQuery query;
    };
  }
}
#endif
