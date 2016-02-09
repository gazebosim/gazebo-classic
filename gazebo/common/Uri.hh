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

#include <memory>
#include <set>
#include <string>
#include <vector>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{

    struct GZ_COMMON_VISIBLE UriEntityPart
    {
      std::string type;

      std::string name;

      std::shared_ptr<UriEntityPart> children = nullptr;
    };

    struct GZ_COMMON_VISIBLE UriParts
    {
      std::string world;

      UriEntityPart entity;

      std::vector<std::string> parameters;
    };

    /// \class Uri Uri.hh common/common.hh
    /// \brief Defines a Gazebo URI.
    class GZ_COMMON_VISIBLE Uri
    {
      /// \brief Constructor
      /// \throws common::Exception
      //public: Uri(const UriParts &_parts);

      /// \brief Destructor
      public: virtual ~Uri();

      public: static bool Parse(const std::string &_uri,
                                UriParts &_parts);

      private: static bool ParseWorld(const std::string &_uri,
                                      std::string &_world,
                                      size_t &_next);

      private: static bool ParseEntity(const std::string &_uri,
                                       const size_t &_from,
                                       UriEntityPart &_entity);

      private: static bool ParseOneEntity(const std::string &_uri,
                                          const size_t &_from,
                                          UriEntityPart &_entity,
                                          size_t &_next);

      private: static void ShowEntityPart(const UriEntityPart &_part);

      private: std::set<std::string> kAllowedEntities = {"model, light"};

      private: UriParts parts;
    };
    /// \}
  }
}
#endif
