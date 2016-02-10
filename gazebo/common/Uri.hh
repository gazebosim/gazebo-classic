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

    // Forward declare private data classes.
    class UriEntityPrivate;
    class UriNestedEntityPrivate;
    //class UriEntityPartPrivate;
    class UriPartsPrivate;

    /// \brief ToDo.
    class GZ_COMMON_VISIBLE UriEntity
    {
      /// \brief ToDo.
      public: UriEntity();

      /// \brief ToDo.
      public: UriEntity(const UriEntity &_entity);

      /// \brief ToDo.
      public: virtual ~UriEntity();

      /// \brief ToDo.
      public: std::string Type() const;

      /// \brief ToDo.
      public: std::string Name() const;

      /// \brief ToDo.
      public: void SetType(const std::string &_type);

      /// \brief ToDo.
      public: void SetName(const std::string &_name);

      /// \brief Equal operator.
      /// \param _p another UriEntity.
      /// \return itself.
      public: UriEntity &operator=(const UriEntity &_p);

      /// \brief ToDo.
      private: std::unique_ptr<UriEntityPrivate> dataPtr;
    };

    /// \brief ToDo.
    class GZ_COMMON_VISIBLE UriNestedEntity
    {
      /// \brief ToDo.
      public: UriNestedEntity();

      /// \brief ToDo.
      public: virtual ~UriNestedEntity();

      /// \brief ToDo.
      public: unsigned int EntityCount() const;

      /// \brief ToDo.
      public: bool Parent(UriEntity &_entity) const;

      /// \brief ToDo.
      public: bool Leaf(UriEntity &_entity) const;

      /// \brief ToDo.
      public: bool Entity(const unsigned int &_index,
                          UriEntity &_entity) const;

      /// \brief ToDo.
      public: void AddEntity(const UriEntity &_entity);

      /// \brief ToDo.
      public: void Clear();

      /// \brief Equal operator.
      /// \param _p another UriNestedEntity.
      /// \return itself.
      public: UriNestedEntity &operator=(const UriNestedEntity &_p);

      /// \brief ToDo.
      private: std::unique_ptr<UriNestedEntityPrivate> dataPtr;
    };

    /// \brief ToDo.
    class GZ_COMMON_VISIBLE UriParts
    {
      /// \brief ToDo.
      public: UriParts();

      /// \brief ToDo.
      public: virtual ~UriParts();

      /// \brief ToDo.
      public: std::string World() const;

      /// \brief ToDo.
      public: UriNestedEntity &Entity() const;

      /// \brief ToDo.
      public: std::vector<std::string> &Parameters() const;

      /// \brief ToDo.
      public: void SetWorld(const std::string &_world);

      /// \brief ToDo.
      public: void SetEntity(const UriNestedEntity &_entity);

      /// \brief ToDo.
      public: void SetParameters(const std::vector<std::string> &_params);

      /// \brief Equal operator.
      /// \param _p another UriParts.
      /// \return itself.
      public: UriParts &operator=(const UriParts &_p);

      /// \brief ToDo.
      private: std::unique_ptr<UriPartsPrivate> dataPtr;
    };

    /*
    /// \brief ToDo.
    class GZ_COMMON_VISIBLE UriEntityPart
    {
      /// \brief ToDo.
      public: UriEntityPart();

      /// \brief ToDo.
      public: virtual ~UriEntityPart();

      /// \brief ToDo.
      public: std::string Type() const;

      /// \brief ToDo.
      public: std::string Name() const;

      /// \brief ToDo.
      public: std::shared_ptr<UriEntityPart> Children() const;

      /// \brief ToDo.
      public: void SetType(const std::string &_type);

      /// \brief ToDo.
      public: void SetName(const std::string &_name);

      /// \brief ToDo.
      public: void SetChildren(const std::shared_ptr<UriEntityPart> &_children);

      /// \brief Equal operator.
      /// \param _p another UriEntityPart.
      /// \return itself.
      public: UriEntityPart &operator=(const UriEntityPart &_p);

      /// \brief ToDo.
      private: std::unique_ptr<UriEntityPartPrivate> dataPtr;
    };

    /// \brief ToDo.
    class GZ_COMMON_VISIBLE UriParts
    {
      /// \brief ToDo.
      public: UriParts();

      /// \brief ToDo.
      public: virtual ~UriParts();

      /// \brief ToDo.
      public: std::string World() const;

      /// \brief ToDo.
      public: UriEntityPart &Entity() const;

      /// \brief ToDo.
      public: std::vector<std::string> &Parameters() const;

      /// \brief ToDo.
      public: void SetWorld(const std::string &_world);

      /// \brief ToDo.
      public: void SetEntity(const UriEntityPart &_entity);

      /// \brief ToDo.
      public: void SetParameters(const std::vector<std::string> &_params);

      /// \brief Equal operator.
      /// \param _p another UriParts.
      /// \return itself.
      public: UriParts &operator=(const UriParts &_p);

      /// \brief ToDo.
      private: std::unique_ptr<UriPartsPrivate> dataPtr;
    };
    */

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
                                       UriNestedEntity &_entity);

      private: static bool ParseOneEntity(const std::string &_uri,
                                          const size_t &_from,
                                          UriEntity &_entity,
                                          size_t &_next);

      //private: static void ShowEntityPart(const UriEntityPart &_part);

      //private: std::set<std::string> kAllowedEntities = {"model, light"};

      private: UriParts parts;
    };
    /// \}
  }
}
#endif
