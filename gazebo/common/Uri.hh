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
    class UriPartsPrivate;
    class UriPrivate;

    /// \class UriEntity Uri.hh common/common.hh
    /// \brief A URI entity abstraction.
    ///
    /// A URI entity is composed by a type and a name. The type is a keyword
    /// such as "model" or "light" and the value is any alphanumeric identifier
    /// without whitespaces or "?". A URI entity is part of a URI.
    /// E.g.: /world/default/model/model_1/model/model_2?p=pose
    ///                      ^^^^^^^^^^^^^^
    /// "model/model_1" is a valid URI entity.
    class GZ_COMMON_VISIBLE UriEntity
    {
      /// \brief Constructor.
      public: UriEntity();

      /// \brief Copy constructor.
      /// \param[in] _entity Another entity.
      public: UriEntity(const UriEntity &_entity);

      /// \brief Destructor.
      public: virtual ~UriEntity();

      /// \brief Get the URI entity type.
      /// \return Type. E.g.: "model".
      /// \sa SetType
      public: std::string Type() const;

      /// \brief Get the URI entity name.
      /// \return The name. E.g.: "model_1".
      /// \sa SetName
      public: std::string Name() const;

      /// \brief Set the type of the URI entity. Any alphanumeric value without
      /// whitespaces or "?" is allowed.
      /// \param[in] _type The type.
      /// \throws common::Exception when _name contains a whitespace or a "?".
      /// \sa Type
      public: void SetType(const std::string &_type);

      /// \brief Set the name of the URI entity.
      /// \param[in] _name The name. Any alphanumeric value without whitespaces
      /// or "?" is allowed.
      /// \throws common::Exception when _name contains a whitespace or a "?".
      /// \sa Name
      public: void SetName(const std::string &_name);

      /// \brief Equal operator.
      /// \param[in] _p another UriEntity.
      /// \return itself.
      public: UriEntity &operator=(const UriEntity &_p);

      /// \brief Validate an identifier. Any alphanumeric identifier is valid
      /// except if contains whitespaces or "?".
      /// \throws common::Exception when _name contains a whitespace or a "?".
      private: void Validate(const std::string &_identifier);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<UriEntityPrivate> dataPtr;
    };

    /// \class UriNestedEntity Uri.hh common/common.hh
    /// \brief A URI nested entity abstraction.
    ///
    /// Some URI entities can be nested and contain URI child entities.
    /// E.g.: A model is a URI entity and can contain another URI nested models.
    /// The top level URI entity is the parent. The last URI entity is the leaf.
    /// All the URI nested entities are stored in a linear way starting from the
    /// parent and finishing with the leaf. A URI nested entity is part of a URI
    /// E.g.: /world/default/model/model_1/model/model_2?p=pose
    ///                      ^^^^^^^^^^^^^^^^^^^^^^^^^^^
    /// "model/model_1/model/model_2" is a valid nested URI entity.
    class GZ_COMMON_VISIBLE UriNestedEntity
    {
      /// \brief Constructor.
      public: UriNestedEntity();

      /// \brief Copy constructor.
      /// \param[in] _entity Another nested entity.
      public: UriNestedEntity(const UriNestedEntity &_entity);

      /// \brief Destructor.
      public: virtual ~UriNestedEntity();

      /// \brief Number of URI nested entities.
      /// \return The number of URI nested entities.
      public: unsigned int EntityCount() const;

      /// \brief Get the parent URI entity.
      /// \return The parent URI entity.
      /// \throws common::Exception when the list of entities is empty.
      public: UriEntity Parent() const;

      /// \brief Get the leaf URI entity.
      /// \return The leaf URI entity.
      /// \throws common::Exception when the list of URI entities is empty.
      public: UriEntity Leaf() const;

      /// \brief Get a specific URI nested entity.
      /// \param[in] _index The position of the requested URI entity.
      /// The parent has the 0 index and the leaf has the EntityCount() - 1.
      /// \return The requested URI entity.
      /// \throws common::Exception when _index >= EntityCount().
      /// \sa EntityCount
      public: UriEntity Entity(const unsigned int &_index) const;

      /// \brief Adds a new URI entity. The new URI entity becomes the leaf.
      /// \param[in] _entity New URI entity.
      public: void AddEntity(const UriEntity &_entity);

      /// \brief Adds a new URI entity. The new URI entity becomes the parent.
      /// \param[in] _entity New URI entity.
      public: void AddParentEntity(const UriEntity &_entity);

      /// \brief Clear the list of URI entities stored in this object.
      public: void Clear();

      /// \brief Equal operator.
      /// \param[in] _p another UriNestedEntity.
      /// \return itself.
      public: UriNestedEntity &operator=(const UriNestedEntity &_p);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<UriNestedEntityPrivate> dataPtr;
    };

    /// \class UriParts Uri.hh common/common.hh
    /// \brief Stores the components of a URI.
    ///
    /// There are multiple components in a URI:
    ///
    ///  - world:      URI entity with with type "/world/". The name contains
    ///                the name of a Gazebo world. E.g.:"/world/default"
    ///
    ///  - entities:   URI nested entity.
    ///                E.g.: "model/model_1/model/model_2"
    ///
    ///  - parameters: Vector of parameters. A parameter is some property
    ///                applied to the nested entity in the specified world.
    ///                E.g.: {"pose", "lin_vel"}
    class GZ_COMMON_VISIBLE UriParts
    {
      /// \brief Constructor.
      public: UriParts();

      /// \brief Copy constructor.
      /// \param[in] _parts Another UriParts object.
      public: UriParts(const UriParts &_parts);

      /// \brief Destructor.
      public: virtual ~UriParts();

      /// \brief Get the world part.
      /// \return The world part.
      /// \sa SetWorld
      public: std::string World() const;

      /// \brief Get the nested entity part.
      /// \return The nested entity part.
      /// \sa SetEntity
      public: UriNestedEntity &Entity() const;

      /// \brief Get the parameters part.
      /// \return The parameters part.
      /// \sa SetParameters.
      public: std::vector<std::string> &Parameters() const;

      /// \brief Set the world part.
      /// \param[in] _world World part.
      /// \sa World
      public: void SetWorld(const std::string &_world);

      /// \brief Set the nested entity part.
      /// \param[in] _entity A nested entity.
      /// \sa Entity
      public: void SetEntity(const UriNestedEntity &_entity);

      /// \brief Set the parameters part.
      /// \param[in] _params The parameters.
      /// \sa Parameters
      public: void SetParameters(const std::vector<std::string> &_params);

      /// \brief Parse a URI string and split it into its URI parts.
      /// \param[in] _uri A URI string.
      ///                 E.g.: "/model/default/model/model_1?p=pose"
      /// \param[out] _parts URI parts after splitting the URI string.
      /// \return True if the URI string was valid or false otherwise.
      public: static bool Parse(const std::string &_uri,
                                UriParts &_parts);

      /// \brief Equal operator.
      /// \param[in] _p another UriParts.
      /// \return itself.
      public: UriParts &operator=(const UriParts &_p);

      /// \brief Extract the world part from an URI string.
      /// \param[in] _uri A URI string.
      /// \param[out] _world Name of the world file.
      /// \param[out] _next The next position after parsing the world part in
      ///                   the URI string.
      /// \return True when the world name was successfully parsed.
      private: static bool ParseWorld(const std::string &_uri,
                                      std::string &_world,
                                      size_t &_next);

      /// \brief Extract the URI nested entity from an URI string.
      /// \param[in] _uri A URI string.
      /// \param[in/out] _from Position of the first character to parse.
      /// \param[out] _entity URI nested entity.
      /// \return True when the URI nested entity was successfully parsed.
      private: static bool ParseEntity(const std::string &_uri,
                                       size_t &_from,
                                       UriNestedEntity &_entity);

      /// \brief Parse one single entity from an URI string.
      /// \param[in] _uri A URI string.
      /// \param[in] _from Position of the first character to parse.
      /// \param[out] _entity URI entity.
      /// \return True when the URI entity was successfully parsed.
      private: static bool ParseOneEntity(const std::string &_uri,
                                          const size_t &_from,
                                          UriEntity &_entity,
                                          size_t &_next);

      /// \brief Extract the parameters part from an URI string.
      /// \param[in] _uri A URI string.
      /// \param[in] _from Position of the first character to parse.
      /// \param[out] _params Vector of parameters parsed.
      /// \return True when the parameters were successfully parsed.
      private: static bool ParseParameters(const std::string &_uri,
                                           const size_t &_from,
                                           std::vector<std::string> &_params);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<UriPartsPrivate> dataPtr;
    };

    /// \class Uri Uri.hh common/common.hh
    /// \brief A Gazebo URI abstraction.
    ///
    /// A Uri is a string representing some property of an entity in a given
    /// world. For example, the pose (property) of the model "model_1" (entity)
    /// in the "default" world.
    class GZ_COMMON_VISIBLE Uri
    {
      /// \brief Constructor.
      /// \param[in] _uri A URI string.
      /// \throws common::Exception when _uri cannot be correctly parsed.
      public: Uri(const std::string &_uri);

      /// \brief Copy constructor.
      /// \param[in] _uri Another Uri object.
      public: Uri(const common::Uri &_uri);

      /// \brief Constructor.
      /// \param[in] _parts Individual parts of the URI.
      public: Uri(const UriParts &_parts);

      /// \brief Destructor.
      public: virtual ~Uri();

      /// \brief Get the parts of the current URI.
      /// \return The parts in which this URI is composed.
      public: UriParts Split() const;

      /// \brief Get the URI string of the current URI.
      /// Note that the URI string is "/" terminated.
      /// \return The URI string. E.g.: "/world/default/light/light_1?p=pose/"
      public: std::string CanonicalUri() const;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<UriPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
