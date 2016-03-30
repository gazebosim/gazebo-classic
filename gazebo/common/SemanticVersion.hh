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

#ifndef GAZEBO_COMMON_SEMANTICVERSION_HH_
#define GAZEBO_COMMON_SEMANTICVERSION_HH_

#include <string>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common
    /// \{

    /// \brief Version comparison class based on Semantic Versioning 2.0.0
    /// http://semver.org/
    /// Compares versions and converts versions from string.
    class GZ_COMMON_VISIBLE SemanticVersion
    {
      /// \brief Constructor
      /// \param[in] _v the string version. ex: "0.3.2"
      public: SemanticVersion(const std::string &_v);

      /// \brief Constructor
      /// \param[in] _major The major number
      /// \param[in] _minor The minor number
      /// \param[in] _patch The patch number
      /// \param[in] _prerelease The prerelease string
      /// \param[in] _build The build metadata string
      public: SemanticVersion(const unsigned int _major,
                              const unsigned int _minor = 0,
                              const unsigned int _patch = 0,
                              const std::string &_prerelease = "",
                              const std::string &_build = "");

      /// \brief Parse a version string and set the major, minor, patch
      /// numbers, and prerelease and build strings.
      /// \param[in] _versionStr The version string, such as "1.2.3-pr+123"
      /// \retur True on success.
      public: bool Parse(const std::string &_versionStr);

      /// \brief Returns the version as a string
      /// \return The semantic version string
      public: std::string Version() const;

      /// \brief Get the major number
      /// \return The major number
      public: unsigned int Major() const;

      /// \brief Get the minor number
      /// \return The minor number
      public: unsigned int Minor() const;

      /// \brief Get the patch number
      /// \return The patch number
      public: unsigned int Patch() const;

      /// \brief Get the prerelease string.
      /// \return Prelrease string, empty if a prerelease string was not
      /// specified.
      public: std::string Prerelease() const;

      /// \brief Get the build metadata string. Build meta data is not used
      /// when determining precedence.
      /// \return Build metadata string, empty if a build metadata string was
      /// not specified.
      public: std::string Build() const;

      /// \brief Less than comparison operator
      /// \param[in] _other The other version to compare to
      /// returns True if _other version is newer
      public: bool operator<(const SemanticVersion &_other) const;

      /// \brief Less than or equal comparison operator
      /// \param[in] _other The other version to compare to
      /// returns True if _other version is older or equal
      public: bool operator<=(const SemanticVersion &_other) const;

      /// \brief Greater than comparison operator
      /// \param[in] _other The other version to compare to
      /// returns True if _other version is older
      public: bool operator>(const SemanticVersion &_other) const;

      /// \brief Greater than or equal comparison operator
      /// \param[in] _other The other version to compare to
      /// returns True if _other version is newer or the same
      public: bool operator>=(const SemanticVersion &_other) const;

      /// \brief Equality comparison operator
      /// \param[in] _other The other version to compare to
      /// returns True if _other version is the same
      public: bool operator==(const SemanticVersion &_other) const;

      /// \brief Inequality comparison operator
      /// \param[in] _other The other version to compare to
      /// returns True if _other version is different
      public: bool operator!=(const SemanticVersion &_other) const;

      /// \brief Major revision (incompatible api changes)
      private: unsigned int maj = 0;

      /// \brief Minor revision (backwards compatible new functionality)
      private: unsigned int min = 0;

      /// \brief Patch (bug fixes)
      private: unsigned int patch = 0;

      /// \brief Optional pre-release info. A prerelease string may be
      /// denoted by appending a hyphen and a series of dot separated
      /// identifiers immediately following the patch version
      private: std::string prerelease = "";

      /// \brief Optional build meta-data. Build metadata may be denoted by
      //appending a plus sign and a series of dot separated identifiers
      //immediately following the patch or pre-release version
      private: std::string build = "";
    };
  }
}

#endif
