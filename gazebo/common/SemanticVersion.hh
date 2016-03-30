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
#include <memory>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    // Forward declare private data class
    class SemanticVersionPrivate;

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
      /// \param[in] _maj The major number
      /// \param[in] _min The minor number
      /// \param[in] _patch The patch number
      public: SemanticVersion(const unsigned int _maj,
                              const unsigned int _min,
                              const unsigned int _patch);

      /// \brief Destructor
      public: ~SemanticVersion();

     /// \brief Returns the version as a string
     /// \return The semantic version string
     public: std::string Version() const;

      /// \brief Less than comparison operator
      /// \param[in] _other The other version to compare to
      /// \return True if _other version is newer
      public: bool operator<(const SemanticVersion &_other) const;

      /// \brief Less than or equal comparison operator
      /// \param[in] _other The other version to compare to
      /// \return True if _other version is older or equal
      public: bool operator<=(const SemanticVersion &_other) const;

      /// \brief Greater than comparison operator
      /// \param[in] _other The other version to compare to
      /// \return True if _other version is older
      public: bool operator>(const SemanticVersion &_other) const;

      /// \brief Greater than or equal comparison operator
      /// \param[in] _other The other version to compare to
      /// \return True if _other version is newer or the same
      public: bool operator>=(const SemanticVersion &_other) const;

      /// \brief Equality comparison operator
      /// \param[in] _other The other version to compare to
      /// \return True if _other version is the same
      public: bool operator==(const SemanticVersion &_other) const;

      /// \brief Inequality comparison operator
      /// \param[in] _other The other version to compare to
      /// \return True if _other version is different
      public: bool operator!=(const SemanticVersion &_other) const;

      /// \brief Pointer to private data
      private: std::unique_ptr<SemanticVersionPrivate> dataPtr;
    };
  }
}
#endif
