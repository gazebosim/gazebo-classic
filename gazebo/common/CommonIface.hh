/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _COMMONIFACE_HH_
#define _COMMONIFACE_HH_

#include <string>
#include <boost/filesystem/path.hpp>

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common
    /// \{

    /// \brief Load the common library.
    void load();

    /// \brief add path prefix to common::SystemPaths
    void add_search_path_suffix(const std::string &_suffix);

    /// \brief search for file in common::SystemPaths
    /// \param[in] _file Name of the file to find.
    std::string find_file(const std::string &_file);

    /// \brief search for file in common::SystemPaths
    /// \param[in] _file Name of the file to find.
    /// \param[in] _searchLocalPath True to search in the current working
    /// directory.
    std::string find_file(const std::string &_file,
                          bool _searchLocalPath);

    /// \brief search for a file in common::SystemPaths
    /// \param[in] _file the file name to look for
    /// \return The path containing the file
    std::string find_file_path(const std::string &_file);

    /// \brief Compute the SHA1 hash of a file.
    /// \param[in] _filename Input file.
    /// \return The string representation (40 character) of the SHA1 hash.
    std::string GetSHA1(const boost::filesystem::path &_filename);

    /// \}
  }
}
#endif
