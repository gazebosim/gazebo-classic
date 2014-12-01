/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <vector>
#include <boost/uuid/sha1.hpp>
#include <iomanip>
#include <sstream>

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common
    /// \{

    /// \brief Load the common library.
    GZ_COMMON_VISIBLE
    void load();

    /// \brief add path sufix to common::SystemPaths
    /// \param[in] _suffix The suffix to add.
    GZ_COMMON_VISIBLE
    void add_search_path_suffix(const std::string &_suffix);

    /// \brief search for file in common::SystemPaths
    /// \param[in] _file Name of the file to find.
    /// \return The path containing the file.
    GZ_COMMON_VISIBLE
    std::string find_file(const std::string &_file);

    /// \brief search for file in common::SystemPaths
    /// \param[in] _file Name of the file to find.
    /// \param[in] _searchLocalPath True to search in the current working
    /// directory.
    /// \return The path containing the file.
    GZ_COMMON_VISIBLE
    std::string find_file(const std::string &_file,
                          bool _searchLocalPath);

    /// \brief search for a file in common::SystemPaths
    /// \param[in] _file the file name to look for.
    /// \return The path containing the file.
    GZ_COMMON_VISIBLE
    std::string find_file_path(const std::string &_file);

    /// \brief Compute the SHA1 hash of an array of bytes.
    /// \param[in] _buffer Input sequence. The permitted data types for this
    /// function are std::string and any STL container.
    /// \return The string representation (40 character) of the SHA1 hash.
    template<typename T>
    std::string get_sha1(const T &_buffer);

    /// \}
  }

  ///////////////////////////////////////////////
  // Implementation of get_sha1
  template<typename T>
  std::string common::get_sha1(const T &_buffer)
  {
    boost::uuids::detail::sha1 sha1;
    unsigned int hash[5];
    std::stringstream stream;

    if (_buffer.size() == 0)
    {
      sha1.process_bytes(NULL, 0);
    }
    else
    {
      sha1.process_bytes(&(_buffer[0]), _buffer.size() * sizeof(_buffer[0]));
    }

    sha1.get_digest(hash);

    for (std::size_t i = 0; i < sizeof(hash) / sizeof(hash[0]); ++i)
    {
      stream << std::setfill('0')
             << std::setw(sizeof(hash[0]) * 2)
             << std::hex
             << hash[i];
    }

    return stream.str();
  }
}
#endif
