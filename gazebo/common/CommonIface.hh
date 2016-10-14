/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_COMMON_COMMONIFACE_HH_
#define GAZEBO_COMMON_COMMONIFACE_HH_

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

    /// \brief Cross platform retrieval of an environment variable.
    /// \param[in] _name Name of the environment variable to get.
    /// \return Environment variable contents, or nullptr on error.
    GZ_COMMON_VISIBLE
    const char *getEnv(const char *_name);

    /// \brief Get the current working directory
    /// \return Name of the current directory
    GZ_COMMON_VISIBLE
    std::string cwd();

    /// \brief Returns true if _path is a file or directory
    /// \param[in] _path Path to check.
    /// \return True if _path is a file or directory
    GZ_COMMON_VISIBLE
    bool exists(const std::string &_path);

    /// \brief Check if the given path is a directory.
    /// \param[in] _path Path to a directory.
    /// \return True if _path is a directory.
    GZ_COMMON_VISIBLE
    bool isDirectory(const std::string &_path);

    /// \brief Check if the given path is a file.
    /// \param[in] _path Path to a file.
    /// \return True if _path is a file.
    GZ_COMMON_VISIBLE
    bool isFile(const std::string &_path);

    /// \brief Get the absolute path of a provided path.
    /// \param[in] _path Relative or absolute path.
    /// \return Absolute path
    GZ_COMMON_VISIBLE
    std::string absPath(const std::string &_path);

    /// \brief Copy a file.
    /// \param[in] _existingFilename Path to an existing file.
    /// \param[in] _newFilename Path of the new file.
    /// \return True on success.
    GZ_COMMON_VISIBLE
    bool copyFile(const std::string &_existingFilename,
                  const std::string &_newFilename);

    /// \brief Move a file.
    /// \param[in] _existingFilename Full path to an existing file.
    /// \param[in] _newFilename Full path of the new file.
    /// \return True on success.
    GZ_COMMON_VISIBLE
    bool moveFile(const std::string &_existingFilename,
                  const std::string &_newFilename);

    /// \brief Replace all occurances of _key with _replacement.
    /// \param[out] _result The new string that has had _key replaced
    /// with _replacement.
    /// \param[in] _orig Original string.
    /// \param[in] _key String to replace.
    /// \param[in] _replacement The string that replaces _key.
    /// \sa  std::string replaceAll(const std::string &_orig,
    /// const std::string &_key, const std::string &_replacement)
    GZ_COMMON_VISIBLE
    void replaceAll(std::string &_result,
                    const std::string &_orig,
                    const std::string &_key,
                    const std::string &_replacement);

    /// \brief Replace all occurances of _key with _replacement.
    /// \param[in] _orig Original string.
    /// \param[in] _key String to replace.
    /// \param[in] _replacement The string that replaces _key.
    /// \return The new string that has had _key replaced with _replacement.
    /// \sa void common::replaceAll(std::string &_result,
    /// const std::string &_orig, const std::string &_key,
    /// const std::string &_replacement)
    GZ_COMMON_VISIBLE
    std::string replaceAll(const std::string &_orig,
                           const std::string &_key,
                           const std::string &_replacement);

    /// \brief Splits a string into tokens.
    /// \param[in] _str Input string.
    /// \param[in] _delim Token delimiter.
    /// \return Vector of tokens.
    GZ_COMMON_VISIBLE
    std::vector<std::string> split(const std::string &_str,
                                   const std::string &_delim);

    /// \brief Generates a path for a file which doesn't collide with existing
    /// files, by appending numbers to it (i.e. (0), (1), ...)
    /// \param[in] _pathAndName Full absolute path and file name up to the
    /// file extension.
    /// \param[in] _extension File extension, such as "pdf".
    /// \return Full path with name and extension, which doesn't collide with
    /// existing files
    GZ_COMMON_VISIBLE
    std::string unique_file_path(const std::string &_pathAndName,
                                 const std::string &_extension);
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
      sha1.process_bytes(nullptr, 0);
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
