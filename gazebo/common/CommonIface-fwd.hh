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

#ifndef _COMMONIFACEFWD_HH_
#define _COMMONIFACEFWD_HH_

#include <vector>
#include <boost/uuid/sha1.hpp>
#include <iosfwd>

namespace gazebo
{
  namespace common
  {
    void load();

    void add_search_path_suffix(const std::string &_suffix);

    std::string find_file(const std::string &_file);

    std::string find_file(const std::string &_file,
                          bool _searchLocalPath);

    std::string find_file_path(const std::string &_file);

    /// function are std::string and any STL container.
    template<typename T>
    std::string get_sha1(const T &_buffer);

    const char *getEnv(const char *_name);
    /// \}
  }
}
#endif
