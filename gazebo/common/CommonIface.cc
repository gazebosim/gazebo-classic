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
#ifdef _WIN32
  #include <Windows.h>
#endif

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <gazebo/gazebo_config.h>
#include <gazebo/common/ffmpeg_inc.h>

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/SystemPaths.hh"

using namespace gazebo;
#ifdef _WIN32
  const auto& gzstrtok = strtok_s;
#else
  const auto& gzstrtok = strtok_r;
#endif

/////////////////////////////////////////////////
void common::load()
{
#ifdef HAVE_FFMPEG
  static bool first = true;
  if (first)
  {
    first = false;
    avcodec_register_all();
    av_register_all();
  }
#endif
}

/////////////////////////////////////////////////
void common::add_search_path_suffix(const std::string &_suffix)
{
  common::SystemPaths::Instance()->AddSearchPathSuffix(_suffix);
}

/////////////////////////////////////////////////
std::string common::find_file(const std::string &_file)
{
  return common::SystemPaths::Instance()->FindFile(_file, true);
}

/////////////////////////////////////////////////
std::string common::find_file(const std::string &_file, bool _searchLocalPath)
{
  return common::SystemPaths::Instance()->FindFile(_file, _searchLocalPath);
}

/////////////////////////////////////////////////
std::string common::find_file_path(const std::string &_file)
{
  std::string filepath = common::find_file(_file);

  boost::filesystem::path path(filepath);
  if (boost::filesystem::is_directory(path))
  {
    return filepath;
  }
  else
  {
    int index = filepath.find_last_of("/");
    return filepath.substr(0, index);
  }
}

/////////////////////////////////////////////////
const char *common::getEnv(const char *_name)
{
#ifdef _WIN32
  const DWORD buffSize = 65535;
  static char buffer[buffSize];
  if (GetEnvironmentVariable(_name, buffer, buffSize))
    return buffer;
  else
    return nullptr;
#else
  return getenv(_name);
#endif
}

/////////////////////////////////////////////////
std::vector<std::string> common::split(const std::string &_str,
    const std::string &_delim)
{
  std::vector<std::string> tokens;
  char *saveptr;
  char *str = strdup(_str.c_str());

  auto token = gzstrtok(str, _delim.c_str(), &saveptr);

  while (token)
  {
    tokens.push_back(token);
    token = gzstrtok(nullptr, _delim.c_str(), &saveptr);
  }

  free(str);
  return tokens;
}
