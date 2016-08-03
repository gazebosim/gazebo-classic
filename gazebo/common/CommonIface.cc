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
#include <fcntl.h>
#include <fstream>
#include <sys/stat.h>
#include <sys/sendfile.h>
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

#ifdef _WIN32
# define GZ_PATH_MAX _MAX_PATH
#elif defined(PATH_MAX)
# define GZ_PATH_MAX PATH_MAX
#elif defined(_XOPEN_PATH_MAX)
# define GZ_PATH_MAX _XOPEN_PATH_MAX
#else
# define GZ_PATH_MAX _POSIX_PATH_MAX
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
std::string common::unique_file_path(const std::string &_pathAndName,
    const std::string &_extension)
{
  std::string result = _pathAndName + "." + _extension;
  int count = 1;
  struct stat buf;

  // Check if file exists and change name accordingly
  while (stat(result.c_str(), &buf) != -1)
  {
    result = _pathAndName + "(" + std::to_string(count++) + ")." + _extension;
  }

  return result;
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

/////////////////////////////////////////////////
std::string common::cwd()
{
  char buf[GZ_PATH_MAX + 1] = {'\0'};
#ifdef _WIN32
  return _getcwd(buf, sizeof(buf)) == nullptr ? "" : buf;
#else
  return getcwd(buf, sizeof(buf)) == nullptr ? "" : buf;
#endif
}

/////////////////////////////////////////////////
bool common::exists(const std::string &_path)
{
  return common::isFile(_path) ||
         common::isDirectory(_path);
}

/////////////////////////////////////////////////
bool common::isFile(const std::string &_path)
{
  std::ifstream f(_path);
  return f.good();
}

/////////////////////////////////////////////////
bool common::isDirectory(const std::string &_path)
{
  struct stat info;

  if (stat(_path.c_str(), &info) != 0)
    return false;
  else if (info.st_mode & S_IFDIR)
    return true;
  else
    return false;
}

/////////////////////////////////////////////////
bool common::moveFile(const std::string &_existingFilename,
                      const std::string &_newFilename)
{
  return copyFile(_existingFilename, _newFilename) &&
         std::remove(_existingFilename.c_str()) == 0;
}

/////////////////////////////////////////////////
bool common::copyFile(const std::string &_existingFilename,
                      const std::string &_newFilename)
{
#ifdef _WIN32
  return CopyFile(_existingFilename, _newFilename, false);
#elif defined(__APPLE__)
  std::ifstream in(_existingFilename.c_str(), fstream::binary);
  std::ofstream out(_newFilename.c_str(), fstream::trunc|fstream::binary);
  out << in.rdbuf();
#else
  int readFd;
  int writeFd;
  struct stat statBuf;
  off_t offset = 0;

  // Open the input file.
  readFd = open(_existingFilename.c_str(), O_RDONLY);

  // Stat the input file to obtain its size.
  fstat(readFd, &statBuf);

  // Open the output file for writing, with the same permissions as the
  // source file.
  writeFd = open(_newFilename.c_str(), O_WRONLY | O_CREAT, statBuf.st_mode);

  while (offset >= 0 && offset < statBuf.st_size)
  {
    // Blast the bytes from one file to the other.
    ssize_t written = sendfile(writeFd, readFd, &offset, statBuf.st_size);
    if (written < 0)
      break;

    offset += written;
  }

  // Close up.
  close (readFd);
  close (writeFd);

  return offset == statBuf.st_size;
#endif
}
