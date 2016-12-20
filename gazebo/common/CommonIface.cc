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
#include <fstream>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>

#ifdef __linux__
#include <sys/sendfile.h>
#endif

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <gazebo/gazebo_config.h>
#include <gazebo/common/ffmpeg_inc.h>

#include "gazebo/common/Console.hh"
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
// avcodec log callback. We use this to redirect message to gazebo's console
// messages.
#ifdef HAVE_FFMPEG
void logCallback(void *_ptr, int _level, const char *_fmt, va_list _args)
{
  static char message[8192];

  std::string msg = "ffmpeg ";

  // Get the ffmpeg module.
  if (_ptr)
  {
    AVClass *avc = *reinterpret_cast<AVClass**>(_ptr);
    const char *module = avc->item_name(_ptr);
    if (module)
      msg += std::string("[") + module + "] ";
  }

  // Create the actual message
  vsnprintf(message, sizeof(message), _fmt, _args);
  msg += message;

  // Output to the appropriate stream.
  switch (_level)
  {
    case AV_LOG_DEBUG:
      // There are a lot of debug messages. So we'll skip those.
      break;
    case AV_LOG_PANIC:
    case AV_LOG_FATAL:
    case AV_LOG_ERROR:
      gzerr << msg << std::endl;
      break;
    case AV_LOG_WARNING:
      gzwarn << msg << std::endl;
      break;
    default:
      gzmsg << msg << std::endl;
      break;
  }
}
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

#if defined(__linux__) && defined(HAVE_AVDEVICE)
    avdevice_register_all();
#endif

    // Set the log callback function.
    av_log_set_callback(logCallback);
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
  return common::isFile(_path) || common::isDirectory(_path);
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
void common::replaceAll(std::string &_result,
                        const std::string &_orig,
                        const std::string &_key,
                        const std::string &_replacement)
{
  _result = _orig;
  size_t pos = 0;
  while ((pos = _result.find(_key, pos)) != std::string::npos)
  {
    _result = _result.replace(pos, _key.length(), _replacement);
    pos += _key.length() > _replacement.length() ? 0 : _replacement.length();
  }
}

/////////////////////////////////////////////////
std::string common::replaceAll(const std::string &_orig,
                               const std::string &_key,
                               const std::string &_replacement)
{
  std::string result;
  replaceAll(result, _orig, _key, _replacement);
  return result;
}

/////////////////////////////////////////////////
std::string common::absPath(const std::string &_path)
{
  std::string result;

  char path[GZ_PATH_MAX] = "";
#ifdef _WIN32
  if (GetFullPathName(_path.c_str(), GZ_PATH_MAX, &path[0], nullptr) != 0)
#else
  if (realpath(_path.c_str(), &path[0]) != nullptr)
#endif
    result = path;
  else if (!_path.empty())
  {
    // If _path is an absolute path, then return _path.
    // An absoluate path on Windows is a character followed by a colon and a
    // forward-slash.
    if (_path.compare(0, 1, "/") == 0 || _path.compare(1, 3, ":\\") == 0)
      result = _path;
    // Otherwise return the current working directory with _path appended.
    else
      result = common::cwd() + "/" + _path;
  }

  common::replaceAll(result, result, "//", "/");

  return result;
}

/////////////////////////////////////////////////
bool common::copyFile(const std::string &_existingFilename,
                      const std::string &_newFilename)
{
  std::string absExistingFilename = common::absPath(_existingFilename);
  std::string absNewFilename = common::absPath(_newFilename);

  if (absExistingFilename == absNewFilename)
    return false;

#ifdef _WIN32
  return CopyFile(absExistingFilename.c_str(), absNewFilename.c_str(), false);
#elif defined(__APPLE__)
  bool result = false;
  std::ifstream in(absExistingFilename.c_str(), std::ifstream::binary);

  if (in.good())
  {
    std::ofstream out(absNewFilename.c_str(),
                      std::ifstream::trunc | std::ifstream::binary);
    if (out.good())
    {
      out << in.rdbuf();
      result = common::isFile(absNewFilename);
    }
    out.close();
  }
  in.close();

  return result;
#else
  int readFd = 0;
  int writeFd = 0;
  struct stat statBuf;
  off_t offset = 0;

  // Open the input file.
  readFd = open(absExistingFilename.c_str(), O_RDONLY);
  if (readFd < 0)
    return false;

  // Stat the input file to obtain its size.
  fstat(readFd, &statBuf);

  // Open the output file for writing, with the same permissions as the
  // source file.
  writeFd = open(absNewFilename.c_str(), O_WRONLY | O_CREAT, statBuf.st_mode);

  while (offset >= 0 && offset < statBuf.st_size)
  {
    // Send the bytes from one file to the other.
    ssize_t written = sendfile(writeFd, readFd, &offset, statBuf.st_size);
    if (written < 0)
      break;
  }

  close(readFd);
  close(writeFd);

  return offset == statBuf.st_size;
#endif
}
