/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <iostream>
#include <fstream>
#include <sstream>

#include <boost/filesystem.hpp>
#include <sdf/sdf.hh>

#include <sys/stat.h>
#include <sys/types.h>

// See below for Windows dirent include. cpplint complains about system
// header order if "win_dirent.h" is in the wrong location.
#ifndef _WIN32
  #include <dirent.h>
#else
  #include "win_dirent.h"
#endif

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/ModelDatabase.hh"
#include "gazebo/common/SystemPaths.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
char pathDelimiter()
{
#ifdef _WIN32
  return ';';
#else
  return ':';
#endif
}

/// \brief Callbacks to be called in order in case a file can't be found.
/// TODO(chapulina): Move to member variable when porting forward
std::vector<std::function<std::string (const std::string &)>> g_findFileCbs;

//////////////////////////////////////////////////
SystemPaths::SystemPaths()
{
  this->gazeboPaths.clear();
  this->ogrePaths.clear();
  this->pluginPaths.clear();
  this->modelPaths.clear();

  try
  {
    // Get a path suitable for temporary files
    this->tmpPath = boost::filesystem::temp_directory_path();

    // Get a unique path suitable for temporary files. If there are multiple
    // gazebo instances on the same machine, each one will have its own
    // temporary directory
    this->tmpInstancePath = boost::filesystem::unique_path("gazebo-%%%%%%");
  }
  catch(const boost::system::error_code &_ex)
  {
    gzerr << "Failed creating temp directory. Reason: "
          << _ex.message() << "\n";
    return;
  }

  char *homePath = getenv("HOME");
  std::string home;
  if (!homePath)
    home = this->TmpPath() + "/gazebo";
  else
    home = homePath;

  sdf::addURIPath("model://", home + "/.gazebo/models");

  this->modelPaths.push_back(home + "/.gazebo/models");

  char *path = getenv("GAZEBO_LOG_PATH");
  std::string fullPath;
  if (!path)
  {
    if (home != this->TmpPath() + "/gazebo")
      fullPath = home + "/.gazebo";
    else
      fullPath = home;
  }
  else
    fullPath = path;

  DIR *dir = opendir(fullPath.c_str());
  if (!dir)
  {
#ifdef _WIN32
    _mkdir(fullPath.c_str());
#else
    mkdir(fullPath.c_str(), S_IRWXU | S_IRGRP | S_IROTH);
#endif
  }
  else
    closedir(dir);

  this->logPath = fullPath;

  this->UpdateModelPaths();
  this->UpdateGazeboPaths();
  this->UpdatePluginPaths();
  this->UpdateOgrePaths();

  // Add some search paths
  // this->suffixPaths.push_back(std::string("/sdf/") + SDF_VERSION + "/");
  this->suffixPaths.push_back("/models/");
  this->suffixPaths.push_back("/media/models/");
  this->suffixPaths.push_back("/Media/models/");

  this->pluginPathsFromEnv = true;
  this->gazeboPathsFromEnv = true;
  this->modelPathsFromEnv = true;
  this->ogrePathsFromEnv = true;
}

/////////////////////////////////////////////////
std::string SystemPaths::GetLogPath() const
{
  return this->logPath;
}

/////////////////////////////////////////////////
const std::list<std::string> &SystemPaths::GetGazeboPaths()
{
  if (this->gazeboPathsFromEnv)
    this->UpdateGazeboPaths();
  return this->gazeboPaths;
}

/////////////////////////////////////////////////
const std::list<std::string> &SystemPaths::GetPluginPaths()
{
  if (this->pluginPathsFromEnv)
    this->UpdatePluginPaths();
  return this->pluginPaths;
}

/////////////////////////////////////////////////
const std::list<std::string> &SystemPaths::GetModelPaths()
{
  if (this->modelPathsFromEnv)
    this->UpdateModelPaths();
  return this->modelPaths;
}

/////////////////////////////////////////////////
const std::list<std::string> &SystemPaths::GetOgrePaths()
{
  if (this->ogrePathsFromEnv)
    this->UpdateOgrePaths();
  return this->ogrePaths;
}

/////////////////////////////////////////////////
const std::string &SystemPaths::TmpPath() const
{
  return this->tmpPath.string();
}

/////////////////////////////////////////////////
const std::string &SystemPaths::TmpInstancePath() const
{
  return this->tmpInstancePath.string();
}

/////////////////////////////////////////////////
std::string SystemPaths::DefaultTestPath() const
{
  return this->TmpInstancePath() + "/gazebo_test";
}

/////////////////////////////////////////////////
void SystemPaths::UpdateModelPaths()
{
  std::string path;

  char *pathCStr = getenv("GAZEBO_MODEL_PATH");
  if (!pathCStr || *pathCStr == '\0')
  {
    // No env var; take the compile-time default.
    path = GAZEBO_MODEL_PATH;
  }
  else
    path = pathCStr;

  /// TODO(anyone) Use something else to split string.
  size_t pos1 = 0;
  size_t pos2 = path.find(pathDelimiter());
  while (pos2 != std::string::npos)
  {
    sdf::addURIPath("model://", path.substr(pos1, pos2-pos1));
    this->InsertUnique(path.substr(pos1, pos2-pos1), this->modelPaths);
    pos1 = pos2+1;
    if (pos1 >= path.size())
      return;
    pos2 = path.find(pathDelimiter(), pos2+1);
  }
  sdf::addURIPath("model://", path.substr(pos1, path.size()-pos1));
  this->InsertUnique(path.substr(pos1, path.size()-pos1), this->modelPaths);
}

/////////////////////////////////////////////////
void SystemPaths::UpdateGazeboPaths()
{
  std::string path;

  char *pathCStr = getenv("GAZEBO_RESOURCE_PATH");
  if (!pathCStr || *pathCStr == '\0')
  {
    // No env var; take the compile-time default.
    path = GAZEBO_RESOURCE_PATH;
  }
  else
    path = pathCStr;

  size_t pos1 = 0;
  size_t pos2 = path.find(pathDelimiter());
  while (pos2 != std::string::npos)
  {
    this->InsertUnique(path.substr(pos1, pos2-pos1), this->gazeboPaths);
    pos1 = pos2+1;
    if (pos1 >= path.size())
      return;
    pos2 = path.find(pathDelimiter(), pos2+1);
  }
  this->InsertUnique(path.substr(pos1, path.size()-pos1), this->gazeboPaths);
}

//////////////////////////////////////////////////
void SystemPaths::UpdatePluginPaths()
{
  std::string path;

  char *pathCStr = getenv("GAZEBO_PLUGIN_PATH");
  if (!pathCStr || *pathCStr == '\0')
  {
    // No env var; take the compile-time default.
    path = GAZEBO_PLUGIN_PATH;
  }
  else
    path = pathCStr;

  size_t pos1 = 0;
  size_t pos2 = path.find(pathDelimiter());
  while (pos2 != std::string::npos)
  {
    this->InsertUnique(path.substr(pos1, pos2-pos1), this->pluginPaths);
    pos1 = pos2+1;
    if (pos1 >= path.size())
      return;
    pos2 = path.find(pathDelimiter(), pos2+1);
  }
  this->InsertUnique(path.substr(pos1, path.size()-pos1), this->pluginPaths);
}

//////////////////////////////////////////////////
void SystemPaths::UpdateOgrePaths()
{
  std::string path;

  char *pathCStr = getenv("OGRE_RESOURCE_PATH");
  if (!pathCStr || *pathCStr == '\0')
  {
    // No env var; take the compile-time default.
    path = OGRE_RESOURCE_PATH;
  }
  else
    path = pathCStr;

  size_t pos1 = 0;
  size_t pos2 = path.find(pathDelimiter());
  while (pos2 != std::string::npos)
  {
    this->InsertUnique(path.substr(pos1, pos2-pos1), this->ogrePaths);
    pos1 = pos2+1;
    if (pos1 >= path.size())
      return;
    pos2 = path.find(pathDelimiter(), pos2+1);
  }
  this->InsertUnique(path.substr(pos1, path.size()-pos1), this->ogrePaths);
}


//////////////////////////////////////////////////
std::string SystemPaths::GetWorldPathExtension()
{
  return "/worlds";
}

//////////////////////////////////////////////////
std::string SystemPaths::FindFileURI(const std::string &_uri)
{
  int index = _uri.find("://");
  std::string prefix = _uri.substr(0, index);
  std::string suffix = _uri.substr(index + 3, _uri.size() - index - 3);
  std::string filename;

  // If trying to find a model, look through all currently registered model
  // paths
  if (prefix == "model")
  {
    boost::filesystem::path path;
    for (std::list<std::string>::iterator iter = this->modelPaths.begin();
         iter != this->modelPaths.end(); ++iter)
    {
      path = boost::filesystem::path(*iter) / suffix;
      if (boost::filesystem::exists(path))
      {
        filename = path.string();
        break;
      }
    }

    // Try to download the model from models.gazebosim.org if it wasn't found.
    if (filename.empty())
      filename = ModelDatabase::Instance()->GetModelPath(_uri, true);
  }
  else if (prefix.empty() || prefix == "file")
  {
    // Try to find the file on the current system
    filename = this->FindFile(suffix);
  }

  return filename;
}

static bool isAbsolute(const std::string &_filename)
{
  boost::filesystem::path path(_filename);
  return path.is_absolute();
}

//////////////////////////////////////////////////
std::string SystemPaths::FindFile(const std::string &_filename,
                                  bool _searchLocalPath)
{
  boost::filesystem::path path;

  if (_filename.empty())
    return path.string();

  // Handle as URI
  if (_filename.find("://") != std::string::npos)
  {
    path = boost::filesystem::path(this->FindFileURI(_filename));
  }
  // Handle as local absolute path
  else if (isAbsolute(_filename))
  {
    path = boost::filesystem::path(_filename);
    // absolute paths are not portable, e.g. when running world or
    // or log files on a different machine. To workaround this problem,
    // we'll further look for these files in one of gazebo model paths.
    // e.g. /tmp/path/to/my_file
    //      =>  ${GAZEBO_MODEL_PATH}/tmp/path/to/my_file
    // Gazebo log playback makes use of this feature
    if (!boost::filesystem::exists(path))
    {
      for (std::list<std::string>::iterator iter = this->modelPaths.begin();
           iter != this->modelPaths.end(); ++iter)
      {
        auto modelPath = boost::filesystem::path(*iter) / path;
        if (boost::filesystem::exists(modelPath))
        {
          path = modelPath;
          break;
        }
      }
    }
  }
  // Try appending to Gazebo paths
  else
  {
    try
    {
      path = boost::filesystem::operator/(boost::filesystem::current_path(),
          _filename);
    }
    catch(boost::filesystem::filesystem_error &_e)
    {
      gzerr << "Filesystem error[" << _e.what() << "]\n";
      return std::string();
    }

    if (_searchLocalPath && boost::filesystem::exists(path))
    {
      // Do nothing
    }
    else if ((_filename[0] == '/' || _filename[0] == '.' || _searchLocalPath)
             && boost::filesystem::exists(boost::filesystem::path(_filename)))
    {
      path = boost::filesystem::path(_filename);
    }
    else
    {
      bool found = false;
      std::list<std::string> paths = this->GetGazeboPaths();

      for (std::list<std::string>::const_iterator iter = paths.begin();
          iter != paths.end() && !found; ++iter)
      {
        path = boost::filesystem::path((*iter));
        path = boost::filesystem::operator/(path, _filename);
        if (boost::filesystem::exists(path))
        {
          found = true;
          break;
        }

        std::list<std::string>::iterator suffixIter;
        for (suffixIter = this->suffixPaths.begin();
            suffixIter != this->suffixPaths.end(); ++suffixIter)
        {
          path = boost::filesystem::path(*iter);
          path = boost::filesystem::operator/(path, *suffixIter);
          path = boost::filesystem::operator/(path, _filename);
          if (boost::filesystem::exists(path))
          {
            found = true;
            break;
          }
        }
      }

    if (!found)
      path = std::string();
    }
  }

  // If still not found, try custom callbacks
  if (path.empty())
  {
    for (auto cb : g_findFileCbs)
    {
      path = cb(_filename);
      if (!path.empty())
        break;
    }
  }

  if (!boost::filesystem::exists(path))
  {
    gzwarn << "File or path does not exist [" << path << "] ["
           << _filename << "]" << std::endl;
    return std::string();
  }

  return path.string();
}

/////////////////////////////////////////////////
void SystemPaths::AddFindFileCallback(
    std::function<std::string (const std::string &)> _cb)
{
  g_findFileCbs.push_back(_cb);
}

/////////////////////////////////////////////////
void SystemPaths::ClearGazeboPaths()
{
  this->gazeboPaths.clear();
}

/////////////////////////////////////////////////
void SystemPaths::ClearOgrePaths()
{
  this->ogrePaths.clear();
}

/////////////////////////////////////////////////
void SystemPaths::ClearPluginPaths()
{
  this->pluginPaths.clear();
}

/////////////////////////////////////////////////
void SystemPaths::ClearModelPaths()
{
  this->modelPaths.clear();
}

/////////////////////////////////////////////////
void SystemPaths::AddGazeboPaths(const std::string &_path)
{
  size_t pos1 = 0;
  size_t pos2 = _path.find(pathDelimiter());
  while (pos2 != std::string::npos)
  {
    this->InsertUnique(_path.substr(pos1, pos2-pos1), this->gazeboPaths);
    pos1 = pos2+1;
    pos2 = _path.find(pathDelimiter(), pos2+1);
  }
  this->InsertUnique(_path.substr(pos1, _path.size()-pos1), this->gazeboPaths);
}

/////////////////////////////////////////////////
void SystemPaths::AddOgrePaths(const std::string &_path)
{
  size_t pos1 = 0;
  size_t pos2 = _path.find(pathDelimiter());
  while (pos2 != std::string::npos)
  {
    this->InsertUnique(_path.substr(pos1, pos2-pos1), this->ogrePaths);
    pos1 = pos2+1;
    pos2 = _path.find(pathDelimiter(), pos2+1);
  }
  this->InsertUnique(_path.substr(pos1, _path.size()-pos1), this->ogrePaths);
}

/////////////////////////////////////////////////
void SystemPaths::AddPluginPaths(const std::string &_path)
{
  size_t pos1 = 0;
  size_t pos2 = _path.find(pathDelimiter());
  while (pos2 != std::string::npos)
  {
    this->InsertUnique(_path.substr(pos1, pos2-pos1), this->pluginPaths);
    pos1 = pos2+1;
    pos2 = _path.find(pathDelimiter(), pos2+1);
  }
  this->InsertUnique(_path.substr(pos1, _path.size()-pos1), this->pluginPaths);
}

/////////////////////////////////////////////////
void SystemPaths::AddModelPaths(const std::string &_path)
{
  size_t pos1 = 0;
  size_t pos2 = _path.find(pathDelimiter());
  while (pos2 != std::string::npos)
  {
    this->InsertUnique(_path.substr(pos1, pos2-pos1), this->modelPaths);
    pos1 = pos2+1;
    pos2 = _path.find(pathDelimiter(), pos2+1);
  }
  this->InsertUnique(_path.substr(pos1, _path.size()-pos1), this->modelPaths);
}

/////////////////////////////////////////////////
void SystemPaths::AddModelPathsUpdate(const std::string &_path)
{
  this->AddModelPaths(_path);
  updateModelRequest(_path);
}

/////////////////////////////////////////////////
void SystemPaths::InsertUnique(const std::string &_path,
                               std::list<std::string> &_list)
{
  if (std::find(_list.begin(), _list.end(), _path) == _list.end())
    _list.push_back(_path);
}

/////////////////////////////////////////////////
void SystemPaths::AddSearchPathSuffix(const std::string &_suffix)
{
  std::string s;

  if (_suffix[0] != '/')
    s = std::string("/") + _suffix;
  else
    s = _suffix;

  if (_suffix[_suffix.size()-1] != '/')
    s += "/";

  this->suffixPaths.push_back(s);
}
