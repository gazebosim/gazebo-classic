/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

#include <sdf/sdf.hh>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/ModelDatabase.hh"
#include "gazebo/common/SystemPaths.hh"

using namespace gazebo;
using namespace common;


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
    home = this->GetTmpPath() + "/gazebo";
  else
    home = homePath;

  sdf::addURIPath("model://", home + "/.gazebo/models");

  this->modelPaths.push_back(home + "/.gazebo/models");

  char *path = getenv("GAZEBO_LOG_PATH");
  std::string fullPath;
  if (!path)
  {
    if (home != this->GetTmpPath() + "/gazebo")
      fullPath = home + "/.gazebo";
    else
      fullPath = home;
  }
  else
    fullPath = path;

  DIR *dir = opendir(fullPath.c_str());
  if (!dir)
  {
    mkdir(fullPath.c_str(), S_IRWXU | S_IRGRP | S_IROTH);
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
std::string SystemPaths::GetTmpPath()
{
  return this->tmpPath.string();
}

/////////////////////////////////////////////////
std::string SystemPaths::GetTmpInstancePath()
{
  return this->tmpInstancePath.string();
}

/////////////////////////////////////////////////
std::string SystemPaths::GetDefaultTestPath()
{
  return this->GetTmpInstancePath() + "/gazebo_test";
}

/////////////////////////////////////////////////
void SystemPaths::UpdateModelPaths()
{
  std::string delim(":");
  std::string path;

  char *pathCStr = getenv("GAZEBO_MODEL_PATH");
  if (!pathCStr || *pathCStr == '\0')
  {
    // gzdbg << "gazeboPaths is empty and GAZEBO_RESOURCE_PATH doesn't exist. "
    //  << "Set GAZEBO_RESOURCE_PATH to gazebo's installation path. "
    //  << "...or are you using SystemPlugins?\n";
    return;
  }
  path = pathCStr;

  /// \TODO: Use boost to split string.
  size_t pos1 = 0;
  size_t pos2 = path.find(delim);
  while (pos2 != std::string::npos)
  {
    sdf::addURIPath("model://", path.substr(pos1, pos2-pos1));
    this->InsertUnique(path.substr(pos1, pos2-pos1), this->modelPaths);
    pos1 = pos2+1;
    pos2 = path.find(delim, pos2+1);
  }
  this->InsertUnique(path.substr(pos1, path.size()-pos1), this->modelPaths);
}

/////////////////////////////////////////////////
void SystemPaths::UpdateGazeboPaths()
{
  std::string delim(":");
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
  size_t pos2 = path.find(delim);
  while (pos2 != std::string::npos)
  {
    this->InsertUnique(path.substr(pos1, pos2-pos1), this->gazeboPaths);
    pos1 = pos2+1;
    pos2 = path.find(delim, pos2+1);
  }
  this->InsertUnique(path.substr(pos1, path.size()-pos1), this->gazeboPaths);
}

//////////////////////////////////////////////////
void SystemPaths::UpdatePluginPaths()
{
  std::string delim(":");
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
  size_t pos2 = path.find(delim);
  while (pos2 != std::string::npos)
  {
    this->InsertUnique(path.substr(pos1, pos2-pos1), this->pluginPaths);
    pos1 = pos2+1;
    pos2 = path.find(delim, pos2+1);
  }
  this->InsertUnique(path.substr(pos1, path.size()-pos1), this->pluginPaths);
}

//////////////////////////////////////////////////
void SystemPaths::UpdateOgrePaths()
{
  std::string delim(":");
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
  size_t pos2 = path.find(delim);
  while (pos2 != std::string::npos)
  {
    this->InsertUnique(path.substr(pos1, pos2-pos1), this->ogrePaths);
    pos1 = pos2+1;
    pos2 = path.find(delim, pos2+1);
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

  // If trying to find a model, return the path to the users home
  // .gazebo/models
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

    // Try to download the model if it wasn't found.
    if (filename.empty())
      filename = ModelDatabase::Instance()->GetModelPath(_uri, true);
  }
  else if (prefix.empty() || prefix == "file")
  {
    // First try to find the file on the current system
    filename = this->FindFile(suffix);
  }
  else if (prefix != "http" && prefix != "https")
    gzerr << "Unknown URI prefix[" << prefix << "]\n";

  return filename;
}

//////////////////////////////////////////////////
std::string SystemPaths::FindFile(const std::string &_filename,
                                  bool _searchLocalPath)
{
  boost::filesystem::path path;

  if (_filename.empty())
    return path.string();

  if (_filename.find("://") != std::string::npos)
  {
    path = boost::filesystem::path(this->FindFileURI(_filename));
  }
  else if (_filename[0] == '/')
  {
    path = boost::filesystem::path(_filename);
  }
  else
  {
    bool found = false;

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
      found = true;
    }
    else if ((_filename[0] == '/' || _filename[0] == '.' || _searchLocalPath)
             && boost::filesystem::exists(boost::filesystem::path(_filename)))
    {
      path = boost::filesystem::path(_filename);
      found = true;
    }
    else
    {
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
    }

    if (!found)
      return std::string();
  }

  if (!boost::filesystem::exists(path))
  {
    gzerr << "File or path does not exist[" << path << "]\n";
    return std::string();
  }

  return path.string();
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
  std::string delim(":");

  size_t pos1 = 0;
  size_t pos2 = _path.find(delim);
  while (pos2 != std::string::npos)
  {
    this->InsertUnique(_path.substr(pos1, pos2-pos1), this->gazeboPaths);
    pos1 = pos2+1;
    pos2 = _path.find(delim, pos2+1);
  }
  this->InsertUnique(_path.substr(pos1, _path.size()-pos1), this->gazeboPaths);
}

/////////////////////////////////////////////////
void SystemPaths::AddOgrePaths(const std::string &_path)
{
  std::string delim(":");
  size_t pos1 = 0;
  size_t pos2 = _path.find(delim);
  while (pos2 != std::string::npos)
  {
    this->InsertUnique(_path.substr(pos1, pos2-pos1), this->ogrePaths);
    pos1 = pos2+1;
    pos2 = _path.find(delim, pos2+1);
  }
  this->InsertUnique(_path.substr(pos1, _path.size()-pos1), this->ogrePaths);
}

/////////////////////////////////////////////////
void SystemPaths::AddPluginPaths(const std::string &_path)
{
  std::string delim(":");
  size_t pos1 = 0;
  size_t pos2 = _path.find(delim);
  while (pos2 != std::string::npos)
  {
    this->InsertUnique(_path.substr(pos1, pos2-pos1), this->pluginPaths);
    pos1 = pos2+1;
    pos2 = _path.find(delim, pos2+1);
  }
  this->InsertUnique(_path.substr(pos1, _path.size()-pos1), this->pluginPaths);
}

/////////////////////////////////////////////////
void SystemPaths::AddModelPaths(const std::string &_path)
{
  std::string delim(":");
  size_t pos1 = 0;
  size_t pos2 = _path.find(delim);
  while (pos2 != std::string::npos)
  {
    this->InsertUnique(_path.substr(pos1, pos2-pos1), this->modelPaths);
    pos1 = pos2+1;
    pos2 = _path.find(delim, pos2+1);
  }
  this->InsertUnique(_path.substr(pos1, _path.size()-pos1), this->modelPaths);
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
