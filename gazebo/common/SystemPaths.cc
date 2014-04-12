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
#include <ignition/common.hh>

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

  char *homePath = getenv("HOME");
  std::string home;
  if (!homePath)
    home = "/tmp/gazebo";
  else
    home = homePath;

  sdf::addURIPath("model://", home + "/.gazebo/models");

  this->modelPaths.push_back(home + "/.gazebo/models");

  char *path = getenv("GAZEBO_LOG_PATH");
  std::string fullPath;
  if (!path)
  {
    if (home != "/tmp/gazebo")
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
const std::list<std::string> &SystemPaths::GetGazeboPaths()
{
  if (this->gazeboPathsFromEnv)
    this->UpdateGazeboPaths();
  return this->gazeboPaths;
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
void SystemPaths::UpdateModelPaths()
{
  std::string delim(":");
  std::string path;

  char *pathCStr = getenv("GAZEBO_MODEL_PATH");
  if (!pathCStr || *pathCStr == '\0')
  {
    // igndbg << "gazeboPaths is empty and GAZEBO_RESOURCE_PATH doesn't exist. "
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
std::string SystemPaths::FindFileURIHelper(const std::string &_uri)
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

  return filename;
}

//////////////////////////////////////////////////
std::string SystemPaths::FindFileHelper(const std::string &_filename)
{
  boost::filesystem::path path;
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

  if (!boost::filesystem::exists(path))
  {
    ignerr << "File or path does not exist[" << path << "]\n";
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
