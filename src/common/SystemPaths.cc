/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Local Gazebo configuration
 * Author: Jordi Polo
 * Date: 3 May 2008
 */
#include <tinyxml.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <assert.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "common/SystemPaths.hh"
#include "common/Exception.hh"
#include "common/Console.hh"

using namespace gazebo;
using namespace common;


//////////////////////////////////////////////////
SystemPaths::SystemPaths()
{
  this->gazeboPaths.clear();
  this->ogrePaths.clear();
  this->pluginPaths.clear();

  char *path = getenv("GAZEBO_LOG_PATH");
  std::string fullPath;
  if (!path)
  {
    path = getenv("HOME");
    if (!path)
      fullPath = "/tmp/gazebo";
    else
      fullPath = std::string(path) + "/.gazebo";
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
}

std::string SystemPaths::GetLogPath() const
{
  return this->logPath;
}


const std::list<std::string> &SystemPaths::GetGazeboPaths()
{
  if (this->gazeboPaths.empty())
  {
    std::string delim(":");
    std::string path;

    char *pathCStr = getenv("GAZEBO_RESOURCE_PATH");
    if (!pathCStr || *pathCStr == '\0')
    {
      gzdbg << "gazeboPaths is empty and GAZEBO_RESOURCE_PATH doesn't exist. "
            << "Set GAZEBO_RESOURCE_PATH to gazebo's installation path. "
            << "...or are you using SystemPlugins?\n";
      return this->gazeboPaths;
    }
    path = pathCStr;

    size_t pos1 = 0;
    size_t pos2 = path.find(delim);
    while (pos2 != std::string::npos)
    {
      this->gazeboPaths.push_back(path.substr(pos1, pos2-pos1));
      pos1 = pos2+1;
      pos2 = path.find(delim, pos2+1);
    }
    this->gazeboPaths.push_back(path.substr(pos1, path.size()-pos1));
  }

  return this->gazeboPaths;
}

//////////////////////////////////////////////////
const std::list<std::string> &SystemPaths::GetOgrePaths()
{
  if (this->ogrePaths.size() == 0)
  {
    std::string delim(":");
    std::string path;

    char *pathCStr = getenv("OGRE_RESOURCE_PATH");
    if (!pathCStr || *pathCStr == '\0')
    {
      gzdbg << "ogrePaths is empty and OGRE_RESOURCE_PATH doesn't exist. "
            << "Set OGRE_RESOURCE_PATH to Ogre's installation path. "
            << "...or are you using SystemPlugins?\n";
      return this->ogrePaths;
    }
    path = pathCStr;

    size_t pos1 = 0;
    size_t pos2 = path.find(delim);
    while (pos2 != std::string::npos)
    {
      this->ogrePaths.push_back(path.substr(pos1, pos2-pos1));
      pos1 = pos2+1;
      pos2 = path.find(delim, pos2+1);
    }
    this->ogrePaths.push_back(path.substr(pos1, path.size()-pos1));
  }

  return this->ogrePaths;
}

//////////////////////////////////////////////////
const std::list<std::string> &SystemPaths::GetPluginPaths()
{
  if (this->pluginPaths.size() == 0)
  {
    std::string delim(":");
    std::string path;

    char *pathCStr = getenv("GAZEBO_PLUGIN_PATH");
    if (!pathCStr || *pathCStr == '\0')
    {
      gzdbg << "pluginPaths and GAZEBO_PLUGIN_PATH doesn't exist."
        << "Set GAZEBO_PLUGIN_PATH to Ogre's installation path."
        << " ...or are you loading via SystemPlugins?\n";
      return this->pluginPaths;
    }
    path = pathCStr;

    size_t pos1 = 0;
    size_t pos2 = path.find(delim);
    while (pos2 != std::string::npos)
    {
      this->pluginPaths.push_back(path.substr(pos1, pos2-pos1));
      pos1 = pos2+1;
      pos2 = path.find(delim, pos2+1);
    }
    this->pluginPaths.push_back(path.substr(pos1, path.size()-pos1));
  }

  return this->pluginPaths;
}

//////////////////////////////////////////////////
std::string SystemPaths::GetModelPathExtension()
{
  return "/models";
}

//////////////////////////////////////////////////
std::string SystemPaths::GetWorldPathExtension()
{
  return "/worlds";
}

//////////////////////////////////////////////////
std::string SystemPaths::FindFileWithGazeboPaths(const std::string &_filename)
{
  if (_filename[0] == '/')
    return _filename;

  struct stat st;
  std::string fullname = std::string("./")+_filename;
  bool found = false;

  std::list<std::string> paths = GetGazeboPaths();

  if (stat(fullname.c_str(), &st) == 0)
  {
    found = true;
  }
  else if (stat(_filename.c_str(), &st) == 0)
  {
    fullname = _filename;
    found = true;
  }
  else
  {
    for (std::list<std::string>::const_iterator iter = paths.begin();
        iter != paths.end(); ++iter)
    {
      fullname = (*iter)+"/"+_filename;
      if (stat(fullname.c_str(), &st) == 0)
      {
        found = true;
        break;
      }
      // also search under some default hardcoded subdirectories
      // is this a good idea?
      fullname = (*iter)+"/Media/models/"+_filename;
      if (stat(fullname.c_str(), &st) == 0)
      {
        found = true;
        break;
      }
    }
  }

  if (!found)
  {
    fullname.clear();
    gzerr << "cannot load file [" << _filename << "]in GAZEBO_RESOURCE_PATH\n";
  }

  return fullname;
}

void SystemPaths::ClearGazeboPaths()
{
  this->gazeboPaths.clear();
}
void SystemPaths::ClearOgrePaths()
{
  this->ogrePaths.clear();
}
void SystemPaths::ClearPluginPaths()
{
  this->pluginPaths.clear();
}
void SystemPaths::AddGazeboPaths(std::string _gazebo_resource_path)
{
  std::string delim(":");
  if (!_gazebo_resource_path.empty())
  {
    size_t pos1 = 0;
    size_t pos2 = _gazebo_resource_path.find(delim);
    while (pos2 != std::string::npos)
    {
      this->gazeboPaths.push_back(_gazebo_resource_path.substr(pos1,
                                                               pos2-pos1));
      pos1 = pos2+1;
      pos2 = _gazebo_resource_path.find(delim, pos2+1);
    }
    this->gazeboPaths.push_back(_gazebo_resource_path.substr(pos1,
                                _gazebo_resource_path.size()-pos1));
  }
}

void SystemPaths::AddOgrePaths(std::string _ogre_resource_path)
{
  std::string delim(":");
  if (!_ogre_resource_path.empty())
  {
    size_t pos1 = 0;
    size_t pos2 = _ogre_resource_path.find(delim);
    while (pos2 != std::string::npos)
    {
      this->ogrePaths.push_back(_ogre_resource_path.substr(pos1, pos2-pos1));
      pos1 = pos2+1;
      pos2 = _ogre_resource_path.find(delim, pos2+1);
    }
    this->ogrePaths.push_back(_ogre_resource_path.substr(pos1,
          _ogre_resource_path.size()-pos1));
  }
}

void SystemPaths::AddPluginPaths(std::string _gazebo_plugin_path)
{
  std::string delim(":");
  if (!_gazebo_plugin_path.empty())
  {
    size_t pos1 = 0;
    size_t pos2 = _gazebo_plugin_path.find(delim);
    while (pos2 != std::string::npos)
    {
      this->pluginPaths.push_back(_gazebo_plugin_path.substr(pos1, pos2-pos1));
      pos1 = pos2+1;
      pos2 = _gazebo_plugin_path.find(delim, pos2+1);
    }
    this->pluginPaths.push_back(_gazebo_plugin_path.substr(pos1,
          _gazebo_plugin_path.size()-pos1));
  }
}
