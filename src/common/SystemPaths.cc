/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Local Gazebo configuration 
 * Author: Jordi Polo
 * Date: 3 May 2008
 */
#include <assert.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <tinyxml.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "common/SystemPaths.hh"
#include "common/Exception.hh"
#include "common/Console.hh"

using namespace gazebo;
using namespace common;


////////////////////////////////////////////////////////////////////////////////
/// Get the gazebo install paths
SystemPaths::SystemPaths()
{
  this->gazeboPaths.clear();
  this->ogrePaths.clear();
  this->pluginPaths.clear();

  char *path = getenv("GAZEBO_LOG_PATH");
  if (!path)
  {
    path = getenv("HOME");
    if (!path)
      path = strdup("/tmp/gazebo");
    else
      path = strcat(path, "/.gazebo");
  }

  DIR *dir = opendir(path); 
  if (!dir)
  {
    mkdir(path, S_IRWXU | S_IRGRP | S_IROTH);
  }
  else
    closedir(dir);

  this->logPath = path;
}

std::string SystemPaths::GetLogPath() const
{
  return this->logPath;
}


const std::list<std::string> &SystemPaths::GetGazeboPaths()
{
  if (this->gazeboPaths.size() == 0)
  {
    std::string delim(":");
    std::string path;

    char *pathCStr = getenv("GAZEBO_RESOURCE_PATH");
    if (!pathCStr || *pathCStr == '\0')
    {
      gzdbg << "gazeboPaths is empty and GAZEBO_RESOURCE_PATH doesn't exist. Set GAZEBO_RESOURCE_PATH to gazebo's installation path.  ...or are you using SystemPlugins?\n";
      return this->gazeboPaths;
    }
    path = pathCStr;

    int pos1 = 0;
    int pos2 = path.find(delim);
    while (pos2 != (int)std::string::npos)
    {
      this->gazeboPaths.push_back(path.substr(pos1,pos2-pos1));
      pos1 = pos2+1;
      pos2 = path.find(delim,pos2+1);
    }
    this->gazeboPaths.push_back(path.substr(pos1,path.size()-pos1));
  }

  return this->gazeboPaths;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ogre install paths  
const std::list<std::string> &SystemPaths::GetOgrePaths()
{
  if (this->ogrePaths.size() == 0)
  {
    std::string delim(":");
    std::string path;

    char *pathCStr = getenv("OGRE_RESOURCE_PATH");
    if (!pathCStr || *pathCStr == '\0')
    {
      gzdbg << "ogrePaths is empty and OGRE_RESOURCE_PATH doesn't exist. Set OGRE_RESOURCE_PATH to Ogre's installation path. ...or are you using SystemPlugins?\n";
      return this->ogrePaths;
    }
    path = pathCStr;

    int pos1 = 0;
    int pos2 = path.find(delim);
    while (pos2 != (int)std::string::npos)
    {
      this->ogrePaths.push_back(path.substr(pos1,pos2-pos1));
      pos1 = pos2+1;
      pos2 = path.find(delim,pos2+1);
    }
    this->ogrePaths.push_back(path.substr(pos1,path.size()-pos1));
  }

  return this->ogrePaths;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the plugin paths  
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
        << "  ...or are you loading via SystemPlugins?\n";
      return this->pluginPaths;
    }
    path = pathCStr;

    int pos1 = 0;
    int pos2 = path.find(delim);
    while (pos2 != (int)std::string::npos)
    {
      this->pluginPaths.push_back(path.substr(pos1,pos2-pos1));
      pos1 = pos2+1;
      pos2 = path.find(delim,pos2+1);
    }
    this->pluginPaths.push_back(path.substr(pos1,path.size()-pos1));
  }

  return this->pluginPaths;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the model path extension
std::string SystemPaths::GetModelPathExtension() 
{
  return "/models";
}

////////////////////////////////////////////////////////////////////////////////
/// Get the world path extension
std::string SystemPaths::GetWorldPathExtension()
{
  return "/worlds";
}

////////////////////////////////////////////////////////////////////////////////
/// search for file given GAZEBO_RESOURCE_PATHS
std::string SystemPaths::FindFileWithGazeboPaths(std::string filename)
{
  struct stat st;
  std::string fullname =  std::string("./")+filename;
  bool found = false;

  std::list<std::string> paths = GetGazeboPaths();

  if (stat(fullname.c_str(), &st) == 0)
  {
    found = true;
  }
  else if ( stat(filename.c_str(), &st) == 0)
  {
    fullname =  filename;
    found = true;
  }
  else
  {
    for (std::list<std::string>::const_iterator iter = paths.begin(); 
        iter != paths.end(); ++iter)
    {
      fullname = (*iter)+"/"+filename;
      if (stat(fullname.c_str(), &st) == 0)
      {
        found = true;
        break;
      }
      // also search under some default hardcoded subdirectories
      // is this a good idea?
      fullname = (*iter)+"/Media/models/"+filename;
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
    gzerr << "cannot load file [" << filename << "]in GAZEBO_RESOURCE_PATHS\n";
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
void SystemPaths::AddGazeboPaths(std::string gazebo_resource_path)
{
  std::string delim(":");
  if(!gazebo_resource_path.empty()) 
  {
    int pos1 = 0;
    int pos2 = gazebo_resource_path.find(delim);
    while (pos2 != (int)std::string::npos)
    {
      this->gazeboPaths.push_back(gazebo_resource_path.substr(pos1,pos2-pos1));
      pos1 = pos2+1;
      pos2 = gazebo_resource_path.find(delim,pos2+1);
    }
    this->gazeboPaths.push_back(gazebo_resource_path.substr(pos1,gazebo_resource_path.size()-pos1));
  }
}

void SystemPaths::AddOgrePaths(std::string ogre_resource_path)
{
  std::string delim(":");
  if(!ogre_resource_path.empty()) 
  {
    int pos1 = 0;
    int pos2 = ogre_resource_path.find(delim);
    while (pos2 != (int)std::string::npos)
    {
      this->ogrePaths.push_back(ogre_resource_path.substr(pos1,pos2-pos1));
      pos1 = pos2+1;
      pos2 = ogre_resource_path.find(delim,pos2+1);
    }
    this->ogrePaths.push_back(ogre_resource_path.substr(pos1,ogre_resource_path.size()-pos1));
  }
}

void SystemPaths::AddPluginPaths(std::string gazebo_plugin_path)
{
  std::string delim(":");
  if(!gazebo_plugin_path.empty()) 
  {
    int pos1 = 0;
    int pos2 = gazebo_plugin_path.find(delim);
    while (pos2 != (int)std::string::npos)
    {
      this->pluginPaths.push_back(gazebo_plugin_path.substr(pos1,pos2-pos1));
      pos1 = pos2+1;
      pos2 = gazebo_plugin_path.find(delim,pos2+1);
    }
    this->pluginPaths.push_back(gazebo_plugin_path.substr(pos1,gazebo_plugin_path.size()-pos1));
  }
}

