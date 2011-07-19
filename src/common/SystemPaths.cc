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
#include <sys/stat.h>

#include "common/SystemPaths.hh"
#include "common/Exception.hh"
#include "common/Console.hh"

using namespace gazebo;
using namespace common;


////////////////////////////////////////////////////////////////////////////////
/// Get the gazebo install paths
const std::list<std::string> &SystemPaths::GetGazeboPaths()
{
  static std::list<std::string> gazeboPaths;

  if (gazeboPaths.size() == 0)
  {
    std::string delim(":");
    std::string path;

    char *pathCStr = getenv("GAZEBO_RESOURCE_PATH");
    if (!pathCStr || strlen(pathCStr) == 0)
    {
      gzerr << "GAZEBO_RESOURCE_PATH doesn't exist. Set GAZEBO_RESOURCE_PATH to gazebo's installation path.\n";
      return gazeboPaths;
    }
    path = pathCStr;

    int pos1 = 0;
    int pos2 = path.find(delim);
    while (pos2 != (int)std::string::npos)
    {
      gazeboPaths.push_back(path.substr(pos1,pos2-pos1));
      pos1 = pos2+1;
      pos2 = path.find(delim,pos2+1);
    }
    gazeboPaths.push_back(path.substr(pos1,path.size()-pos1));
  }

  return gazeboPaths;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ogre install paths  
const std::list<std::string> &SystemPaths::GetOgrePaths()
{
  static std::list<std::string> ogrePaths;

  if (ogrePaths.size() == 0)
  {
    std::string delim(":");
    std::string path;

    char *pathCStr = getenv("OGRE_RESOURCE_PATH");
    if (!pathCStr || strlen(pathCStr) == 0)
    {
      gzerr << "OGRE_RESOURCE_PATH doesn't exist. Set OGRE_RESOURCE_PATH to Ogre's installation path.\n";
      return ogrePaths;
    }
    path = pathCStr;

    int pos1 = 0;
    int pos2 = path.find(delim);
    while (pos2 != (int)std::string::npos)
    {
      ogrePaths.push_back(path.substr(pos1,pos2-pos1));
      pos1 = pos2+1;
      pos2 = path.find(delim,pos2+1);
    }
    ogrePaths.push_back(path.substr(pos1,path.size()-pos1));
  }

  return ogrePaths;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the plugin paths  
const std::list<std::string> &SystemPaths::GetPluginPaths()
{
  static std::list<std::string> pluginPaths;

  if (pluginPaths.size() == 0)
  {
    std::string delim(":");
    std::string path;

    char *pathCStr = getenv("GAZEBO_PLUGIN_PATH");
    if (!pathCStr || strlen(pathCStr) == 0)
    {
      gzerr << "GAZEBO_PLUGIN_PATH doesn't exist. Set GAZEBO_PLUGIN_PATH to Ogre's installation path.\n";
      return pluginPaths;
    }
    path = pathCStr;

    int pos1 = 0;
    int pos2 = path.find(delim);
    while (pos2 != (int)std::string::npos)
    {
      pluginPaths.push_back(path.substr(pos1,pos2-pos1));
      pos1 = pos2+1;
      pos2 = path.find(delim,pos2+1);
    }
    pluginPaths.push_back(path.substr(pos1,path.size()-pos1));
  }

  return pluginPaths;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the model path extension
std::string SystemPaths::GetModelPathExtension() 
{
  return "/sdf/models";
}

////////////////////////////////////////////////////////////////////////////////
/// Get the world path extension
std::string SystemPaths::GetWorldPathExtension()
{
  return "/sdf/worlds";
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
