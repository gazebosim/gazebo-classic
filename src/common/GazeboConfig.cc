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
 * SVN: $Id$
 */
#include <assert.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "common/XMLConfig.hh"
#include "common/GazeboConfig.hh"
#include "common/Console.hh"

using namespace gazebo;
using namespace common;


////////////////////////////////////////////////////////////////////////////////
/// Constructor
GazeboConfig::GazeboConfig()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
GazeboConfig::~GazeboConfig()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Loads the configuration file 
void GazeboConfig::Load()
{
  std::ifstream cfgFile;

  std::string rcFilename = getenv("HOME");
  rcFilename += "/.gazeborc";

  cfgFile.open(rcFilename.c_str(), std::ios::in);

  std::string delim(":");

  char *ogre_resource_path = getenv("OGRE_RESOURCE_PATH");
  if(ogre_resource_path) 
    this->AddOgrePaths(std::string(ogre_resource_path));

  char *gazebo_resource_path = getenv("GAZEBO_RESOURCE_PATH");
  if(gazebo_resource_path) 
    this->AddGazeboPaths(std::string(gazebo_resource_path));

  char *gazebo_plugin_path = getenv("GAZEBO_PLUGIN_PATH");
  if(gazebo_plugin_path) 
    this->AddPluginPaths(std::string(gazebo_plugin_path));

  if (cfgFile.is_open())
  {
    XMLConfig rc;
    XMLConfigNode *node;
    rc.Load(rcFilename);

    // if gazebo path is set, skip reading from .gazeborc
    if(!ogre_resource_path)
    {
      node = rc.GetRootNode()->GetChild("gazeboPath");
      while (node)
      {
        this->gazeboPaths.push_back(node->GetValue());
        this->AddPluginPaths(node->GetValue()+"/plugins");
        node = node->GetNext("gazeboPath");
      }
    }

    // if ogre path is set, skip reading from .gazeborc
    if(!ogre_resource_path)
    {
      node = rc.GetRootNode()->GetChild("ogrePath");
      while (node)
      {
        this->ogrePaths.push_back( node->GetValue() );
        node = node->GetNext("ogrePath");
      }
    }

  }
  else
  {
    gzwarn << "Unable to find the file ~/.gazeborc. Using default paths. This may cause OGRE to fail.\n";

    if (!gazebo_resource_path )
    {
      this->gazeboPaths.push_back("/usr/local/share/gazebo");
      this->AddPluginPaths("/usr/local/share/gazebo/plugins");
    }

    if (!ogre_resource_path )
    {
      this->ogrePaths.push_back("/usr/local/lib/OGRE");
      this->ogrePaths.push_back("/usr/lib/OGRE");
    }

  }
}

std::list<std::string> &GazeboConfig::GetGazeboPaths() 
{
  return this->gazeboPaths;
}

std::list<std::string> &GazeboConfig::GetOgrePaths()
{
  return this->ogrePaths;
}

std::list<std::string> &GazeboConfig::GetPluginPaths()
{
  return this->pluginPaths;
}

void GazeboConfig::ClearGazeboPaths()
{
  this->gazeboPaths.clear();
}
void GazeboConfig::ClearOgrePaths()
{
  this->ogrePaths.clear();
}
void GazeboConfig::ClearPluginPaths()
{
  this->pluginPaths.clear();
}

void GazeboConfig::AddGazeboPaths(std::string gazebo_resource_path)
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

void GazeboConfig::AddOgrePaths(std::string ogre_resource_path)
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

void GazeboConfig::AddPluginPaths(std::string gazebo_plugin_path)
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
