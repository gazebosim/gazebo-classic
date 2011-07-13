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
/// Constructor
SystemPaths::SystemPaths()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
SystemPaths::~SystemPaths()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Loads the configuration file 
void SystemPaths::Load()
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
    TiXmlDocument xmlDoc;
    xmlDoc.LoadFile(rcFilename);

    TiXmlElement *rootnode = xmlDoc.FirstChildElement("gazeborc");
    
    // if gazebo path is set, skip reading from .gazeborc
    if(!ogre_resource_path)
    {
      TiXmlElement *node = rootnode->FirstChildElement("gazeboPath");
      while (node)
      {
        std::string path = node->GetText();
        this->gazeboPaths.push_back(path);
        this->AddPluginPaths(path+"/plugins");
        node = node->NextSiblingElement("gazeboPath");
      }
    }

    // if ogre path is set, skip reading from .gazeborc
    if(!ogre_resource_path)
    {
      TiXmlElement *node = rootnode->FirstChildElement("ogrePath");
      while (node)
      {
        this->ogrePaths.push_back( node->GetText() );
        node = node->NextSiblingElement("ogrePath");
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

////////////////////////////////////////////////////////////////////////////////
/// Get the gazebo install paths
const std::list<std::string> &SystemPaths::GetGazeboPaths() const
{
  return this->gazeboPaths;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ogre install paths  
const std::list<std::string> &SystemPaths::GetOgrePaths() const
{
  return this->ogrePaths;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the plugin paths  
const std::list<std::string> &SystemPaths::GetPluginPaths() const
{
  return this->pluginPaths;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the model path extension
std::string SystemPaths::GetModelPathExtension() const
{
  return "/models";
}

////////////////////////////////////////////////////////////////////////////////
/// Get the world path extension
std::string SystemPaths::GetWorldPathExtension() const
{
  return "/worlds";
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

////////////////////////////////////////////////////////////////////////////////
/// search for file given GAZEBO_RESOURCE_PATHS
const std::string SystemPaths::FindFileWithGazeboPaths(std::string filename) const
{
  struct stat st;
  std::string fullname =  std::string("./")+filename;
  bool found = false;

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
    for (std::list<std::string>::const_iterator iter=this->gazeboPaths.begin(); 
        iter!=this->gazeboPaths.end(); ++iter)
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
