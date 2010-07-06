/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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

#include "XMLConfig.hh"
#include "GazeboConfig.hh"
#include "GazeboMessage.hh"

using namespace gazebo;

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
  if(this->ogrePaths.empty() && ogre_resource_path) 
    this->AddOgrePaths(std::string(ogre_resource_path));

  char *gazebo_resource_path = getenv("GAZEBO_RESOURCE_PATH");
  if(this->gazeboPaths.empty() && gazebo_resource_path) 
    this->AddGazeboPaths(std::string(gazebo_resource_path));

  char *gazebo_plugin_path = getenv("GAZEBO_PLUGIN_PATH");
  if(this->pluginPaths.empty() && gazebo_plugin_path) 
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
        gzmsg(5) << "Gazebo Path[" << node->GetValue() << "]\n";
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
        gzmsg(5) << "Ogre Path[" << node->GetValue() << "]\n";
        this->ogrePaths.push_back( node->GetValue() );
        node = node->GetNext("ogrePath");
      }
    }

  }
  else
  {
    gzmsg(0) << "Unable to find the file ~/.gazeborc. Using default paths. This may cause OGRE to fail.\n";

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
