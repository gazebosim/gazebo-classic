/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include <boost/algorithm/string.hpp>
#include <vector>

#include "parser.h"
#include "world.h"

using namespace sdf;

////////////////////////////////////////////////////////////////////////////////
World::World()
{
  this->Clear();
}

////////////////////////////////////////////////////////////////////////////////
void World::Clear()
{
  this->name.clear();
  this->joints.clear();
  this->models.clear();
  this->plugins.clear();
}

////////////////////////////////////////////////////////////////////////////////
bool World::InitFile(const std::string &_filename)
{
  TiXmlDocument xmlDoc;
  xmlDoc.LoadFile(_filename);

  return this->Init(&xmlDoc);
}

////////////////////////////////////////////////////////////////////////////////
bool World::InitString(const std::string &_xmlString)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());

  return this->Init(&xmlDoc);
}

////////////////////////////////////////////////////////////////////////////////
bool World::InitXml(TiXmlDocument *_xmlDoc)
{
  if (!_xmlDoc)
  {
    printf("Could not parse the xml\n");
    return false;
  }

  return this->Init(_xmlDoc);
}

////////////////////////////////////////////////////////////////////////////////
bool World::InitXml(TiXmlElement *_worldXml)
{
  printf("World::InitXml Parsing world xml\n");
  if (!_worldXml) 
    return false;

  return this->Init(_worldXml);
}

////////////////////////////////////////////////////////////////////////////////
bool World::Init(TiXmlDocument *_xmlDoc)
{
  if (!_xmlDoc)
  {
    printf("Could not parse the xml\n");
    return false;
  }

  TiXmlElement *worldXml = _xmlDoc->FirstChildElement("world");
  if (!worldXml)
  {
    printf("Could not find the 'world' element in the xml file\n");
    return false;
  }
  return this->Init(worldXml);
}

////////////////////////////////////////////////////////////////////////////////
bool World::Init(TiXmlElement *_worldXml)
{
  this->Clear();

  printf("Parsing the best world xml\n");

  if (!_worldXml) 
  {
    printf("Error: World XML is NULL\n");
    return false;
  }

  // Get world name
  const char *nameStr = _worldXml->Attribute("name");
  if (!nameStr)
  {
    printf("No name given for the world.\n");
    return false;
  }
  this->name = std::string(nameStr);

  this->scene.reset(new Scene);
  this->scene->InitXml(_worldXml->FirstChildElement("scene"));

  this->physics.reset(new Physics);
  this->physics->InitXml(_worldXml->FirstChildElement("physics"));

  // Get all model elements
  for (TiXmlElement* modelXml = _worldXml->FirstChildElement("model"); 
       modelXml; modelXml = modelXml->NextSiblingElement("model"))
  {
    boost::shared_ptr<Model> model;
    model.reset(new Model);

    if (model->InitXml(modelXml))
    {
      if (this->GetModel(model->name))
      {
        printf("model '%s' is not unique.\n", model->name.c_str());
        model.reset();
        return false;
      }
      else
      {
        this->models.insert(make_pair(model->name,model));
        printf("successfully added a new model '%s'\n", model->name.c_str());
      }
    }
    else
    {
      printf("model xml is not initialized correctly\n");
      model.reset();
      return false;
    }
  }

  // Get all Joint elements
  for (TiXmlElement* jointXml = _worldXml->FirstChildElement("joint"); 
       jointXml; jointXml = jointXml->NextSiblingElement("joint"))
  {
    boost::shared_ptr<Joint> joint;
    joint.reset(new Joint);

    if (joint->InitXml(jointXml))
    {
      if (this->GetJoint(joint->name))
      {
        printf("joint '%s' is not unique.\n", joint->name.c_str());
        joint.reset();
        return false;
      }
      else
      {
        this->joints.insert(make_pair(joint->name,joint));
        printf("successfully added a new joint '%s'\n", joint->name.c_str());
      }
    }
    else
    {
      printf("joint xml is not initialized correctly\n");
      joint.reset();
      return false;
    }
  }

  /// Get all the plugins
  getPlugins(_worldXml, this->plugins);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
boost::shared_ptr<const Model> World::GetModel(const std::string &_name) const
{
  boost::shared_ptr<const Model> ptr;

  if (this->models.find(_name) == this->models.end())
    ptr.reset();
  else
    ptr = this->models.find(_name)->second;

  return ptr;
}

////////////////////////////////////////////////////////////////////////////////
boost::shared_ptr<const Joint> World::GetJoint(const std::string &_name) const
{
  boost::shared_ptr<const Joint> ptr;
  if (this->joints.find(_name) == this->joints.end())
    ptr.reset();
  else
    ptr = this->joints.find(_name)->second;
  return ptr;
}
