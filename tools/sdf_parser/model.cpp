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
#include "model.h"

using namespace sdf;

////////////////////////////////////////////////////////////////////////////////
Model::Model()
{
  this->Clear();
}

////////////////////////////////////////////////////////////////////////////////
void Model::Clear()
{
  this->name.clear();
  this->links.clear();
  this->joints.clear();
  this->plugins.clear();
}

////////////////////////////////////////////////////////////////////////////////
bool Model::InitFile(const std::string &_filename)
{
  TiXmlDocument xmlDoc;
  xmlDoc.LoadFile(_filename);

  return this->Init(&xmlDoc);
}

////////////////////////////////////////////////////////////////////////////////
bool Model::InitString(const std::string &_xmlString)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());

  return this->Init(&xmlDoc);
}

////////////////////////////////////////////////////////////////////////////////
bool Model::InitXml(TiXmlDocument *_xmlDoc)
{
  if (!_xmlDoc)
  {
    printf("Could not parse the xml\n");
    return false;
  }

  return this->Init(_xmlDoc);
}

////////////////////////////////////////////////////////////////////////////////
bool Model::InitXml(TiXmlElement *_modelXml)
{
  printf("Parsing model xml\n");
  if (!_modelXml) 
    return false;

  return this->Init(_modelXml);
}

////////////////////////////////////////////////////////////////////////////////
bool Model::Init(TiXmlDocument *_xmlDoc)
{
  if (!_xmlDoc)
  {
    printf("Could not parse the xml\n");
    return false;
  }

  TiXmlElement *modelXml = _xmlDoc->FirstChildElement("model");
  if (!modelXml)
  {
    printf("Could not find the 'model' element in the xml file\n");
    return false;
  }
  return this->Init(modelXml);
}

////////////////////////////////////////////////////////////////////////////////
bool Model::Init(TiXmlElement *_modelXml)
{
  this->Clear();

  printf("Parsing model xml\n");
  if (!_modelXml) 
    return false;

  // Get model name
  const char *nameStr = _modelXml->Attribute("name");
  if (!nameStr)
  {
    printf("No name given for the model.\n");
    return false;
  }
  this->name = std::string(nameStr);

  // Get all Link elements
  for (TiXmlElement* linkXml = _modelXml->FirstChildElement("link"); 
       linkXml; linkXml = linkXml->NextSiblingElement("link"))
  {
    boost::shared_ptr<Link> link;
    link.reset(new Link);

    if (link->InitXml(linkXml))
    {
      if (this->GetLink(link->name))
      {
        printf("link '%s' is not unique.\n", link->name.c_str());
        link.reset();
        return false;
      }
      else
      {
        this->links.insert(make_pair(link->name,link));
        printf("successfully added a new link '%s'\n", link->name.c_str());
      }
    }
    else
    {
      printf("link xml is not initialized correctly\n");
      link.reset();
      return false;
    }
  }

  if (this->links.empty())
  {
    printf("No link elements found in xml file\n");
    return false;
  }

  // Get all Joint elements
  for (TiXmlElement* jointXml = _modelXml->FirstChildElement("joint"); 
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
  getPlugins(_modelXml, this->plugins);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
boost::shared_ptr<const Link> Model::GetLink(const std::string &_name) const
{
  boost::shared_ptr<const Link> ptr;

  if (this->links.find(name) == this->links.end())
    ptr.reset();
  else
    ptr = this->links.find(_name)->second;

  return ptr;
}

////////////////////////////////////////////////////////////////////////////////
void Model::GetLinks(std::vector<boost::shared_ptr<Link> > &_links) const
{
  for (std::map<std::string, boost::shared_ptr<Link> >::const_iterator link = this->links.begin(); link != this->links.end(); link++)
  {
    _links.push_back(link->second);
  }
}

////////////////////////////////////////////////////////////////////////////////
void Model::GetLink(const std::string& _name, 
                    boost::shared_ptr<Link> &_link) const
{
  boost::shared_ptr<Link> ptr;

  if (this->links.find(_name) == this->links.end())
    ptr.reset();
  else
    ptr = this->links.find(_name)->second;
  _link = ptr;
}

////////////////////////////////////////////////////////////////////////////////
boost::shared_ptr<const Joint> Model::GetJoint(const std::string &_name) const
{
  boost::shared_ptr<const Joint> ptr;
  if (this->joints.find(_name) == this->joints.end())
    ptr.reset();
  else
    ptr = this->joints.find(_name)->second;
  return ptr;
}

