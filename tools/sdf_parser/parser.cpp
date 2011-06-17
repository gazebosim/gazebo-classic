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

namespace gdf{

Parser::Parser()
{
  this->clear();
}

void Parser::clear()
{
  name_.clear();
  this->links_.clear();
  this->joints_.clear();
}

bool Parser::init(TiXmlDocument *xml_doc)
{
  if (!xml_doc)
  {
    printf("Could not parse the xml\n");
    return false;
  }

  TiXmlElement *model_xml = xml_doc->FirstChildElement("model");
  if (!model_xml)
  {
    printf("Could not find the 'model' element in the xml file\n");
    return false;
  }
  return init(model_xml);
}


bool Parser::init(TiXmlElement *model_xml)
{
  this->clear();

  printf("Parsing model xml\n");
  if (!model_xml) 
    return false;

  // Get model name
  const char *name = model_xml->Attribute("name");
  if (!name)
  {
    printf("No name given for the model.\n");
    return false;
  }
  this->name_ = std::string(name);

  // Get all Link elements
  for (TiXmlElement* link_xml = model_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
  {
    boost::shared_ptr<Link> link;
    link.reset(new Link);

    if (link->initXml(link_xml))
    {
      if (this->getLink(link->name))
      {
        printf("link '%s' is not unique.\n", link->name.c_str());
        link.reset();
        return false;
      }
      else
      {
        this->links_.insert(make_pair(link->name,link));
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
  if (this->links_.empty()){
    printf("No link elements found in xml file\n");
    return false;
  }

  // Get all Joint elements
  for (TiXmlElement* joint_xml = model_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
  {
    boost::shared_ptr<Joint> joint;
    joint.reset(new Joint);

    if (joint->initXml(joint_xml))
    {
      if (this->getJoint(joint->name))
      {
        printf("joint '%s' is not unique.\n", joint->name.c_str());
        joint.reset();
        return false;
      }
      else
      {
        this->joints_.insert(make_pair(joint->name,joint));
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

  return true;
}

boost::shared_ptr<const Link> Parser::getLink(const std::string& name) const
{
  boost::shared_ptr<const Link> ptr;
  if (this->links_.find(name) == this->links_.end())
    ptr.reset();
  else
    ptr = this->links_.find(name)->second;
  return ptr;
}

void Parser::getLinks(std::vector<boost::shared_ptr<Link> >& links) const
{
  for (std::map<std::string,boost::shared_ptr<Link> >::const_iterator link = this->links_.begin();link != this->links_.end(); link++)
  {
    links.push_back(link->second);
  }
}

void Parser::getLink(const std::string& name,boost::shared_ptr<Link> &link) const
{
  boost::shared_ptr<Link> ptr;
  if (this->links_.find(name) == this->links_.end())
    ptr.reset();
  else
    ptr = this->links_.find(name)->second;
  link = ptr;
}

boost::shared_ptr<const Joint> Parser::getJoint(const std::string& name) const
{
  boost::shared_ptr<const Joint> ptr;
  if (this->joints_.find(name) == this->joints_.end())
    ptr.reset();
  else
    ptr = this->joints_.find(name)->second;
  return ptr;
}

}

