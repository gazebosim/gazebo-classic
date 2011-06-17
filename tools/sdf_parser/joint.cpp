/*********************************************************************
* Software Ligcense Agreement (BSD License)
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

/* Author: John Hsu */

#include "joint.h"
#include <boost/lexical_cast.hpp>

namespace gdf{

bool JointDynamics::initXml(TiXmlElement* config)
{
  this->clear();

  // Get joint damping
  const char* damping_str = config->Attribute("damping");
  if (damping_str == NULL){
    printf("joint dynamics: no damping, defaults to 0\n");
    this->damping = 0;
  }
  else
  {
    try
    {
      this->damping = boost::lexical_cast<double>(damping_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      printf("damping value (%s) is not a float\n",damping_str);
      return false;
    }
  }

  // Get joint friction
  const char* friction_str = config->Attribute("friction");
  if (friction_str == NULL){
    printf("joint dynamics: no friction, defaults to 0\n");
    this->friction = 0;
  }
  else
  {
    try
    {
      this->friction = boost::lexical_cast<double>(friction_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      printf("friction value (%s) is not a float\n",friction_str);
      return false;
    }
  }

  if (damping_str == NULL && friction_str == NULL)
  {
    printf("joint dynamics element specified with no damping and no friction\n");
    return false;
  }
  else{
    printf("joint dynamics: damping %f and friction %f\n", damping, friction);
    return true;
  }
}

bool JointLimits::initXml(TiXmlElement* config)
{
  this->clear();

  // Get lower joint limit
  const char* lower_str = config->Attribute("lower");
  if (lower_str == NULL){
    printf("joint limit: no lower, defaults to 0\n");
    this->lower = 0;
  }
  else
  {
    try
    {
      this->lower = boost::lexical_cast<double>(lower_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      printf("lower value (%s) is not a float\n",lower_str);
      return false;
    }
  }

  // Get upper joint limit
  const char* upper_str = config->Attribute("upper");
  if (upper_str == NULL){
    printf("joint limit: no upper, , defaults to 0\n");
    this->upper = 0;
  }
  else
  {
    try
    {
      this->upper = boost::lexical_cast<double>(upper_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      printf("upper value (%s) is not a float\n",upper_str);
      return false;
    }
  }

  // Get joint effort limit
  const char* effort_str = config->Attribute("effort");
  if (effort_str == NULL){
    printf("joint limit: no effort\n");
    return false;
  }
  else
  {
    try
    {
      this->effort = boost::lexical_cast<double>(effort_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      printf("effort value (%s) is not a float\n",effort_str);
      return false;
    }
  }

  // Get joint velocity limit
  const char* velocity_str = config->Attribute("velocity");
  if (velocity_str == NULL){
    printf("joint limit: no velocity\n");
    return false;
  }
  else
  {
    try
    {
      this->velocity = boost::lexical_cast<double>(velocity_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      printf("velocity value (%s) is not a float\n",velocity_str);
      return false;
    }
  }

  return true;
}



bool Joint::initXml(TiXmlElement* config)
{
  this->clear();

  // Get Joint Name
  const char *name = config->Attribute("name");
  if (!name)
  {
    printf("unnamed joint found\n");
    return false;
  }
  this->name = name;

  // Get transform from Parent Link to Joint Frame
  TiXmlElement *origin_xml = config->FirstChildElement("origin");
  if (!origin_xml)
  {
    this->origin.clear();
  }
  else
  {
    if (!this->origin.initXml(origin_xml))
    {
      printf("Malformed parent origin element for joint '%s'\n", this->name.c_str());
      this->origin.clear();
      return false;
    }
  }

  // Get Parent Link
  TiXmlElement *parent_xml = config->FirstChildElement("parent");
  if (parent_xml)
  {
    const char *pname = parent_xml->Attribute("link");
    if (!pname)
      printf("no parent link name specified for Joint link '%s'. this might be the root?\n", this->name.c_str());
    else
    {
      this->parent_link_name = std::string(pname);

    }
  }

  // Get Child Link
  TiXmlElement *child_xml = config->FirstChildElement("child");
  if (child_xml)
  {
    const char *pname = child_xml->Attribute("link");
    if (!pname)
      printf("no child link name specified for Joint link '%s'.\n", this->name.c_str());
    else
    {
      this->child_link_name = std::string(pname);

    }
  }

  // Get Joint type
  const char* type_char = config->Attribute("type");
  if (!type_char)
  {
    printf("joint '%s' has no type, check to see if it's a reference.\n", this->name.c_str());
    return false;
  }
  std::string type_str = type_char;
  if (type_str == "piston")
    type = PISTON;
  else if (type_str == "revolute2")
    type = REVOLUTE2;
  else if (type_str == "revolute")
    type = REVOLUTE;
  else if (type_str == "universal")
    type = UNIVERSAL;
  else if (type_str == "prismatic")
    type = PRISMATIC;
  else if (type_str == "ball")
    type = BALL;
  else
  {
    printf("Joint '%s' has no known type '%s'\n", this->name.c_str(), type_str.c_str());
    return false;
  }

  // Get Joint Axis
  if (this->type != BALL)
  {
    // axis
    TiXmlElement *axis_xml = config->FirstChildElement("axis");
    if (!axis_xml){
      printf("no axis elemement for Joint link '%s', defaulting to (1,0,0) axis\n", this->name.c_str());
      this->axis = Vector3(1.0, 0.0, 0.0);
    }
    else{
      if (!axis_xml->Attribute("xyz")){
        printf("no xyz attribute for axis element for Joint link '%s'\n", this->name.c_str());
      }
      else {
        if (!this->axis.init(axis_xml->Attribute("xyz")))
        {
          printf("Malformed axis element for joint '%s'\n", this->name.c_str());
          this->axis.clear();
          return false;
        }
      }
    }
  }

  // Get limit
  TiXmlElement *limit_xml = config->FirstChildElement("limit");
  if (limit_xml)
  {
    limits.reset(new JointLimits);
    if (!limits->initXml(limit_xml))
    {
      printf("Could not parse limit element for joint '%s'\n", this->name.c_str());
      limits.reset();
      return false;
    }
  }

  // Get Dynamics
  TiXmlElement *prop_xml = config->FirstChildElement("dynamics");
  if (prop_xml)
  {
    dynamics.reset(new JointDynamics);
    if (!dynamics->initXml(prop_xml))
    {
      printf("Could not parse joint_dynamics element for joint '%s'\n", this->name.c_str());
      dynamics.reset();
      return false;
    }
  }

  return true;
}

}
