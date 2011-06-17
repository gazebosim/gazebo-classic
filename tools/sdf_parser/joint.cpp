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

#include <boost/lexical_cast.hpp>
#include "joint.h"

using namespace sdf;

bool JointDynamics::InitXml(TiXmlElement *_config)
{
  this->Clear();

  // Get joint damping
  const char* dampingStr = _config->Attribute("damping");
  if (dampingStr == NULL)
  {
    printf("joint dynamics: no damping, defaults to 0\n");
    this->damping = 0;
  }
  else
  {
    try
    {
      this->damping = boost::lexical_cast<double>(dampingStr);
    }
    catch (boost::bad_lexical_cast &e)
    {
      printf("damping value (%s) is not a float\n",dampingStr);
      return false;
    }
  }

  // Get joint friction
  const char* frictionStr = _config->Attribute("friction");
  if (frictionStr == NULL){
    printf("joint dynamics: no friction, defaults to 0\n");
    this->friction = 0;
  }
  else
  {
    try
    {
      this->friction = boost::lexical_cast<double>(frictionStr);
    }
    catch (boost::bad_lexical_cast &e)
    {
      printf("friction value (%s) is not a float\n",frictionStr);
      return false;
    }
  }

  if (dampingStr == NULL && frictionStr == NULL)
  {
    printf("joint dynamics element specified with no damping and no friction\n");
    return false;
  }
  else{
    printf("joint dynamics: damping %f and friction %f\n", this->damping, this->friction);
    return true;
  }
}

bool JointLimits::InitXml(TiXmlElement *_config)
{
  this->Clear();

  // Get lower joint limit
  const char* lowerStr = _config->Attribute("lower");
  if (lowerStr == NULL)
  {
    printf("joint limit: no lower, defaults to 0\n");
    this->lower = 0;
  }
  else
  {
    try
    {
      this->lower = boost::lexical_cast<double>(lowerStr);
    }
    catch (boost::bad_lexical_cast &e)
    {
      printf("lower value (%s) is not a float\n",lowerStr);
      return false;
    }
  }

  // Get upper joint limit
  const char* upperStr = _config->Attribute("upper");
  if (upperStr == NULL)
  {
    printf("joint limit: no upper, , defaults to 0\n");
    this->upper = 0;
  }
  else
  {
    try
    {
      this->upper = boost::lexical_cast<double>(upperStr);
    }
    catch (boost::bad_lexical_cast &e)
    {
      printf("upper value (%s) is not a float\n",upperStr);
      return false;
    }
  }

  // Get joint effort limit
  const char* effortStr = _config->Attribute("effort");
  if (effortStr == NULL){
    printf("joint limit: no effort\n");
    return false;
  }
  else
  {
    try
    {
      this->effort = boost::lexical_cast<double>(effortStr);
    }
    catch (boost::bad_lexical_cast &e)
    {
      printf("effort value (%s) is not a float\n",effortStr);
      return false;
    }
  }

  // Get joint velocity limit
  const char* velocityStr = _config->Attribute("velocity");
  if (velocityStr == NULL)
  {
    printf("joint limit: no velocity\n");
    return false;
  }
  else
  {
    try
    {
      this->velocity = boost::lexical_cast<double>(velocityStr);
    }
    catch (boost::bad_lexical_cast &e)
    {
      printf("velocity value (%s) is not a float\n",velocityStr);
      return false;
    }
  }

  return true;
}



bool Joint::InitXml(TiXmlElement *_config)
{
  this->Clear();

  // Get Joint Name
  const char *nameStr = _config->Attribute("name");
  if (!nameStr)
  {
    printf("unnamed joint found\n");
    return false;
  }
  this->name = nameStr;

  // Get transform from Parent Link to Joint Frame
  TiXmlElement *originXml = _config->FirstChildElement("origin");
  if (!originXml)
  {
    this->origin.Clear();
  }
  else
  {
    if (!this->origin.InitXml(originXml))
    {
      printf("Malformed parent origin element for joint '%s'\n", this->name.c_str());
      this->origin.Clear();
      return false;
    }
  }

  // Get Parent Link
  TiXmlElement *parentXml = _config->FirstChildElement("parent");
  if (parentXml)
  {
    const char *pname = parentXml->Attribute("link");
    if (!pname)
      printf("no parent link name specified for Joint link '%s'. this might be the root?\n", this->name.c_str());
    else
    {
      this->parentLinkName = std::string(pname);
    }
  }

  // Get Child Link
  TiXmlElement *childXml = _config->FirstChildElement("child");
  if (childXml)
  {
    const char *pname = childXml->Attribute("link");
    if (!pname)
      printf("no child link name specified for Joint link '%s'.\n", this->name.c_str());
    else
    {
      this->childLinkName = std::string(pname);
    }
  }

  // Get Joint type
  const char* typeChar = _config->Attribute("type");
  if (!typeChar)
  {
    printf("joint '%s' has no type, check to see if it's a reference.\n", this->name.c_str());
    return false;
  }
  std::string typeStr = typeChar;
  if (typeStr == "piston")
    type = PISTON;
  else if (typeStr == "revolute2")
    type = REVOLUTE2;
  else if (typeStr == "revolute")
    type = REVOLUTE;
  else if (typeStr == "universal")
    type = UNIVERSAL;
  else if (typeStr == "prismatic")
    type = PRISMATIC;
  else if (typeStr == "ball")
    type = BALL;
  else
  {
    printf("Joint '%s' has no known type '%s'\n", this->name.c_str(), typeStr.c_str());
    return false;
  }

  // Get Joint Axis
  if (this->type != BALL)
  {
    // axis
    TiXmlElement *axisXml = _config->FirstChildElement("axis");
    if (!axisXml)
    {
      printf("no axis elemement for Joint link '%s', defaulting to (1,0,0) axis\n", this->name.c_str());
      this->axis = Vector3(1.0, 0.0, 0.0);
    }
    else{
      if (!axisXml->Attribute("xyz"))
      {
        printf("no xyz attribute for axis element for Joint link '%s'\n", this->name.c_str());
      }
      else 
      {
        if (!this->axis.Init(axisXml->Attribute("xyz")))
        {
          printf("Malformed axis element for joint '%s'\n", this->name.c_str());
          this->axis.Clear();
          return false;
        }
      }
    }
  }

  // Get limit
  TiXmlElement *limitXml = _config->FirstChildElement("limit");
  if (limitXml)
  {
    limits.reset(new JointLimits);
    if (!limits->InitXml(limitXml))
    {
      printf("Could not parse limit element for joint '%s'\n", this->name.c_str());
      limits.reset();
      return false;
    }
  }

  // Get Dynamics
  TiXmlElement *propXml = _config->FirstChildElement("dynamics");
  if (propXml)
  {
    dynamics.reset(new JointDynamics);
    if (!dynamics->InitXml(propXml))
    {
      printf("Could not parse joint_dynamics element for joint '%s'\n", this->name.c_str());
      dynamics.reset();
      return false;
    }
  }

  return true;
}
