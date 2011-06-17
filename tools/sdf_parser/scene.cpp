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

/* Author: Nate Koenig */


#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <algorithm>

#include "parser.h"
#include "scene.h"

using namespace sdf;

////////////////////////////////////////////////////////////////////////////////
bool Scene::InitXml(TiXmlElement* _config)
{
  this->Clear();

  TiXmlElement *ambient = _config->FirstChildElement("ambient");
  if (ambient)
  {
    if (ambient->Attribute("rgba"))
    {
      if (!this->ambientColor.Init(ambient->Attribute("rgba")))
      {
        printf("Error: Ambient rgba is malformed\n");
        return false;
      }
    }
    else
    {
      printf("Error: Ambient requires a rgba attribute\n");
      return false;
    }
  }

  TiXmlElement *background = _config->FirstChildElement("background");
  if (background)
  {
    if (background->Attribute("rgba"))
    {
      if (!this->backgroundColor.Init(background->Attribute("rgba")))
      {
        printf("Error: Background rgba is malformed\n");
        return false;
      }
    }
    else
    {
      printf("Error: Background requires a rgba attribute\n");
      return false;
    }


    TiXmlElement *sky = background->FirstChildElement("sky");
    if (sky)
    {
      const char *materialChar = sky->Attribute("material");
      if (!materialChar)
      {
        printf("Error: No material for the sky.\n");
        return false;
      }
      this->skyMaterial = std::string(materialChar);
    }
  }

  TiXmlElement *shadow = _config->FirstChildElement("shadows");
  if (shadow)
  {
    std::string enabled = shadow->Attribute("enabled");
    if (!enabled.empty() && !getBoolFromStr(enabled, this->shadowEnabled))
    {
      printf("Error: Shadown element requires an enabled attribute");
    }

    if (shadow->Attribute("rgba"))
    {
      if (!this->shadowColor.Init(shadow->Attribute("rgba")))
      {
        printf("Error: Shadow rgba is malformed\n");
        return false;
      }
    }
    else
    {
      printf("Error: Shadow requires a rgba attribute\n");
      return false;
    }

    this->shadowType = shadow->Attribute("type");
    if (this->shadowType.empty())
    {
      printf("Error: Shadow requires a type attribute\n");
      return false;
    }
  }

  return true;
}
