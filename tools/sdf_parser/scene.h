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

#ifndef URDF_SCENE_H
#define URDF_SCENE_H

#include <string>
#include <tinyxml.h>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include "color.h"

namespace sdf
{
  class Scene
  {
    public: Scene() { this->Clear(); };
  
    public: Color ambientColor;
    public: Color backgroundColor;
    public: std::string skyMaterial;

    public: bool shadowEnabled;
    public: Color shadowColor;
    public: std::string shadowType;
  
    public: bool InitXml(TiXmlElement *_config);
  
    public: void Clear()
    {
      this->ambientColor.Clear();
      this->backgroundColor.Clear();
      this->skyMaterial.clear();
      this->shadowEnabled = true;
      this->shadowColor.Clear();
      this->shadowType.clear();
    }

    public: friend std::ostream &operator<<(std::ostream &out, const Scene &scene)
    {
      out << "Scene:\n";
      out << "  Ambient[" << scene.ambientColor << "]\n";
      out << "  Background Color[" << scene.backgroundColor << "]\n";
      out << "  SkyMaterial[" << scene.skyMaterial << "]\n";
      out << "  ShadowEnabled[" << scene.shadowEnabled << "]\n";
      out << "  ShadowColor[" << scene.shadowColor << "]\n";
      out << "  ShadowType[" << scene.shadowType << "]\n";
      return out;
    }
  };
}

#endif
