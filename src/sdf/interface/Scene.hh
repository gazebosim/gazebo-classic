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

/* Author: Nate Koenig, John Hsu */

#ifndef SDF_SCENE_HH
#define SDF_SCENE_HH

#include <string>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include "common/Color.hh"
#include "sdf/interface/Param.hh"

namespace sdf
{
  class Scene : public SDFBase
  {
    public: Scene() : 
            ambientColor("rgba", gazebo::common::Color(), true),
            backgroundColor("rgba", gazebo::common::Color(), true),
            skyMaterial("material","", false),
            shadowEnabled("enabled",true, false),
            shadowColor("rgba", gazebo::common::Color(), false),
            shadowType("type","", false),
            fogColor("rgba", gazebo::common::Color(), false),
            fogType("type","linear", false),
            fogStart("start",1.0, false),
            fogEnd("end",100.0, false),
            fogDensity("density",1.0, false)
    { 
      Param::End();
      this->xmlTree = "{scene:{ambient:rgba},{background:rgba,{sky:material}},{shadows:enabled, rgba,type},{fog:rgba,type,start,end,density}}";
    }
  
    public: ParamT<gazebo::common::Color> ambientColor;
    public: ParamT<gazebo::common::Color> backgroundColor;
    public: ParamT<std::string> skyMaterial;

    public: ParamT<bool> shadowEnabled;
    public: ParamT<gazebo::common::Color> shadowColor;
    public: ParamT<std::string> shadowType;

    public: ParamT<gazebo::common::Color> fogColor;
    public: ParamT<std::string> fogType;
    public: ParamT<double> fogStart;
    public: ParamT<double> fogEnd;
    public: ParamT<double> fogDensity;
  
    public: void Print( const std::string _prefix)
    {
      std::cout << _prefix << "Scene:\n";
      std::cout << _prefix << "  SkyMaterial[" << this->skyMaterial << "]\n";
      std::cout << _prefix << "  ShadowType[" << this->shadowType << "]\n";

      std::cout << _prefix << "  Ambient Color[" << this->ambientColor << "]\n";

      std::cout << _prefix << "  Background Color[" << this->backgroundColor << "]\n";

      std::cout << _prefix << "  Shadow Color[" << this->shadowColor << "]\n";
      std::cout << _prefix << "  Shadow Enabled[" << this->shadowEnabled << "]\n";
    }
  };
}

#endif
