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

#ifndef SDF_SENSOR_HH
#define SDF_SENSOR_HH

#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include "sdf/interface/SDFBase.hh"
#include "sdf/interface/Plugin.hh"
#include "math/Pose.hh"

namespace sdf
{
 
  class Sensor : public SDFBase
  {
    public: Sensor() : 
            name("name","", true), 
            type("type","", true), 
            alwaysOn("always_on", false,false), 
            updateRate("update_rate",10.0,true),
            origin("origin", gazebo::math::Pose(),true)
    { 
      this->xmlTree = "{sensor:name,type,always_on,update_rate,{origin:pose}}";
    }
  
    public: ParamT<std::string> name;
    public: ParamT<std::string> type;
    public: ParamT<bool> alwaysOn;
    public: ParamT<double> updateRate;
  
    public: ParamT<gazebo::math::Pose> origin;
  
    /// \brief complete list of plugins
    public: std::map<std::string, boost::shared_ptr<Plugin> > plugins;
  
    public: void Clear()
    {
      SDFBase::Clear();
      this->plugins.clear();
    }

    public: void Print( const std::string &_prefix )
    {
      std::cout << _prefix << "Sensor: Name[" << this->name 
                << "] Type[" << this->type << "]\n";

      std::map<std::string, boost::shared_ptr<Plugin> >::const_iterator iter;
      for (iter = this->plugins.begin(); iter != this->plugins.end(); iter++)
      {
        iter->second->Print( _prefix + "  " );
      }
    }
  };
  
  class Contact : public Sensor
  {
    public: Contact() : Sensor() {Param::End();}
  };
  
  class Camera : public Sensor
  {
    public: Camera() : Sensor(), 
            horizontalFov("angle", 0.0, true),
            imageWidth("width",0,true), 
            imageHeight("height",0,true),
            imageFormat("format","R8G8B8", false), 
            clipNear("near",0.0, true), 
            clipFar("far",1.0, false), 
            saveEnabled("enabled", false, false), 
            savePath("path","",false)
    { 
      Param::End();
      this->xmlTree="{camera:{horizontal_fov:angle},{image:width,height,format},{clip:near,far},{save:enabled,path}}";
    }

    public: ParamT<double> horizontalFov;
    public: ParamT<unsigned int> imageWidth;
    public: ParamT<unsigned int> imageHeight;
    public: ParamT<std::string> imageFormat;
    public: ParamT<double> clipNear;
    public: ParamT<double> clipFar;
    public: ParamT<bool> saveEnabled;
    public: ParamT<std::string> savePath;
  };
  
  class Ray : public Sensor
  {
    public: Ray() : Sensor(), 
            display("display", false, false),
            horizontalSamples("samples",1,true), 
            horizontalResolution("resolution", 1,true), 
            horizontalMinAngle("min_angle",0,true), 
            horizontalMaxAngle("max_angle",1,true),
            verticalSamples("samples",1,true), 
            verticalResolution("resolution", 1,true), 
            verticalMinAngle("min_angle",0,true), 
            verticalMaxAngle("max_angle",1,true),
            rangeMin("min",0,true), 
            rangeMax("max",1.0,true), 
            rangeResolution("resolution",0.0001,true)
    { 
      Param::End();
      this->xmlTree = "{ray:{scan:display,{horizontal:samples,resolution,min_angle,max_angle},{vertical:samples,resolution,min_angle,max_angle}},{range:min,max,resolution}}";
    }

    public: ParamT<bool> display;
    public: ParamT<unsigned int> horizontalSamples;
    public: ParamT<double>       horizontalResolution;
    public: ParamT<double>       horizontalMinAngle;
    public: ParamT<double>       horizontalMaxAngle;
  
    public: ParamT<unsigned int> verticalSamples;
    public: ParamT<double>       verticalResolution;
    public: ParamT<double>       verticalMinAngle;
    public: ParamT<double>       verticalMaxAngle;
  
    public: ParamT<double> rangeMin;
    public: ParamT<double> rangeMax;
    public: ParamT<double> rangeResolution;
  };

}

#endif
