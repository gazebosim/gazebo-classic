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

#include "sdf/interface/Param.hh"
#include "sdf/interface/Plugin.hh"
#include "math/Pose3d.hh"

namespace sdf
{
  class SensorType
  {
    public: enum Type{CAMERA, RAY, CONTACT};
    public: SensorType(Type _type) : type(_type) {}
    public: Type type;
  };
  
  class Sensor
  {
    public: Sensor() : name("name",""), type("type",""), 
            alwaysOn("always_on", false), updateRate("update_rate",10.0),
            origin("origin", gazebo::math::Pose3d())
    { this->Clear(); };
  
    public: ParamT<std::string, true> name;
    public: ParamT<std::string, true> type;
    public: ParamT<bool, false> alwaysOn;
    public: ParamT<double, true> updateRate;
  
    public: ParamT<gazebo::math::Pose3d, true> origin;
  
    public: boost::shared_ptr<SensorType> sensorType;

    /// \brief complete list of plugins
    public: std::map<std::string, boost::shared_ptr<Plugin> > plugins;
  
    public: void Clear()
    {
      this->name.Reset();
      this->type.Reset();
      this->alwaysOn.Reset();
      this->updateRate.Reset();
      this->origin.Reset();

      this->sensorType.reset();
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
  
  class ContactSensor : public SensorType
  {
    public: ContactSensor() : SensorType(CONTACT) {}
  };
  
  class CameraSensor : public SensorType
  {
    public: CameraSensor() : SensorType(CAMERA), horizontalFov("horizontal_fov", 0.0),
            imageWidth("width",0), imageHeight("height",0),
            imageFormat("format","R8G8B8"), clipNear("near",0.0), 
            clipFar("far",1.0), saveEnabled("enabled", false), 
            savePath("path","")
    { this->Clear(); };

    public: ParamT<double, true> horizontalFov;
    public: ParamT<unsigned int, true> imageWidth;
    public: ParamT<unsigned int, true> imageHeight;
    public: ParamT<std::string, false> imageFormat;
    public: ParamT<double, true> clipNear;
    public: ParamT<double, true> clipFar;
    public: ParamT<bool, false> saveEnabled;
    public: ParamT<std::string, false> savePath;
  
    public: void Clear()
    {
      this->horizontalFov.Reset();
      this->imageWidth.Reset();
      this->imageHeight.Reset();
      this->imageFormat.Reset();
      this->clipNear.Reset();
      this->clipFar.Reset();
      this->saveEnabled.Reset();
      this->savePath.Reset();
    }
  };
  
  class RaySensor : public SensorType
  {
    public: RaySensor() : SensorType(RAY), 
            display("display", false),
            horizontalSamples("samples",1), 
            horizontalResolution("resolution", 1), 
            horizontalMinAngle("min_angle",0), 
            horizontalMaxAngle("max_angle",1),
            verticalSamples("samples",1), 
            verticalResolution("resolution", 1), 
            verticalMinAngle("min_angle",0), 
            verticalMaxAngle("max_angle",1),
            rangeMin("min",0), rangeMax("max",1.0), 
            rangeResolution("resolution",0.0001)
    { this->Clear(); }

    public: ParamT<bool, false> display;
    public: ParamT<unsigned int, true> horizontalSamples;
    public: ParamT<double, true>       horizontalResolution;
    public: ParamT<double, true>       horizontalMinAngle;
    public: ParamT<double, true>       horizontalMaxAngle;
  
    public: ParamT<unsigned int, true> verticalSamples;
    public: ParamT<double, true>       verticalResolution;
    public: ParamT<double, true>       verticalMinAngle;
    public: ParamT<double, true>       verticalMaxAngle;
  
    public: ParamT<double, true> rangeMin;
    public: ParamT<double, true> rangeMax;
    public: ParamT<double, false> rangeResolution;
  
    public: void Clear()
    {
      this->display.Reset();
      this->horizontalSamples.Reset();
      this->horizontalResolution.Reset();
      this->horizontalMinAngle.Reset();
      this->horizontalMaxAngle.Reset();
  
      this->verticalSamples.Reset();
      this->verticalResolution.Reset();
      this->verticalMinAngle.Reset();
      this->verticalMaxAngle.Reset();
  
      this->rangeMin.Reset();
      this->rangeMax.Reset();
      this->rangeResolution.Reset();
    };
  };

}

#endif
