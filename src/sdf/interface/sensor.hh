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

#include "sdf/interface/plugin.hh"
#include "sdf/interface/pose.hh"

namespace sdf
{
  class SensorType
  {
    public: enum {CAMERA, RAY, CONTACT} type;
  };
  
  class Sensor
  {
    public: Sensor() { this->Clear(); };
  
    public: std::string name;
    public: std::string type;
    public: bool alwaysOn;
    public: double updateRate;
  
    public: Pose origin;
  
    public: boost::shared_ptr<SensorType> sensorType;

    /// \brief complete list of plugins
    public: std::map<std::string, boost::shared_ptr<Plugin> > plugins;
  
    public: void Clear()
    {
      this->name.clear();
      this->type.clear();
      this->alwaysOn = false;
      this->updateRate = -1;
      this->sensorType.reset();
      this->origin.Clear();
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
  
  class Contact : public SensorType
  {
    public: Contact() {}
  };
  
  class Camera : public SensorType
  {
    public: Camera() { this->Clear(); };
    public: double horizontalFov;
    public: unsigned int imageWidth;
    public: unsigned int imageHeight;
    public: std::string imageFormat;
    public: double clipNear;
    public: double clipFar;
    public: bool saveEnabled;
    public: std::string savePath;
  
    public: void Clear()
    {
      this->horizontalFov = 0;
      this->imageWidth = 0;
      this->imageHeight = 0;
      this->imageFormat.clear();
      this->clipNear = 0;
      this->clipFar = 0;
      this->saveEnabled = false;
      this->savePath.clear();
    }
  };
  
  class Ray : public SensorType
  {
    public: Ray() { this->Clear(); };
    public: bool display;
    public: unsigned int horizontalSamples;
    public: double       horizontalResolution;
    public: double       horizontalMinAngle;
    public: double       horizontalMaxAngle;
  
    public: unsigned int verticalSamples;
    public: double       verticalResolution;
    public: double       verticalMinAngle;
    public: double       verticalMaxAngle;
  
    public: double rangeMin;
    public: double rangeMax;
    public: double rangeResolution;
  
    public: void Clear()
    {
      this->display = false;
      this->horizontalSamples = 1;
      this->horizontalResolution = 1;
      this->horizontalMinAngle = 0;
      this->horizontalMaxAngle = 0;
  
      this->verticalSamples = 1;
      this->verticalResolution = 1;
      this->verticalMinAngle = 0;
      this->verticalMaxAngle = 0;
  
      this->rangeMin = 0;
      this->rangeMax = 1000;
      this->rangeResolution = 0;
    };
  };

}

#endif
