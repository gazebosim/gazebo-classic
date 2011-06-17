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

#ifndef URDF_SENSOR_H
#define URDF_SENSOR_H

#include <string>
#include <vector>
#include <tinyxml.h>
#include <boost/shared_ptr.hpp>

#include "pose.h"

namespace gdf{

class SensorType
{
  enum {CAMERA, RAY, CONTACT} type;

  virtual bool initXml(TiXmlElement *) = 0;
};

class Sensor
{
public:
  Sensor() { this->clear(); };

  std::string name;
  std::string type;
  bool always_on;
  double update_rate;

  Pose origin;

  boost::shared_ptr<SensorType> sensor_type;

  bool initXml(TiXmlElement* config);

  void clear()
  {
    this->name.clear();
    this->type.clear();
    this->always_on = false;
    this->update_rate = -1;
    this->sensor_type.reset();
    this->origin.clear();
  }

};



class Contact : public SensorType
{
  Contact() { }
  bool initXml(TiXmlElement *) {return true;}
};

class Camera : public SensorType
{
public:
  Camera() { this->clear(); };
  double horizontal_fov;
  unsigned int image_width;
  unsigned int image_height;
  std::string image_format;
  double clip_near;
  double clip_far;
  bool save_enabled;
  std::string save_path;

  void clear()
  {
    this->horizontal_fov = 0;
    this->image_width = 0;
    this->image_height = 0;
    this->image_format.clear();
    this->clip_near = 0;
    this->clip_far = 0;
    this->save_enabled = false;
    this->save_path.clear();
  }

  bool initXml(TiXmlElement *);
};

class Ray : public SensorType
{
public:
  Ray() { this->clear(); };
  bool display;
  unsigned int horizontal_samples;
  double       horizontal_resolution;
  double       horizontal_min_angle;
  double       horizontal_max_angle;

  unsigned int vertical_samples;
  double       vertical_resolution;
  double       vertical_min_angle;
  double       vertical_max_angle;

  double range_min;
  double range_max;
  double range_resolution;

  void clear()
  {
    this->display = false;
    this->horizontal_samples = 1;
    this->horizontal_resolution = 1;
    this->horizontal_min_angle = 0;
    this->horizontal_max_angle = 0;

    this->vertical_samples = 1;
    this->vertical_resolution = 1;
    this->vertical_min_angle = 0;
    this->vertical_max_angle = 0;

    this->range_min = 0;
    this->range_max = 1000;
    this->range_resolution = 0;
  };

  bool initXml(TiXmlElement *);
};


}

#endif
