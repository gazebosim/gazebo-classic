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


#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <algorithm>

#include "parser.h"
#include "sensor.h"

using namespace sdf;

////////////////////////////////////////////////////////////////////////////////
bool Camera::InitXml(TiXmlElement *_config)
{
  this->Clear();

  // Horizontal field of view
  TiXmlElement *hfov = _config->FirstChildElement("horizontal_fov");
  if (!hfov)
  {
    printf("Error: Camera sensor requires a horizontal field of view via <horizontal_fov angle='radians'/>\n");
    return false;
  }

  if (!getDoubleFromStr( hfov->Attribute("angle"), this->horizontalFov))
  {
    printf("Err: Invalid horizontal_fov");
    return false;
  }

  // Image 
  TiXmlElement *image = _config->FirstChildElement("image");
  if (!image)
  {
    printf("Error: Camera sensor requires an image element \n");
    return false;
  }

  if (!getUIntFromStr( image->Attribute("width"), this->imageWidth))
  {
    printf("Err: Invalid image_width");
    return false;
  }

  if (!getUIntFromStr( image->Attribute("height"), this->imageHeight))
  {
    printf("Err: Invalid image_height");
    return false;
  }

  this->imageFormat = image->Attribute("format");

  // clip 
  TiXmlElement *clip = _config->FirstChildElement("clip");
  if (!clip)
  {
    printf("Error: Camera sensor requires an clip element \n");
    return false;
  }

  if (!getDoubleFromStr( clip->Attribute("near"), this->clipNear))
  {
    printf("Err: Invalid near clip");
    return false;
  }

  if (!getDoubleFromStr( clip->Attribute("far"), this->clipFar))
  {
    printf("Err: Invalid far clip");
    return false;
  }

  // save 
  TiXmlElement *save = _config->FirstChildElement("save");
  if (save)
  {
    std::string enabled = save->Attribute("enabled");
    if (enabled.empty() || !getBoolFromStr(enabled, this->saveEnabled))
    {
      printf("Err: invalid save enabled flag\n");
      return false;
    }

    this->savePath = save->Attribute("path");
    if (this->savePath.empty())
    {
      printf("Err: invalid save path\n");
      return false;
    }
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool Ray::InitXml(TiXmlElement *_config)
{
  this->Clear();

  // scan 
  TiXmlElement *scan = _config->FirstChildElement("scan");
  if (scan)
  {
    std::string displayStr = _config->Attribute("display");
    if (!displayStr.empty() && !getBoolFromStr(displayStr, this->display))
    {
      printf("Err: invalid display flag\n");
      return false;
    }

    // Horizontal scans
    TiXmlElement *horizontal = _config->FirstChildElement("horizontal");
    if (!horizontal)
    {
      printf("Err: missing horizontal element\n");
      return false;
    }

    if (!getUIntFromStr( horizontal->Attribute("samples"), this->horizontalSamples))
    {
      printf("Err: Invalid horizontal samples");
      return false;
    }

    std::string hResStr = horizontal->Attribute("resolution"); 
    if (!hResStr.empty() && !getDoubleFromStr( hResStr, this->horizontalResolution))
    {
      printf("Err: Invalid horizontal resolution");
      return false;
    }

    std::string minAngleStr = horizontal->Attribute("min_angle"); 
    if (!minAngleStr.empty() && !getDoubleFromStr( minAngleStr, this->horizontalMinAngle))
    {
      printf("Err: Invalid horizontal min angle");
      return false;
    }

    std::string maxAngleStr = horizontal->Attribute("max_angle"); 
    if (!maxAngleStr.empty() && !getDoubleFromStr( maxAngleStr, this->horizontalMaxAngle))
    {
      printf("Err: Invalid horizontal max angle");
      return false;
    }

    // Vertical scans
    TiXmlElement *vertical = _config->FirstChildElement("vertical");
    if (!vertical)
    {
      printf("Err: missing vertical element\n");
      return false;
    }

    if (!getUIntFromStr( vertical->Attribute("samples"), this->verticalSamples))
    {
      printf("Err: Invalid vertical samples");
      return false;
    }

    std::string vResStr = vertical->Attribute("resolution"); 
    if (!vResStr.empty() && !getDoubleFromStr( vResStr, this->verticalResolution))
    {
      printf("Err: Invalid vertical resolution");
      return false;
    }

    minAngleStr = vertical->Attribute("min_angle"); 
    if (!minAngleStr.empty() && !getDoubleFromStr( minAngleStr, this->verticalMinAngle))
    {
      printf("Err: Invalid vertical min angle");
      return false;
    }

    maxAngleStr = vertical->Attribute("max_angle"); 
    if (!maxAngleStr.empty() && !getDoubleFromStr( maxAngleStr, this->verticalMaxAngle))
    {
      printf("Err: Invalid vertical max angle");
      return false;
    }
  }

  // range 
  TiXmlElement *range = _config->FirstChildElement("range");
  if (range)
  {
    if (!getDoubleFromStr( range->Attribute("min"), this->rangeMin))
    {
      printf("Err: Invalid min range\n");
      return false;
    }

    if (!getDoubleFromStr( range->Attribute("max"), this->rangeMax))
    {
      printf("Err: Invalid max range\n");
      return false;
    }

    std::string res = range->Attribute("resolution");
    if (!res.empty() && !getDoubleFromStr( res, this->rangeResolution))
    {
      printf("Err: Invalid range resolution\n");
      return false;
    }
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool Sensor::InitXml(TiXmlElement* _config)
{
  this->Clear();

  /// Get all the plugins
  getPlugins(_config, this->plugins);

  const char *nameChar = _config->Attribute("name");
  if (!nameChar)
  {
    printf("No name given for the sensor.\n");
    return false;
  }
  this->name = std::string(nameChar);

  const char *typeChar = _config->Attribute("type");
  if (!typeChar)
  {
    printf("No type given for the sensor.\n");
    return false;
  }
  this->type = std::string(typeChar);

  std::string alwaysOnStr = _config->Attribute("always_on");
  if (alwaysOnStr.empty() && 
      !getBoolFromStr(alwaysOnStr, this->alwaysOn))
  {
    printf("ERR: invalid always_on_str\n");
    return false;
  }

  std::string hz = _config->Attribute("update_rate");
  if (!hz.empty() && !getDoubleFromStr(hz, this->updateRate))
  {
    printf("ERR: invalid update_rate\n");
    return false;
  }

  // Origin
  TiXmlElement *o = _config->FirstChildElement("origin");
  if (!o)
  {
    printf("INFO: Origin tag not present for sensor element, using default (Identity)\n\n");
    this->origin.Clear();
  }
  else
  {
    if (!this->origin.InitXml(o))
    {
      printf("Error: Sensor has a malformed origin tag\n");
      this->origin.Clear();
      return false;
    }
  }

  return true;
}
