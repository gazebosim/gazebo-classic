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


#include "sensor.h"
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <algorithm>

namespace gdf{

bool getBoolFromStr(std::string str, bool &value)
{
  boost::to_lower(str);
  if (str == "true" || str == "t" || str == "1")
    value = true;
  else if (str == "false" || str == "f" || str == "0")
    value = false;
  else
  {
    value = false;
    return false;
  }

  return true;
}

bool getDoubleFromStr(const std::string &str, double &value)
{
  try
  {
    value = boost::lexical_cast<double>(str);
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("ERR: string is not a double format\n");
    return false;
  }

  return true;
}

bool geIntFromStr(const std::string &str, int &value)
{
  try
  {
    value = boost::lexical_cast<int>(str);
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("ERR: string is not an int format\n");
    return false;
  }

  return true;
}

bool getUIntFromStr(const std::string &str, unsigned int &value)
{
  try
  {
    value = boost::lexical_cast<unsigned int>(str);
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("ERR: string is not an unsigned int format\n");
    return false;
  }

  return true;
}

bool Camera::initXml(TiXmlElement *config)
{
  this->clear();

  // Horizontal field of view
  TiXmlElement *hfov = config->FirstChildElement("horizontal_fov");
  if (!hfov)
  {
    printf("Error: Camera sensor requires a horizontal field of view via <horizontal_fov angle='radians'/>\n");
    return false;
  }

  if (!getDoubleFromStr( hfov->Attribute("angle"), this->horizontal_fov))
  {
    printf("Err: Invalid horizontal_fov");
    return false;
  }

  // Image 
  TiXmlElement *image = config->FirstChildElement("image");
  if (!image)
  {
    printf("Error: Camera sensor requires an image element \n");
    return false;
  }

  if (!getUIntFromStr( image->Attribute("width"), this->image_width))
  {
    printf("Err: Invalid image_width");
    return false;
  }

  if (!getUIntFromStr( image->Attribute("height"), this->image_height))
  {
    printf("Err: Invalid image_height");
    return false;
  }

  this->image_format = image->Attribute("format");

  // clip 
  TiXmlElement *clip = config->FirstChildElement("clip");
  if (!clip)
  {
    printf("Error: Camera sensor requires an clip element \n");
    return false;
  }

  if (!getDoubleFromStr( clip->Attribute("near"), this->clip_near))
  {
    printf("Err: Invalid near clip");
    return false;
  }

  if (!getDoubleFromStr( clip->Attribute("far"), this->clip_far))
  {
    printf("Err: Invalid far clip");
    return false;
  }

  // save 
  TiXmlElement *save = config->FirstChildElement("save");
  if (save)
  {
    std::string enabled = save->Attribute("enabled");
    if (enabled.empty() || !getBoolFromStr(enabled, this->save_enabled))
    {
      printf("Err: invalid save enabled flag\n");
      return false;
    }

    this->save_path = save->Attribute("path");
    if (this->save_path.empty())
    {
      printf("Err: invalid save path\n");
      return false;
    }
  }

  return true;
}

bool Ray::initXml(TiXmlElement *config)
{
  this->clear();

  // scan 
  TiXmlElement *scan = config->FirstChildElement("scan");
  if (scan)
  {
    std::string display_str = config->Attribute("display");
    if (!display_str.empty() && !getBoolFromStr(display_str, this->display))
    {
      printf("Err: invalid display flag\n");
      return false;
    }

    // Horizontal scans
    TiXmlElement *horizontal = config->FirstChildElement("horizontal");
    if (!horizontal)
    {
      printf("Err: missing horizontal element\n");
      return false;
    }

    if (!getUIntFromStr( horizontal->Attribute("samples"), this->horizontal_samples))
    {
      printf("Err: Invalid horizontal samples");
      return false;
    }

    std::string h_res_str = horizontal->Attribute("resolution"); 
    if (!h_res_str.empty() && !getDoubleFromStr( h_res_str, this->horizontal_resolution))
    {
      printf("Err: Invalid horizontal resolution");
      return false;
    }

    std::string min_angle_str = horizontal->Attribute("min_angle"); 
    if (!min_angle_str.empty() && !getDoubleFromStr( min_angle_str, this->horizontal_min_angle))
    {
      printf("Err: Invalid horizontal min angle");
      return false;
    }

    std::string max_angle_str = horizontal->Attribute("max_angle"); 
    if (!max_angle_str.empty() && !getDoubleFromStr( max_angle_str, this->horizontal_max_angle))
    {
      printf("Err: Invalid horizontal max angle");
      return false;
    }

    // Vertical scans
    TiXmlElement *vertical = config->FirstChildElement("vertical");
    if (!vertical)
    {
      printf("Err: missing vertical element\n");
      return false;
    }

    if (!getUIntFromStr( vertical->Attribute("samples"), this->vertical_samples))
    {
      printf("Err: Invalid vertical samples");
      return false;
    }

    std::string v_res_str = vertical->Attribute("resolution"); 
    if (!v_res_str.empty() && !getDoubleFromStr( v_res_str, this->vertical_resolution))
    {
      printf("Err: Invalid vertical resolution");
      return false;
    }

    min_angle_str = vertical->Attribute("min_angle"); 
    if (!min_angle_str.empty() && !getDoubleFromStr( min_angle_str, this->vertical_min_angle))
    {
      printf("Err: Invalid vertical min angle");
      return false;
    }

    max_angle_str = vertical->Attribute("max_angle"); 
    if (!max_angle_str.empty() && !getDoubleFromStr( max_angle_str, this->vertical_max_angle))
    {
      printf("Err: Invalid vertical max angle");
      return false;
    }
  }

  // range 
  TiXmlElement *range = config->FirstChildElement("range");
  if (range)
  {
    if (!getDoubleFromStr( range->Attribute("min"), this->range_min))
    {
      printf("Err: Invalid min range\n");
      return false;
    }

    if (!getDoubleFromStr( range->Attribute("max"), this->range_max))
    {
      printf("Err: Invalid max range\n");
      return false;
    }

    std::string res = range->Attribute("resolution");
    if (!res.empty() && !getDoubleFromStr( res, this->range_resolution))
    {
      printf("Err: Invalid range resolution\n");
      return false;
    }
  }

  return true;
}




bool Sensor::initXml(TiXmlElement* config)
{
  this->clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    printf("No name given for the sensor.\n");
    return false;
  }
  this->name = std::string(name_char);

  const char *type_char = config->Attribute("type");
  if (!type_char)
  {
    printf("No type given for the sensor.\n");
    return false;
  }
  this->type = std::string(type_char);

  std::string always_on_str = config->Attribute("always_on");
  if (always_on_str.empty() && 
      !getBoolFromStr(always_on_str, this->always_on))
  {
    printf("ERR: invalid always_on_str\n");
    return false;
  }

  std::string hz = config->Attribute("update_rate");
  if (!hz.empty() && !getDoubleFromStr(hz, this->update_rate))
  {
    printf("ERR: invalid update_rate\n");
    return false;
  }

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (!o)
  {
    printf("INFO: Origin tag not present for sensor element, using default (Identity)\n\n");
    this->origin.clear();
  }
  else
  {
    if (!this->origin.initXml(o))
    {
      printf("Error: Sensor has a malformed origin tag\n");
      this->origin.clear();
      return false;
    }
  }

  return true;
}


}
