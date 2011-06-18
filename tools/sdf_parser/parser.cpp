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

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <tinyxml.h>
#include <stdio.h>

#include "parser.h"

namespace sdf
{

bool getBoolFromStr(std::string _str, bool &_value)
{
  boost::to_lower(_str);
  if (_str == "true" || _str == "t" || _str == "1")
    _value = true;
  else if (_str == "false" || _str == "f" || _str == "0")
    _value = false;
  else
  {
    _value = false;
    return false;
  }

  return true;
}

bool getDoubleFromStr(const std::string &_str, double &_value)
{
  try
  {
    _value = boost::lexical_cast<double>(_str);
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("ERR: string is not a double format\n");
    return false;
  }

  return true;
}

bool getIntFromStr(const std::string &_str, int &_value)
{
  try
  {
    _value = boost::lexical_cast<int>(_str);
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("ERR: string is not an int format\n");
    return false;
  }

  return true;
}

bool getUIntFromStr(const std::string &_str, unsigned int &_value)
{
  try
  {
    _value = boost::lexical_cast<unsigned int>(_str);
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("ERR: string is not an unsigned int format\n");
    return false;
  }

  return true;
}

bool getPlugins(TiXmlElement *_parentXml, 
                std::map<std::string, boost::shared_ptr<Plugin> > &_plugins)
{
  // Get all plugins 
  for (TiXmlElement* pluginXml = _parentXml->FirstChildElement("plugin"); 
      pluginXml; pluginXml = _parentXml->NextSiblingElement("plugin"))
  {
    boost::shared_ptr<Plugin> plugin;
    plugin.reset(new Plugin);

    if (initXml(pluginXml, plugin))
    {
      if (_plugins.find(plugin->name) != _plugins.end())
      {
        printf("plugin '%s' is not unique.\n", plugin->name.c_str());
        plugin.reset();
        return false;
      }
      else
      {
        _plugins.insert(make_pair(plugin->name,plugin));
        printf("successfully added a new plugin '%s'\n", plugin->name.c_str());
      }
    }
    else
    {
      printf("plugin xml is not initialized correctly\n");
      plugin.reset();
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<SensorType> &_sensor_type)
{
  if (_sensor_type->type == SensorType::CAMERA)
  {
    boost::shared_ptr<Camera> camera_sensor = boost::shared_static_cast<Camera>( _sensor_type);
    //boost::shared_ptr<Camera> camera_sensor(boost::shared_dynamic_cast<Camera>( _sensor_type));
    return initXml(_config, camera_sensor);
  }
  else if (_sensor_type->type == SensorType::RAY)
  {
    boost::shared_ptr<Ray> ray_sensor(boost::shared_static_cast<Ray>( _sensor_type));
    return initXml(_config, ray_sensor);
  }
  else if (_sensor_type->type == SensorType::CONTACT)
  {
    boost::shared_ptr<Contact> contact_sensor(boost::shared_static_cast<Contact>( _sensor_type));
    return initXml(_config, contact_sensor);
  }
  else
  {
    printf("Err: SensorType init unknown type\n");
    return false;
  }

}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Sensor> &_sensor)
{
  _sensor->Clear();

  /// Get all the plugins
  getPlugins(_config, _sensor->plugins);

  const char *nameChar = _config->Attribute("name");
  if (!nameChar)
  {
    printf("No name given for the sensor.\n");
    return false;
  }
  _sensor->name = std::string(nameChar);

  const char *typeChar = _config->Attribute("type");
  if (!typeChar)
  {
    printf("No type given for the sensor.\n");
    return false;
  }
  _sensor->type = std::string(typeChar);

  if (_sensor->type == "camera")
  {
    _sensor->sensorType.reset(new Camera);
    initXml(_config, _sensor->sensorType);
  }
  else if (_sensor->type == "ray")
  {
    _sensor->sensorType.reset(new Ray);
    initXml(_config, _sensor->sensorType);
  }
  else if (_sensor->type == "contact")
  {
    _sensor->sensorType.reset(new Contact);
    initXml(_config, _sensor->sensorType);
  }
  else
  {
    printf("Error: Unknown sensor type[%s]\n",_sensor->type.c_str());
    return false;
  }



  std::string alwaysOnStr = _config->Attribute("always_on");
  if (alwaysOnStr.empty() && 
      !getBoolFromStr(alwaysOnStr, _sensor->alwaysOn))
  {
    printf("ERR: invalid always_on_str\n");
    return false;
  }

  std::string hz = _config->Attribute("update_rate");
  if (!hz.empty() && !getDoubleFromStr(hz, _sensor->updateRate))
  {
    printf("ERR: invalid update_rate\n");
    return false;
  }

  // Origin
  TiXmlElement *o = _config->FirstChildElement("origin");
  if (!o)
  {
    printf("INFO: Origin tag not present for sensor element, using default (Identity)\n\n");
    _sensor->origin.Clear();
  }
  else
  {
    if (!InitXml(o,_sensor->origin))
    {
      printf("Error: Sensor has a malformed origin tag\n");
      _sensor->origin.Clear();
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Camera> &_sensor)
{
  _sensor->Clear();

  // Horizontal field of view
  TiXmlElement *hfov = _config->FirstChildElement("horizontal_fov");
  if (!hfov)
  {
    printf("Error: Camera sensor requires a horizontal field of view via <horizontal_fov angle='radians'/>\n");
    return false;
  }

  if (!getDoubleFromStr( hfov->Attribute("angle"), _sensor->horizontalFov))
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

  if (!getUIntFromStr( image->Attribute("width"), _sensor->imageWidth))
  {
    printf("Err: Invalid image_width");
    return false;
  }

  if (!getUIntFromStr( image->Attribute("height"), _sensor->imageHeight))
  {
    printf("Err: Invalid image_height");
    return false;
  }

  _sensor->imageFormat = image->Attribute("format");

  // clip 
  TiXmlElement *clip = _config->FirstChildElement("clip");
  if (!clip)
  {
    printf("Error: Camera sensor requires an clip element \n");
    return false;
  }

  if (!getDoubleFromStr( clip->Attribute("near"), _sensor->clipNear))
  {
    printf("Err: Invalid near clip");
    return false;
  }

  if (!getDoubleFromStr( clip->Attribute("far"), _sensor->clipFar))
  {
    printf("Err: Invalid far clip");
    return false;
  }

  // save 
  TiXmlElement *save = _config->FirstChildElement("save");
  if (save)
  {
    std::string enabled = save->Attribute("enabled");
    if (enabled.empty() || !getBoolFromStr(enabled, _sensor->saveEnabled))
    {
      printf("Err: invalid save enabled flag\n");
      return false;
    }

    _sensor->savePath = save->Attribute("path");
    if (_sensor->savePath.empty())
    {
      printf("Err: invalid save path\n");
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Ray> &_sensor)
{
  _sensor->Clear();

  // scan 
  TiXmlElement *scan = _config->FirstChildElement("scan");
  if (scan)
  {
    std::string displayStr = _config->Attribute("display");
    if (!displayStr.empty() && !getBoolFromStr(displayStr, _sensor->display))
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

    if (!getUIntFromStr( horizontal->Attribute("samples"), _sensor->horizontalSamples))
    {
      printf("Err: Invalid horizontal samples");
      return false;
    }

    std::string hResStr = horizontal->Attribute("resolution"); 
    if (!hResStr.empty() && !getDoubleFromStr( hResStr, _sensor->horizontalResolution))
    {
      printf("Err: Invalid horizontal resolution");
      return false;
    }

    std::string minAngleStr = horizontal->Attribute("min_angle"); 
    if (!minAngleStr.empty() && !getDoubleFromStr( minAngleStr, _sensor->horizontalMinAngle))
    {
      printf("Err: Invalid horizontal min angle");
      return false;
    }

    std::string maxAngleStr = horizontal->Attribute("max_angle"); 
    if (!maxAngleStr.empty() && !getDoubleFromStr( maxAngleStr, _sensor->horizontalMaxAngle))
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

    if (!getUIntFromStr( vertical->Attribute("samples"), _sensor->verticalSamples))
    {
      printf("Err: Invalid vertical samples");
      return false;
    }

    std::string vResStr = vertical->Attribute("resolution"); 
    if (!vResStr.empty() && !getDoubleFromStr( vResStr, _sensor->verticalResolution))
    {
      printf("Err: Invalid vertical resolution");
      return false;
    }

    minAngleStr = vertical->Attribute("min_angle"); 
    if (!minAngleStr.empty() && !getDoubleFromStr( minAngleStr, _sensor->verticalMinAngle))
    {
      printf("Err: Invalid vertical min angle");
      return false;
    }

    maxAngleStr = vertical->Attribute("max_angle"); 
    if (!maxAngleStr.empty() && !getDoubleFromStr( maxAngleStr, _sensor->verticalMaxAngle))
    {
      printf("Err: Invalid vertical max angle");
      return false;
    }
  }

  // range 
  TiXmlElement *range = _config->FirstChildElement("range");
  if (range)
  {
    if (!getDoubleFromStr( range->Attribute("min"), _sensor->rangeMin))
    {
      printf("Err: Invalid min range\n");
      return false;
    }

    if (!getDoubleFromStr( range->Attribute("max"), _sensor->rangeMax))
    {
      printf("Err: Invalid max range\n");
      return false;
    }

    std::string res = range->Attribute("resolution");
    if (!res.empty() && !getDoubleFromStr( res, _sensor->rangeResolution))
    {
      printf("Err: Invalid range resolution\n");
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Material> &_material)
{
  bool hasRGB = false;

  _material->Clear();

  _material->script = _config->Attribute("script");

  // color
  TiXmlElement *c = _config->FirstChildElement("color");
  if (c)
  {
    if (c->Attribute("rgba"))
    {
      if (!_material->color.Init(c->Attribute("rgba")))
      {
        printf("Material has malformed color rgba values.\n");
        _material->color.Clear();
        return false;
      }
      else
        hasRGB = true;
    }
    else
    {
      printf("Material color has no rgba\n");
    }
  }

  return !_material->script.empty() || hasRGB;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Inertial> &_inertial)
{
  _inertial->Clear();

  // Origin
  TiXmlElement *o = _config->FirstChildElement("origin");
  if (!o)
  {
    printf("INFO: Origin tag not present for inertial element, using default (Identity)\n\n");
    _inertial->origin.Clear();
  }
  else
  {
    if (!InitXml(o,_inertial->origin))
    {
      printf("Inertial has a malformed origin tag\n");
      _inertial->origin.Clear();
      return false;
    }
  }

  if (!_config->Attribute("mass"))
  {
    printf("Inertial element must have mass attribute.");
    return false;
  }

  try
  {
    _inertial->mass = boost::lexical_cast<double>(_config->Attribute("mass"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("mass (%s) is not a float",_config->Attribute("mass"));
    return false;
  }

  TiXmlElement *inertiaXml = _config->FirstChildElement("inertia");
  if (!inertiaXml)
  {
    printf("Inertial element must have inertia element");
    return false;
  }

  if (!(inertiaXml->Attribute("ixx") && inertiaXml->Attribute("ixy") && 
        inertiaXml->Attribute("ixz") && inertiaXml->Attribute("iyy") && 
        inertiaXml->Attribute("iyz") && inertiaXml->Attribute("izz")))
  {
    printf("Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes");
    return false;
  }

  try
  {
    _inertial->ixx  = boost::lexical_cast<double>(inertiaXml->Attribute("ixx"));
    _inertial->ixy  = boost::lexical_cast<double>(inertiaXml->Attribute("ixy"));
    _inertial->ixz  = boost::lexical_cast<double>(inertiaXml->Attribute("ixz"));
    _inertial->iyy  = boost::lexical_cast<double>(inertiaXml->Attribute("iyy"));
    _inertial->iyz  = boost::lexical_cast<double>(inertiaXml->Attribute("iyz"));
    _inertial->izz  = boost::lexical_cast<double>(inertiaXml->Attribute("izz"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("one of the inertia elements: ixx (%s) ixy (%s) ixz (%s) iyy (%s) iyz (%s) izz (%s) is not a valid double",
              inertiaXml->Attribute("ixx"),
              inertiaXml->Attribute("ixy"),
              inertiaXml->Attribute("ixz"),
              inertiaXml->Attribute("iyy"),
              inertiaXml->Attribute("iyz"),
              inertiaXml->Attribute("izz"));
    return false;
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Collision> &_collision)
{
  _collision->Clear();

  _collision->name = _config->Attribute("name");

  // Origin
  TiXmlElement *o = _config->FirstChildElement("origin");
  if (!o)
  {
    printf("Origin tag not present for collision element, using default (Identity)");
    _collision->origin.Clear();
  }
  else if (!InitXml(o,_collision->origin))
  {
    printf("Collision has a malformed origin tag");
    _collision->origin.Clear();
    return false;
  }

  // Geometry
  TiXmlElement *geom = _config->FirstChildElement("geometry");
  boost::shared_ptr<Geometry> geometry( new Geometry);
  initXml(geom, geometry);

  if (!geometry)
  {
    printf("Malformed geometry for Collision element");
    return false;
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Sphere> &_sphere)
{
  _sphere->Clear();

  _sphere->type = Geometry::SPHERE;
  if (!_config->Attribute("radius"))
  {
    printf("Sphere shape must have a radius attribute");
    return false;
  }

  try
  {
    _sphere->radius = boost::lexical_cast<double>(_config->Attribute("radius"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("radius (%s) is not a valid float",_config->Attribute("radius"));
    return false;
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Box> &_box)
{
  _box->Clear();

  _box->type = Geometry::BOX;
  if (!_config->Attribute("size"))
  {
    printf("Box shape has no size attribute");
    return false;
  }

  if (!_box->size.Init(_config->Attribute("size")))
  {
    printf("Box shape has malformed size attribute");
    _box->size.Clear();
    return false;
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Cylinder> &_cylinder)
{
  _cylinder->Clear();

  _cylinder->type = Geometry::CYLINDER;

  if (!_config->Attribute("length") ||
      !_config->Attribute("radius"))
  {
    printf("Cylinder shape must have both length and radius attributes");
    return false;
  }

  try
  {
    _cylinder->length = boost::lexical_cast<double>(_config->Attribute("length"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("length (%s) is not a valid float",_config->Attribute("length"));
    return false;
  }

  try
  {
    _cylinder->radius = boost::lexical_cast<double>(_config->Attribute("radius"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("radius (%s) is not a valid float",_config->Attribute("radius"));
    return false;
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Mesh> &_mesh)
{
  _mesh->Clear();

  _mesh->type = Geometry::MESH;
  if (!_config->Attribute("filename"))
  {
    printf("Mesh must contain a filename attribute\n");
    return false;
  }

  _mesh->filename = _config->Attribute("filename");

  // check if filename exists, is this really necessary?
  if (!_mesh->FileExists(_mesh->filename))
    printf("filename referred by mesh [%s] does not appear to exist.\n", _mesh->filename.c_str());

  if (_config->Attribute("scale"))
  {
    if (!_mesh->scale.Init(_config->Attribute("scale")))
    {
      printf("Mesh scale was specified, but could not be parsed\n");
      _mesh->scale.Clear();
      return false;
    }
  }
  else
    printf("Mesh scale was not specified, default to (1,1,1)\n");

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Link> &_link)
{
  _link->Clear();

  const char *nameChar = _config->Attribute("name");
  if (!nameChar)
  {
    printf("No name given for the link.\n");
    return false;
  }
  _link->name = std::string(nameChar);

  // Inertial (optional)
  TiXmlElement *i = _config->FirstChildElement("inertial");
  if (i)
  {
    _link->inertial.reset(new Inertial);
    if (!initXml(i,_link->inertial))
    {
      printf("Could not parse inertial element for Link '%s'\n", _link->name.c_str());
      return false;
    }
  }

  // Multiple Visuals (optional)
  for (TiXmlElement* visXml = _config->FirstChildElement("visual"); 
       visXml; visXml = visXml->NextSiblingElement("visual"))
  {
    boost::shared_ptr<Visual> vis;
    vis.reset(new Visual);

    if (initXml(visXml,vis))
    {
      //  add Visual to the vector
      _link->visuals.push_back(vis);
      printf("successfully added a new visual\n");
    }
    else
    {
      printf("Could not parse visual element for Link '%s'\n", _link->name.c_str());
      vis.reset();
      return false;
    }
  }

  // Multiple Collisions (optional)
  for (TiXmlElement* colXml = _config->FirstChildElement("collision"); 
       colXml; colXml = colXml->NextSiblingElement("collision"))
  {
    boost::shared_ptr<Collision> col;
    col.reset(new Collision);

    if (initXml(colXml,col))
    {
      // group exists, add Collision to the vector in the map
      _link->collisions.push_back(col);
      printf("successfully added a new collision\n");
    }
    else
    {
      printf("Could not parse collision element for Link '%s'\n", _link->name.c_str());
      col.reset();
      return false;
    }
  }

  // Get all sensor elements
  for (TiXmlElement* sensorXml = _config->FirstChildElement("sensor"); 
       sensorXml; sensorXml = _config->NextSiblingElement("sensor"))
  {
    boost::shared_ptr<Sensor> sensor;
    sensor.reset(new Sensor);

    if (initXml(sensorXml,sensor))
    {
      if (_link->GetSensor(sensor->name))
      {
        printf("sensor '%s' is not unique.\n", sensor->name.c_str());
        sensor.reset();
        return false;
      }
      else
      {
        _link->sensors.insert(make_pair(sensor->name,sensor));
        printf("successfully added a new sensor '%s'\n", sensor->name.c_str());
      }
    }
    else
    {
      printf("sensor xml is not initialized correctly\n");
      sensor.reset();
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Visual> &_visual)
{
  _visual->Clear();

  _visual->name = _config->Attribute("name");

  // Origin
  TiXmlElement *o = _config->FirstChildElement("origin");
  if (!o)
  {
    printf("Origin tag not present for visual element, using default (Identity)");
    _visual->origin.Clear();
  }
  else if (!InitXml(o,_visual->origin))
  {
    printf("Visual has a malformed origin tag");
    _visual->origin.Clear();
    return false;
  }

  // Geometry
  TiXmlElement *geometryXml = _config->FirstChildElement("geometry");
  boost::shared_ptr<Geometry> geometry(new Geometry);
  initXml(geometryXml,geometry);
  if (!geometry)
  {
    printf("Malformed geometry for Visual element");
    return false;
  }

  // Material
  TiXmlElement *mat = _config->FirstChildElement("material");
  if (!mat)
  {
    printf("visual element has no material tag.");
  }
  else
  {
    // try to parse material element in place
    _visual->material.reset(new Material);
    if (!initXml(mat,_visual->material))
    {
      printf("Could not parse material element in Visual block, maybe defined outside.");
      _visual->material.reset();
    }
    else
    {
      printf("Parsed material element in Visual block.");
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<JointDynamics> &_jointDynamics)
{
  _jointDynamics->Clear();

  // Get joint damping
  const char* dampingStr = _config->Attribute("damping");
  if (dampingStr == NULL)
  {
    printf("joint dynamics: no damping, defaults to 0\n");
    _jointDynamics->damping = 0;
  }
  else
  {
    try
    {
      _jointDynamics->damping = boost::lexical_cast<double>(dampingStr);
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
    _jointDynamics->friction = 0;
  }
  else
  {
    try
    {
      _jointDynamics->friction = boost::lexical_cast<double>(frictionStr);
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
    printf("joint dynamics: damping %f and friction %f\n", _jointDynamics->damping, _jointDynamics->friction);
    return true;
  }
}


bool initXml(TiXmlElement *_config, boost::shared_ptr<JointLimits> &_jointLimits)
{
  _jointLimits->Clear();

  // Get lower joint limit
  const char* lowerStr = _config->Attribute("lower");
  if (lowerStr == NULL)
  {
    printf("joint limit: no lower, defaults to 0\n");
    _jointLimits->lower = 0;
  }
  else
  {
    try
    {
      _jointLimits->lower = boost::lexical_cast<double>(lowerStr);
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
    _jointLimits->upper = 0;
  }
  else
  {
    try
    {
      _jointLimits->upper = boost::lexical_cast<double>(upperStr);
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
      _jointLimits->effort = boost::lexical_cast<double>(effortStr);
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
      _jointLimits->velocity = boost::lexical_cast<double>(velocityStr);
    }
    catch (boost::bad_lexical_cast &e)
    {
      printf("velocity value (%s) is not a float\n",velocityStr);
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Joint> &_joint)
{
  _joint->Clear();

  // Get Joint Name
  const char *nameStr = _config->Attribute("name");
  if (!nameStr)
  {
    printf("unnamed joint found\n");
    return false;
  }
  _joint->name = nameStr;

  // Get transform from Parent Link to Joint Frame
  TiXmlElement *originXml = _config->FirstChildElement("origin");
  if (!originXml)
  {
    _joint->origin.Clear();
  }
  else
  {
    if (!InitXml(originXml,_joint->origin))
    {
      printf("Malformed parent origin element for joint '%s'\n", _joint->name.c_str());
      _joint->origin.Clear();
      return false;
    }
  }

  // Get Parent Link
  TiXmlElement *parentXml = _config->FirstChildElement("parent");
  if (parentXml)
  {
    const char *pname = parentXml->Attribute("link");
    if (!pname)
      printf("no parent link name specified for Joint link '%s'. this might be the root?\n", _joint->name.c_str());
    else
    {
      _joint->parentLinkName = std::string(pname);
    }
  }

  // Get Child Link
  TiXmlElement *childXml = _config->FirstChildElement("child");
  if (childXml)
  {
    const char *pname = childXml->Attribute("link");
    if (!pname)
      printf("no child link name specified for Joint link '%s'.\n", _joint->name.c_str());
    else
    {
      _joint->childLinkName = std::string(pname);
    }
  }

  // Get Joint type
  const char* typeChar = _config->Attribute("type");
  if (!typeChar)
  {
    printf("joint '%s' has no type, check to see if it's a reference.\n", _joint->name.c_str());
    return false;
  }
  std::string typeStr = typeChar;
  if (typeStr == "piston")
    _joint->type = Joint::PISTON;
  else if (typeStr == "revolute2")
    _joint->type = Joint::REVOLUTE2;
  else if (typeStr == "revolute")
    _joint->type = Joint::REVOLUTE;
  else if (typeStr == "universal")
    _joint->type = Joint::UNIVERSAL;
  else if (typeStr == "prismatic")
    _joint->type = Joint::PRISMATIC;
  else if (typeStr == "ball")
    _joint->type = Joint::BALL;
  else
  {
    printf("Joint '%s' has no known type '%s'\n", _joint->name.c_str(), typeStr.c_str());
    return false;
  }

  // Get Joint Axis
  if (_joint->type != Joint::BALL)
  {
    // axis
    TiXmlElement *axisXml = _config->FirstChildElement("axis");
    if (!axisXml)
    {
      printf("no axis elemement for Joint link '%s', defaulting to (1,0,0) axis\n", _joint->name.c_str());
      _joint->axis = Vector3(1.0, 0.0, 0.0);
    }
    else{
      if (!axisXml->Attribute("xyz"))
      {
        printf("no xyz attribute for axis element for Joint link '%s'\n", _joint->name.c_str());
      }
      else 
      {
        if (!_joint->axis.Init(axisXml->Attribute("xyz")))
        {
          printf("Malformed axis element for joint '%s'\n", _joint->name.c_str());
          _joint->axis.Clear();
          return false;
        }
      }
    }
  }

  // Get limit
  TiXmlElement *limitXml = _config->FirstChildElement("limit");
  if (limitXml)
  {
    _joint->limits.reset(new JointLimits);
    if (!initXml(limitXml,_joint->limits))
    {
      printf("Could not parse limit element for joint '%s'\n", _joint->name.c_str());
      _joint->limits.reset();
      return false;
    }
  }

  // Get Dynamics
  TiXmlElement *propXml = _config->FirstChildElement("dynamics");
  if (propXml)
  {
    _joint->dynamics.reset(new JointDynamics);
    if (!initXml(propXml,_joint->dynamics))
    {
      printf("Could not parse joint_dynamics element for joint '%s'\n", _joint->name.c_str());
      _joint->dynamics.reset();
      return false;
    }
  }

  return true;
}

/// \brief Load Model from TiXMLElement
bool initXml(TiXmlElement *_xml, boost::shared_ptr<Model> &_model)
{
  _model->Clear();

  printf("Parsing model xml\n");
  if (!_xml) 
    return false;

  // Get model name
  const char *nameStr = _xml->Attribute("name");
  if (!nameStr)
  {
    printf("No name given for the model.\n");
    return false;
  }
  _model->name = std::string(nameStr);

  // Get all Link elements
  for (TiXmlElement* linkXml = _xml->FirstChildElement("link"); 
       linkXml; linkXml = linkXml->NextSiblingElement("link"))
  {
    boost::shared_ptr<Link> link;
    link.reset(new Link);

    if (initXml(linkXml,link))
    {
      if (_model->GetLink(link->name))
      {
        printf("link '%s' is not unique.\n", link->name.c_str());
        link.reset();
        return false;
      }
      else
      {
        _model->links.insert(make_pair(link->name,link));
        printf("successfully added a new link '%s'\n", link->name.c_str());
      }
    }
    else
    {
      printf("link xml is not initialized correctly\n");
      link.reset();
      return false;
    }
  }

  if (_model->links.empty())
  {
    printf("No link elements found in xml file\n");
    return false;
  }

  // Get all Joint elements
  for (TiXmlElement* jointXml = _xml->FirstChildElement("joint"); 
       jointXml; jointXml = jointXml->NextSiblingElement("joint"))
  {
    boost::shared_ptr<Joint> joint;
    joint.reset(new Joint);

    if (initXml(jointXml,joint))
    {
      if (_model->GetJoint(joint->name))
      {
        printf("joint '%s' is not unique.\n", joint->name.c_str());
        joint.reset();
        return false;
      }
      else
      {
        _model->joints.insert(make_pair(joint->name,joint));
        printf("successfully added a new joint '%s'\n", joint->name.c_str());
      }
    }
    else
    {
      printf("joint xml is not initialized correctly\n");
      joint.reset();
      return false;
    }
  }

  /// Get all the plugins
  getPlugins(_xml, _model->plugins);

  return true;
}

/*
////////////////////////////////////////////////////////////////////////////////
bool initFile(const std::string &_filename, boost::shared_ptr<Model> &_model)
{
  TiXmlDocument xmlDoc;
  xmlDoc.LoadFile(_filename);

  return initDoc(&xmlDoc,_model);
}
*/

////////////////////////////////////////////////////////////////////////////////
bool initString(const std::string &_xmlString, boost::shared_ptr<Model> &_model)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());

  return initDoc(&xmlDoc, _model);
}

/// \brief Load Model from TiXMLDocument
bool initDoc(TiXmlDocument *_xmlDoc, boost::shared_ptr<Model> &_model)
{
  if (!_xmlDoc)
  {
    printf("Could not parse the xml\n");
    return false;
  }

  TiXmlElement *modelXml = _xmlDoc->FirstChildElement("model");
  if (!modelXml)
  {
    printf("Could not find the 'model' element in the xml file\n");
    return false;
  }

  return initXml(modelXml, _model);
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Geometry> &_geometry)
{
  TiXmlElement *shape = _config->FirstChildElement();
  if (!shape)
  {
    printf("Geometry tag contains no child element.\n");
    return _geometry;
  }

  std::string typeName = shape->ValueStr();
  if (typeName == "sphere")
    _geometry.reset(new Sphere);
  else if (typeName == "box")
    _geometry.reset(new Box);
  else if (typeName == "cylinder")
    _geometry.reset(new Cylinder);
  else if (typeName == "mesh")
    _geometry.reset(new Mesh);
  else
  {
    printf("Unknown geometry type '%s'\n", typeName.c_str());
    return _geometry;
  }

  // clear geom object when fails to initialize
  if (!initXml(shape, _geometry))
  {
    printf("Geometry failed to parse\n");
    _geometry.reset();
  }

  return _geometry;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Plugin> &_plugin)
{
  _plugin->Clear();

  const char *nameChar = _config->Attribute("name");
  if (!nameChar)
  {
    printf("No name given for the plugin.\n");
    return false;
  }
  _plugin->name = std::string(nameChar);

  const char *filenameChar = _config->Attribute("filename");
  if (!filenameChar)
  {
    printf("No filename given for the plugin.\n");
    return false;
  }
  _plugin->filename = std::string(filenameChar);

  return true;
}

/*
////////////////////////////////////////////////////////////////////////////////
bool InitFile(const std::string &_filename, World &_world)
{
  TiXmlDocument xmlDoc;
  xmlDoc.LoadFile(_filename);

  return Init(&xmlDoc,_world);
}
*/

////////////////////////////////////////////////////////////////////////////////
bool InitString(const std::string &_xmlString, World &_world)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());

  return Init(&xmlDoc,_world);
}

////////////////////////////////////////////////////////////////////////////////
bool initXml(TiXmlDocument *_xmlDoc, World &_world)
{
  if (!_xmlDoc)
  {
    printf("Could not parse the xml\n");
    return false;
  }

  return Init(_xmlDoc,_world);
}

////////////////////////////////////////////////////////////////////////////////
bool initXml(TiXmlElement *_worldXml, World &_world)
{
  printf("World::initXml Parsing world xml\n");
  if (!_worldXml) 
    return false;

  return Init(_worldXml,_world);
}

////////////////////////////////////////////////////////////////////////////////
bool Init(TiXmlDocument *_xmlDoc, World &_world)
{
  if (!_xmlDoc)
  {
    printf("Could not parse the xml\n");
    return false;
  }

  TiXmlElement *worldXml = _xmlDoc->FirstChildElement("world");
  if (!worldXml)
  {
    printf("Could not find the 'world' element in the xml file\n");
    return false;
  }
  return Init(worldXml,_world);
}

////////////////////////////////////////////////////////////////////////////////
bool Init(TiXmlElement *_worldXml, World &_world)
{
  _world.Clear();

  printf("Parsing the best world xml\n");

  if (!_worldXml) 
  {
    printf("Error: World XML is NULL\n");
    return false;
  }

  // Get world name
  const char *nameStr = _worldXml->Attribute("name");
  if (!nameStr)
  {
    printf("No name given for the world.\n");
    return false;
  }
  _world.name = std::string(nameStr);

  _world.scene.reset(new Scene);
  initXml(_worldXml->FirstChildElement("scene"),_world.scene);

  _world.physics.reset(new Physics);
  initXml(_worldXml->FirstChildElement("physics"),_world.physics);

  // Get all model elements
  for (TiXmlElement* modelXml = _worldXml->FirstChildElement("model"); 
       modelXml; modelXml = modelXml->NextSiblingElement("model"))
  {
    boost::shared_ptr<Model> model;
    model.reset(new Model);

    if (initXml(modelXml,model))
    {
      if (_world.GetModel(model->name))
      {
        printf("model '%s' is not unique.\n", model->name.c_str());
        model.reset();
        return false;
      }
      else
      {
        _world.models.insert(make_pair(model->name,model));
        printf("successfully added a new model '%s'\n", model->name.c_str());
      }
    }
    else
    {
      printf("model xml is not initialized correctly\n");
      model.reset();
      return false;
    }
  }

  // Get all Joint elements
  for (TiXmlElement* jointXml = _worldXml->FirstChildElement("joint"); 
       jointXml; jointXml = jointXml->NextSiblingElement("joint"))
  {
    boost::shared_ptr<Joint> joint;
    joint.reset(new Joint);

    if (initXml(jointXml,joint))
    {
      if (_world.GetJoint(joint->name))
      {
        printf("joint '%s' is not unique.\n", joint->name.c_str());
        joint.reset();
        return false;
      }
      else
      {
        _world.joints.insert(make_pair(joint->name,joint));
        printf("successfully added a new joint '%s'\n", joint->name.c_str());
      }
    }
    else
    {
      printf("joint xml is not initialized correctly\n");
      joint.reset();
      return false;
    }
  }

  /// Get all the plugins
  getPlugins(_worldXml, _world.plugins);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool initXml(TiXmlElement* _config, boost::shared_ptr<Scene> &_scene)
{
  _scene->Clear();

  TiXmlElement *ambient = _config->FirstChildElement("ambient");
  if (ambient)
  {
    if (ambient->Attribute("rgba"))
    {
      if (!_scene->ambientColor.Init(ambient->Attribute("rgba")))
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
      if (!_scene->backgroundColor.Init(background->Attribute("rgba")))
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
      _scene->skyMaterial = std::string(materialChar);
    }
  }

  TiXmlElement *shadow = _config->FirstChildElement("shadows");
  if (shadow)
  {
    std::string enabled = shadow->Attribute("enabled");
    if (!enabled.empty() && !getBoolFromStr(enabled, _scene->shadowEnabled))
    {
      printf("Error: Shadown element requires an enabled attribute");
    }

    if (shadow->Attribute("rgba"))
    {
      if (!_scene->shadowColor.Init(shadow->Attribute("rgba")))
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

    _scene->shadowType = shadow->Attribute("type");
    if (_scene->shadowType.empty())
    {
      printf("Error: Shadow requires a type attribute\n");
      return false;
    }
  }

  return true;
}

//bool initXml(TiXmlElement *_config, boost::shared_ptr<PhysicsEngine> &_physics_engine)
//{
//}

bool initXml(TiXmlElement *_config, boost::shared_ptr<OpenDynamicsEngine> &_open_dynamics_engine)
{
  TiXmlElement *odeConfig = _config->FirstChildElement("ode");

  if ( !odeConfig )
  {
    printf("Error: Physics element missing <ode>\n");
    return false;
  }

  TiXmlElement *solverConfig = odeConfig->FirstChildElement("solver");
  if (!solverConfig)
  {
    printf("Error: ODE Physics missing solver element\n");
    return false;
  }

  _open_dynamics_engine->solverType = solverConfig->Attribute("type");
  if (_open_dynamics_engine->solverType.empty())
  {
    printf("Error: ODE Physics missing solver type\n");
    return false;
  }

  std::string dtStr = solverConfig->Attribute("dt");
  if (dtStr.empty())
  {
    printf("Error: ODE Physics solver missing dt attribute\n");
    return false;
  }
  if (!getDoubleFromStr(dtStr, _open_dynamics_engine->dt))
  {
    printf("Error: ODE Physics solver malformed dt attribute\n");
    return false;
  }

  std::string itersStr = solverConfig->Attribute("iters");
  if (itersStr.empty())
  {
    printf("Error: ODE Physics solver missing iters attribute\n");
    return false;
  }
  if (!getIntFromStr(itersStr, _open_dynamics_engine->iters))
  {
    printf("Error: ODE Physics solver malformed iters attribute\n");
    return false;
  }

  std::string sorStr = solverConfig->Attribute("sor");
  if (sorStr.empty())
  {
    printf("Error: ODE Physics solver missing sor attribute\n");
    return false;
  }
  if (!getDoubleFromStr(sorStr, _open_dynamics_engine->sor))
  {
    printf("Error: ODE Physics solver malformed sor attribute\n");
    return false;
  }


  // Contraints
  TiXmlElement *constraintsConfig = odeConfig->FirstChildElement("constraints");
  if (constraintsConfig)
  {
    std::string cfmStr = constraintsConfig->Attribute("cfm");
    if (cfmStr.empty())
    {
      printf("Error: ODE Physics contraints missing cfm attribute\n");
      return false;
    }
    if (!getDoubleFromStr(cfmStr, _open_dynamics_engine->cfm))
    {
      printf("Error: ODE Physics contraints malformed cfm attribute\n");
      return false;
    }

    std::string erpStr = constraintsConfig->Attribute("erp");
    if (erpStr.empty())
    {
      printf("Error: ODE Physics contraints missing erp attribute\n");
      return false;
    }
    if (!getDoubleFromStr(erpStr, _open_dynamics_engine->erp))
    {
      printf("Error: ODE Physics contraints malformed erp attribute\n");
      return false;
    }

    std::string contactMaxCorrectingVelStr = constraintsConfig->Attribute("contact_max_correcting_vel");
    if (contactMaxCorrectingVelStr.empty())
    {
      printf("Error: ODE Physics contraints missing contact_max_correcting_vel attribute\n");
      return false;
    }
    if (!getDoubleFromStr(contactMaxCorrectingVelStr, _open_dynamics_engine->contactMaxCorrectingVel))
    {
      printf("Error: ODE Physics contraints malformed contact_max_correcting_vel attribute\n");
      return false;
    } 
 
    std::string contactSurfaceLayerStr = constraintsConfig->Attribute("contact_surface_layer");
    if (contactSurfaceLayerStr.empty())
    {
      printf("Error: ODE Physics contraints missing contact_surface_layer attribute\n");
      return false;
    }
    if (!getDoubleFromStr(contactSurfaceLayerStr, _open_dynamics_engine->contactSurfaceLayer))
    {
      printf("Error: ODE Physics contraints malformed contact_surface_layer attribute\n");
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement* _config, boost::shared_ptr<Physics> &_physics)
{
  _physics->Clear();
  
  if (!_config)
  {
    printf("Error: xml config is NULL\n");
    return false;
  }

  _physics->type = _config->Attribute("type");
  if (_physics->type.empty())
  {
    printf("Error: Missing physics type attribute\n");
    return false;
  }

  TiXmlElement *gravityElement = _config->FirstChildElement("gravity");
  if (gravityElement)
  {
    if (!_physics->gravity.Init(gravityElement->Attribute("xyz")))
    {
      printf("Gravity has malformed xyz\n");
      _physics->gravity.Clear();
      return false;
    }
  }

  if (_physics->type == "ode")
  {
    _physics->engine.reset(new OpenDynamicsEngine );

    boost::shared_ptr<OpenDynamicsEngine> open_dynamics_engine = boost::shared_static_cast<OpenDynamicsEngine>(_physics->engine);
    
    initXml(_config,open_dynamics_engine);
  }
  else
  {
    printf("Error: Unknown physics engine type[%s]\n",_physics->type.c_str());
    return false;
  }
  
  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Contact> &_contact)
{
  return true;
}

bool InitXml(TiXmlElement *_xml, Pose &_pose)
{
  _pose.Clear();
  if (!_xml)
  {
    printf("parsing pose: _xml empty");
    return false;
  }
  else
  {
    const char* _xyzStr = _xml->Attribute("xyz");
    if (_xyzStr == NULL)
    {
      printf("parsing pose: no xyz, using default values.");
      return true;
    }
    else
    {
      if (!_pose.position.Init(_xyzStr))
      {
        printf("malformed xyz");
        _pose.position.Clear();
        return false;
      }
    }

    const char* rpyStr = _xml->Attribute("rpy");
    if (rpyStr == NULL)
    {
      printf("parsing pose: no rpy, using default values.");
      return true;
    }
    else
    {
      if (!_pose.rotation.Init(rpyStr))
      {
        printf("malformed rpy");
        return false;
        _pose.rotation.Clear();
      }
    }

    return true;
  }
}


}

