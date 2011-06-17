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

    if (plugin->InitXml(pluginXml))
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

bool initXml(TiXmlElement *_config, boost::shared_ptr<Sensor> &_sensor)
{
  _sensor.Clear();

  /// Get all the plugins
  getPlugins(_config, _sensor.plugins);

  const char *nameChar = _config->Attribute("name");
  if (!nameChar)
  {
    printf("No name given for the sensor.\n");
    return false;
  }
  _sensor.name = std::string(nameChar);

  const char *typeChar = _config->Attribute("type");
  if (!typeChar)
  {
    printf("No type given for the sensor.\n");
    return false;
  }
  _sensor.type = std::string(typeChar);

  if (_sensor.type == "camera")
  {
    _sensor.sensorType.reset(new Camera);
    initXml(_config, _sensor.sensorType);
  }
  else if (_sensor.type == "ray")
  {
    _sensor.sensorType.reset(new Ray);
    initXml(_config, _sensor.sensorType);
  }
  else if (_sensor.type == "contact")
  {
    _sensor.sensorType.reset(new Contact);
    initXml(_config, _sensor.sensorType);
  }
  else
  {
    printf("Error: Unknown sensor type[%s]\n",_sensor.type.c_str());
    return false;
  }



  std::string alwaysOnStr = _config->Attribute("always_on");
  if (alwaysOnStr.empty() && 
      !getBoolFromStr(alwaysOnStr, _sensor.alwaysOn))
  {
    printf("ERR: invalid always_on_str\n");
    return false;
  }

  std::string hz = _config->Attribute("update_rate");
  if (!hz.empty() && !getDoubleFromStr(hz, _sensor.updateRate))
  {
    printf("ERR: invalid update_rate\n");
    return false;
  }

  // Origin
  TiXmlElement *o = _config->FirstChildElement("origin");
  if (!o)
  {
    printf("INFO: Origin tag not present for sensor element, using default (Identity)\n\n");
    _sensor.origin.Clear();
  }
  else
  {
    if (!_sensor.origin.InitXml(o))
    {
      printf("Error: Sensor has a malformed origin tag\n");
      _sensor.origin.Clear();
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Camera> &_sensor)
{
  _sensor.Clear();

  // Horizontal field of view
  TiXmlElement *hfov = _config->FirstChildElement("horizontal_fov");
  if (!hfov)
  {
    printf("Error: Camera sensor requires a horizontal field of view via <horizontal_fov angle='radians'/>\n");
    return false;
  }

  if (!getDoubleFromStr( hfov->Attribute("angle"), _sensor.horizontalFov))
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

  if (!getUIntFromStr( image->Attribute("width"), _sensor.imageWidth))
  {
    printf("Err: Invalid image_width");
    return false;
  }

  if (!getUIntFromStr( image->Attribute("height"), _sensor.imageHeight))
  {
    printf("Err: Invalid image_height");
    return false;
  }

  _sensor.imageFormat = image->Attribute("format");

  // clip 
  TiXmlElement *clip = _config->FirstChildElement("clip");
  if (!clip)
  {
    printf("Error: Camera sensor requires an clip element \n");
    return false;
  }

  if (!getDoubleFromStr( clip->Attribute("near"), _sensor.clipNear))
  {
    printf("Err: Invalid near clip");
    return false;
  }

  if (!getDoubleFromStr( clip->Attribute("far"), _sensor.clipFar))
  {
    printf("Err: Invalid far clip");
    return false;
  }

  // save 
  TiXmlElement *save = _config->FirstChildElement("save");
  if (save)
  {
    std::string enabled = save->Attribute("enabled");
    if (enabled.empty() || !getBoolFromStr(enabled, _sensor.saveEnabled))
    {
      printf("Err: invalid save enabled flag\n");
      return false;
    }

    _sensor.savePath = save->Attribute("path");
    if (_sensor.savePath.empty())
    {
      printf("Err: invalid save path\n");
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Ray> &_sensor)
{
  _sensor.Clear();

  // scan 
  TiXmlElement *scan = _config->FirstChildElement("scan");
  if (scan)
  {
    std::string displayStr = _config->Attribute("display");
    if (!displayStr.empty() && !getBoolFromStr(displayStr, _sensor.display))
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

    if (!getUIntFromStr( horizontal->Attribute("samples"), _sensor.horizontalSamples))
    {
      printf("Err: Invalid horizontal samples");
      return false;
    }

    std::string hResStr = horizontal->Attribute("resolution"); 
    if (!hResStr.empty() && !getDoubleFromStr( hResStr, _sensor.horizontalResolution))
    {
      printf("Err: Invalid horizontal resolution");
      return false;
    }

    std::string minAngleStr = horizontal->Attribute("min_angle"); 
    if (!minAngleStr.empty() && !getDoubleFromStr( minAngleStr, _sensor.horizontalMinAngle))
    {
      printf("Err: Invalid horizontal min angle");
      return false;
    }

    std::string maxAngleStr = horizontal->Attribute("max_angle"); 
    if (!maxAngleStr.empty() && !getDoubleFromStr( maxAngleStr, _sensor.horizontalMaxAngle))
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

    if (!getUIntFromStr( vertical->Attribute("samples"), _sensor.verticalSamples))
    {
      printf("Err: Invalid vertical samples");
      return false;
    }

    std::string vResStr = vertical->Attribute("resolution"); 
    if (!vResStr.empty() && !getDoubleFromStr( vResStr, _sensor.verticalResolution))
    {
      printf("Err: Invalid vertical resolution");
      return false;
    }

    minAngleStr = vertical->Attribute("min_angle"); 
    if (!minAngleStr.empty() && !getDoubleFromStr( minAngleStr, _sensor.verticalMinAngle))
    {
      printf("Err: Invalid vertical min angle");
      return false;
    }

    maxAngleStr = vertical->Attribute("max_angle"); 
    if (!maxAngleStr.empty() && !getDoubleFromStr( maxAngleStr, _sensor.verticalMaxAngle))
    {
      printf("Err: Invalid vertical max angle");
      return false;
    }
  }

  // range 
  TiXmlElement *range = _config->FirstChildElement("range");
  if (range)
  {
    if (!getDoubleFromStr( range->Attribute("min"), _sensor.rangeMin))
    {
      printf("Err: Invalid min range\n");
      return false;
    }

    if (!getDoubleFromStr( range->Attribute("max"), _sensor.rangeMax))
    {
      printf("Err: Invalid max range\n");
      return false;
    }

    std::string res = range->Attribute("resolution");
    if (!res.empty() && !getDoubleFromStr( res, _sensor.rangeResolution))
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

  _material.Clear();

  _material.script = _config->Attribute("script");

  // color
  TiXmlElement *c = _config->FirstChildElement("color");
  if (c)
  {
    if (c->Attribute("rgba"))
    {
      if (!_material.color.Init(c->Attribute("rgba")))
      {
        printf("Material has malformed color rgba values.\n");
        _material.color.Clear();
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

  return !_material.script.empty() || hasRGB;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Inertial> &_inertial);
{
  _inertial.Clear();

  // Origin
  TiXmlElement *o = _config->FirstChildElement("origin");
  if (!o)
  {
    printf("INFO: Origin tag not present for inertial element, using default (Identity)\n\n");
    _inertial.origin.Clear();
  }
  else
  {
    if (!_inertial.origin.InitXml(o))
    {
      printf("Inertial has a malformed origin tag\n");
      _inertial.origin.Clear();
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
    mass = boost::lexical_cast<double>(_config->Attribute("mass"));
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
    ixx  = boost::lexical_cast<double>(inertiaXml->Attribute("ixx"));
    ixy  = boost::lexical_cast<double>(inertiaXml->Attribute("ixy"));
    ixz  = boost::lexical_cast<double>(inertiaXml->Attribute("ixz"));
    iyy  = boost::lexical_cast<double>(inertiaXml->Attribute("iyy"));
    iyz  = boost::lexical_cast<double>(inertiaXml->Attribute("iyz"));
    izz  = boost::lexical_cast<double>(inertiaXml->Attribute("izz"));
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

/* John below this line */

////////////////////////////////////////////////////////////////////////////////
bool InitFile(const std::string &_filename, boost::shared_ptr<World> &_world)
{
  TiXmlDocument xmlDoc;
  xmlDoc.LoadFile(_filename);

  return Init(&xmlDoc,_world);
}

////////////////////////////////////////////////////////////////////////////////
bool InitString(const std::string &_xmlString, boost::shared_ptr<World> &_world)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());

  return Init(&xmlDoc,_world);
}

////////////////////////////////////////////////////////////////////////////////
bool InitXml(TiXmlDocument *_xmlDoc, boost::shared_ptr<World> &_world)
{
  if (!_xmlDoc)
  {
    printf("Could not parse the xml\n");
    return false;
  }

  return Init(_xmlDoc,_world);
}

////////////////////////////////////////////////////////////////////////////////
bool InitXml(TiXmlElement *_worldXml, boost::shared_ptr<World> &_world)
{
  printf("World::InitXml Parsing world xml\n");
  if (!_worldXml) 
    return false;

  return Init(_worldXml,_world);
}

////////////////////////////////////////////////////////////////////////////////
bool Init(TiXmlDocument *_xmlDoc, boost::shared_ptr<World> &_world)
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
bool Init(TiXmlElement *_worldXml, boost::shared_ptr<World> &_world)
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
  InitXml(_worldXml->FirstChildElement("scene"),_world.scene);

  _world.physics.reset(new Physics);
  InitXml(_worldXml->FirstChildElement("physics"),_world.physics);

  // Get all model elements
  for (TiXmlElement* modelXml = _worldXml->FirstChildElement("model"); 
       modelXml; modelXml = modelXml->NextSiblingElement("model"))
  {
    boost::shared_ptr<Model> model;
    model.reset(new Model);

    if (InitXml(modelXml,model))
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

    if (InitXml(jointXml,joint))
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
bool InitXml(TiXmlElement* _config, boost::shared_ptr<Scene> &_scene)
{
  _scene.Clear();

  TiXmlElement *ambient = _config->FirstChildElement("ambient");
  if (ambient)
  {
    if (ambient->Attribute("rgba"))
    {
      if (!_scene.ambientColor.Init(ambient->Attribute("rgba")))
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
      if (!_scene.backgroundColor.Init(background->Attribute("rgba")))
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
      _scene.skyMaterial = std::string(materialChar);
    }
  }

  TiXmlElement *shadow = _config->FirstChildElement("shadows");
  if (shadow)
  {
    std::string enabled = shadow->Attribute("enabled");
    if (!enabled.empty() && !getBoolFromStr(enabled, _scene.shadowEnabled))
    {
      printf("Error: Shadown element requires an enabled attribute");
    }

    if (shadow->Attribute("rgba"))
    {
      if (!_scene.shadowColor.Init(shadow->Attribute("rgba")))
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

    _scene.shadowType = shadow->Attribute("type");
    if (_scene.shadowType.empty())
    {
      printf("Error: Shadow requires a type attribute\n");
      return false;
    }
  }

  return true;
}

bool InitXml(TiXmlElement *_config, boost::shared_ptr<OpenDynamicsEngine> &_open_dynamics_engine)
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

  _open_dynamics_engine.solverType = solverConfig->Attribute("type");
  if (_open_dynamics_engine.solverType.empty())
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
  if (!getDoubleFromStr(dtStr, _open_dynamics_engine.dt))
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
  if (!getIntFromStr(itersStr, _open_dynamics_engine.iters))
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
  if (!getDoubleFromStr(sorStr, _open_dynamics_engine.sor))
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
    if (!getDoubleFromStr(cfmStr, _open_dynamics_engine.cfm))
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
    if (!getDoubleFromStr(erpStr, _open_dynamics_engine.erp))
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
    if (!getDoubleFromStr(contactMaxCorrectingVelStr, _open_dynamics_engine.contactMaxCorrectingVel))
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
    if (!getDoubleFromStr(contactSurfaceLayerStr, _open_dynamics_engine.contactSurfaceLayer))
    {
      printf("Error: ODE Physics contraints malformed contact_surface_layer attribute\n");
      return false;
    }
  }

  return true;
}

bool InitXml(TiXmlElement* _config, boost::shared_ptr<Physics> &_physics)
{
  _physics.Clear();
  
  if (!_config)
  {
    printf("Error: xml config is NULL\n");
    return false;
  }

  _physics.type = _config->Attribute("type");
  if (_physics.type.empty())
  {
    printf("Error: Missing physics type attribute\n");
    return false;
  }

  TiXmlElement *gravityElement = _config->FirstChildElement("gravity");
  if (gravityElement)
  {
    if (!Init(gravityElement->Attribute("xyz"),_physics.gravity))
    {
      printf("Gravity has malformed xyz\n");
      _physics.gravity.Clear();
      return false;
    }
  }

  if (_physics.type == "ode")
  {
    _physics.engine.reset(new OpenDynamicsEngine );
  }
  else
  {
    printf("Error: Unknown physics engine type[%s]\n",_physics.type.c_str());
    return false;
  }
  InitXml(_config,_physics.engine);
  
  return true;
}

bool Init(const std::string &_vectorStr, boost::shared_ptr<Color> &_color)
{
  _color.Clear();

  std::vector<std::string> pieces;
  std::vector<float> rgba;
  boost::split( pieces, _vectorStr, boost::is_any_of(" "));

  for (unsigned int i = 0; i < pieces.size(); ++i)
  {
    if (!pieces[i].empty())
    {
      try
      {
        rgba.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
      }
      catch (boost::bad_lexical_cast &e)
      {
        printf("color rgba element (%s) is not a valid float",pieces[i].c_str());
        return false;
      }
    }
  }

  if (rgba.size() != 4)
  {
    printf("Color contains %i elements instead of 4 elements", (int)rgba.size());
    return false;
  }

  _color.r = rgba[0];
  _color.g = rgba[1];
  _color.b = rgba[2];
  _color.a = rgba[3];

  return true;
}

}

