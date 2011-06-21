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

#include "common/Console.hh"
#include "sdf/parser/parser.hh"

namespace sdf
{

bool getBoolFromStr(std::string _str, bool &_value, bool _default, 
                    bool _required)
{
  _value = _default;

  if (_str.empty() && _required)
  {
    gzerr << "Required key is empty\n";
    return false;
  }
  
  boost::to_lower(_str);
  if (_str == "true" || _str == "t" || _str == "1")
    _value = true;
  else if (_str == "false" || _str == "f" || _str == "0")
    _value = false;
  else
  {
    gzerr << "Malformed boolean string[" << _str << "]\n";
    return false;
  }

  return true;
}

bool getDoubleFromStr(const std::string &_str, double &_value, double _default,
                      bool _required)
{
  _value = _default;

  if (_str.empty() && _required)
  {
    gzerr << "Required key is empty\n";
    return false;
  }
 
  try
  {
    _value = boost::lexical_cast<double>(_str);
  }
  catch (boost::bad_lexical_cast &e)
  {
    _value = _default;
    gzerr << "Malformed double string[" << _str << "]\n";
    return false;
  }

  return true;
}

bool getPlugins(TiXmlElement *_parentXml, 
                std::map<std::string, boost::shared_ptr<Plugin> > &_plugins)
{
  // Get all plugins 
  for (TiXmlElement* pluginXml = _parentXml->FirstChildElement("plugin"); 
      pluginXml; pluginXml = pluginXml->NextSiblingElement("plugin"))
  {
    boost::shared_ptr<Plugin> plugin;
    plugin.reset(new Plugin);

    if (initXml(pluginXml, plugin))
    {
      if (_plugins.find(plugin->name.GetValue()) != _plugins.end())
      {
        gzerr << "plugin '" << plugin->name.GetValue() << "' is not unique.\n";
        plugin.reset();
        return false;
      }
      else
      {
        _plugins.insert(make_pair(plugin->name.GetValue(),plugin));
      }
    }
    else
    {
      gzerr << "plugin xml is not initialized correctly\n";
      plugin.reset();
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<SensorType> &_sensorType)
{
  if (_sensorType->type == SensorType::CAMERA)
  {
    boost::shared_ptr<Camera> cameraSensor = boost::shared_static_cast<Camera>( _sensorType);
    return initXml(_config, cameraSensor);
  }
  else if (_sensorType->type == SensorType::RAY)
  {
    boost::shared_ptr<Ray> raySensor(boost::shared_static_cast<Ray>( _sensorType));
    return initXml(_config, raySensor);
  }
  else if (_sensorType->type == SensorType::CONTACT)
  {
    boost::shared_ptr<Contact> contactSensor(boost::shared_static_cast<Contact>( _sensorType));
    return initXml(_config, contactSensor);
  }
  else
  {
    gzerr << "SensorType init unknown type\n";
    return false;
  }

}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Sensor> &_sensor)
{
  _sensor->Clear();

  /// Get all the plugins
  getPlugins(_config, _sensor->plugins);

  if (!_sensor->name.Set(_config->Attribute("name")))
  {
     gzerr << "Unable to parse sensor name.\n";
    return false;
  }
     
  if (!_sensor->type.Set(_config->Attribute("type")))
  {
    gzerr << "Unable to parse sensor type\n";
    return false;
  }

  if (_sensor->type.GetValue() == "camera")
  {
    _sensor->sensorType.reset(new Camera);
    initXml(_config, _sensor->sensorType);
  }
  else if (_sensor->type.GetValue() == "ray")
  {
    _sensor->sensorType.reset(new Ray);
    initXml(_config, _sensor->sensorType);
  }
  else if (_sensor->type.GetValue() == "contact")
  {
    _sensor->sensorType.reset(new Contact);
    initXml(_config, _sensor->sensorType);
  }
  else
  {
    gzerr << "Unknown sensor type[" << _sensor->type << "]\n";
    return false;
  }

  if (!_sensor->alwaysOn.Set(_config->Attribute("always_on")))
  {
    gzerr << "Unable to parse sensor always_on\n";
    return false;
  }

  if (!_sensor->updateRate.Set( _config->Attribute("update_rate") ))
  {
    gzerr << "Unable to parse sensor update_rate\n";
    return false;
  }

  // Origin
  TiXmlElement *o = _config->FirstChildElement("origin");
  if (!o)
  {
    gzwarn << "Origin tag not present for sensor element, using default (Identity)\n";
    _sensor->origin.Reset();
  }
  else
  {
    if (!_sensor->origin.Set( o->Attribute("xyz"), o->Attribute("rpy") ))
    {
      gzerr << "Sensor has malformed orgin\n";
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
  if(!hfov)
  {
    gzerr << "Camera missing horizontal_fov element\n";
    return false;
  }

  if (!_sensor->horizontalFov.Set(hfov->Attribute("angle")))
  {
    gzerr << "Unable to parse camera horizontal_fov angle attribute\n";
    return false;
  }

  // Image 
  TiXmlElement *image = _config->FirstChildElement("image");
  if (!image)
  {
    gzerr << "Camera sensor requires an image element \n";
    return false;
  }

  if (!_sensor->imageWidth.Set( image->Attribute("width")))
  {
    gzerr << "Unable to parse sensor image width\n";
    return false;
  }

  if (!_sensor->imageWidth.Set( image->Attribute("height")))
  {
    gzerr << "Unable to parse sensor image height\n";
    return false;
  }

  if (!_sensor->imageFormat.Set( image->Attribute("format")))
  {
    gzerr << "Unable to parse sensor image format\n";
    return false;
  }

  // clip 
  TiXmlElement *clip = _config->FirstChildElement("clip");
  if (!clip)
  {
    gzerr << "Camera sensor requires an clip element \n";
    return false;
  }

  if (!_sensor->clipNear.Set( clip->Attribute("near") ))
  {
    gzerr << "Unable to parse camera near clip";
    return false;
  }

  if (!_sensor->clipFar.Set( clip->Attribute("far") ))
  {
    gzerr << "Unable to parse camera far clip";
    return false;
  }

  // save 
  TiXmlElement *save = _config->FirstChildElement("save");
  if (save)
  {
    if (!_sensor->saveEnabled.Set(save->Attribute("enabled")))
    {
      gzerr << "Unable to parse camera save enabled flag\n";
      return false;
    }

    if (!_sensor->savePath.Set(save->Attribute("path")))
    {
      gzerr << "Unable to parse camera save path\n";
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
    if (!_sensor->display.Set(scan->Attribute("display")))
    {
      gzerr << "Unable to parse ray display flag\n";
      return false;
    }

    // Horizontal scans
    TiXmlElement *horizontal = scan->FirstChildElement("horizontal");
    if (!horizontal)
    {
      gzerr << "missing horizontal element";
      return false;
    }

    if (!_sensor->horizontalSamples.Set( horizontal->Attribute("samples")))
    {
      gzerr << "Unable to parse ray sensor horizontal samples";
      return false;
    }

    if (!_sensor->horizontalResolution.Set(horizontal->Attribute("resolution")))
    {
      gzerr << "Unable to parse ray sensor horizontal resolution";
      return false;
    }

    if (!_sensor->horizontalMinAngle.Set(horizontal->Attribute("min_angle")))
    {
      gzerr << "Unable to parse ray sensor horizontal min_angle";
      return false;
    }

    if (!_sensor->horizontalMaxAngle.Set(horizontal->Attribute("max_angle")))
    {
      gzerr << "Unable to parse ray sensor horizontal max_angle";
      return false;
    }

    // Vertical scans
    TiXmlElement *vertical = _config->FirstChildElement("vertical");
    if (!vertical)
    {
      gzerr << "missing vertical element\n";
      return false;
    }

    if (!_sensor->verticalSamples.Set( vertical->Attribute("samples")))
    {
      gzerr << "Unable to parse ray sensor vertical samples";
      return false;
    }

    if (!_sensor->verticalResolution.Set(vertical->Attribute("resolution")))
    {
      gzerr << "Unable to parse ray sensor vertical resolution";
      return false;
    }

    if (!_sensor->verticalMinAngle.Set(vertical->Attribute("min_angle")))
    {
      gzerr << "Unable to parse ray sensor vertical min_angle";
      return false;
    }

    if (!_sensor->verticalMaxAngle.Set(vertical->Attribute("max_angle")))
    {
      gzerr << "Unable to parse ray sensor vertical max_angle";
      return false;
    }


    // range 
    TiXmlElement *range = _config->FirstChildElement("range");
    if (!range)
    {
      gzerr << "ray sensor missing range element\n";
      return false;
    }

    if (!_sensor->rangeMin.Set( range->Attribute("min")))
    {
      gzerr << "Invalid min range\n";
      return false;
    }

    if (!_sensor->rangeMax.Set( range->Attribute("max")))
    {
      gzerr << "Invalid max range\n";
      return false;
    }

    if (!_sensor->rangeResolution.Set( range->Attribute("resolution")))
    {
      gzerr << "Invalid range resolution\n";
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Material> &_material)
{
  bool hasRGB = false;

  _material->Clear();

  if (!_material->script.Set( _config->Attribute("script") ))
  {
    gzerr << "Unable to parse material script attribute\n";
    return false;
  }

  // color
  TiXmlElement *c = _config->FirstChildElement("color");
  if (c)
  {
    if (c->Attribute("rgba"))
    {
      if (!_material->color.Set(c->Attribute("rgba")))
      {
        gzerr << "Unable to parse material color rgba attribute.\n";
        _material->color.Reset();
        return false;
      }
      else
        hasRGB = true;
    }
    else
    {
      gzwarn << "Material color has no rgba\n";
    }
  }

  return !_material->script.GetValue().empty() || hasRGB;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Inertial> &_inertial)
{
  _inertial->Clear();

  // Origin
  TiXmlElement *o = _config->FirstChildElement("origin");
  if (!o)
  {
    gzwarn << "INFO: Origin tag not present for inertial element, using default (Identity)\n";
    _inertial->origin.Reset();
  }
  else
  {
    if (!_inertial->origin.Set(o->Attribute("xyz"), o->Attribute("rpy")))
    {
      gzerr << "Unable to parse origin tag\n";
      _inertial->origin.Reset();
      return false;
    }
  }

  if (!_config->Attribute("mass"))
  {
    gzerr << "Inertial element must have mass attribute.";
    return false;
  }

  if (!_inertial->mass.Set(_config->Attribute("mass")))
  {
    gzerr << "Unable to parse inerital mass attribute\n";
    return false;
  }

  TiXmlElement *inertiaXml = _config->FirstChildElement("inertia");
  if (!inertiaXml)
  {
    gzerr << "Inertial element must have inertia element\n";
    return false;
  }

  if (!(inertiaXml->Attribute("ixx") && inertiaXml->Attribute("ixy") && 
        inertiaXml->Attribute("ixz") && inertiaXml->Attribute("iyy") && 
        inertiaXml->Attribute("iyz") && inertiaXml->Attribute("izz")))
  {
    gzerr << "Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes\n";
    return false;
  }

  if (!_inertial->ixx.Set(inertiaXml->Attribute("ixx")) || 
      !_inertial->ixy.Set(inertiaXml->Attribute("ixy")) || 
      !_inertial->ixz.Set(inertiaXml->Attribute("ixz")) || 
      !_inertial->iyy.Set(inertiaXml->Attribute("iyy")) || 
      !_inertial->iyz.Set(inertiaXml->Attribute("iyz")) || 
      !_inertial->izz.Set(inertiaXml->Attribute("izz")) )
  {
    gzerr << "one of the inertia elements: "
      << "ixx (" << inertiaXml->Attribute("ixx") << ") "
      << "ixy (" << inertiaXml->Attribute("ixy") << ") "
      << "ixz (" << inertiaXml->Attribute("ixz")  << ") "
      << "iyy (" << inertiaXml->Attribute("iyy")  << ") "
      << "iyz (" << inertiaXml->Attribute("iyz")  << ") "
      << "izz (" << inertiaXml->Attribute("izz")  << ") "
      << "is not a valid double.\n";
    return false;
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Collision> &_collision)
{
  _collision->Clear();

  if (!_collision->name.Set(_config->Attribute("name")))
  {
    gzerr << "Unable to parse collision name\n";
    return false;
  }

  // Origin
  TiXmlElement *o = _config->FirstChildElement("origin");
  if (!o)
  {
    gzwarn << "Origin tag not present for collision element, using default (Identity\n";
    _collision->origin.Reset();
  }
  else if (!_collision->origin.Set(o->Attribute("xyz"), o->Attribute("rpy")))
  {
    gzerr << "Unable to parse collision origin element\n";
    _collision->origin.Reset();
    return false;
  }

  // Geometry
  TiXmlElement *geom = _config->FirstChildElement("geometry");
  if (geom)
  {
    _collision->geometry.reset( new Geometry);
    if (!initXml(geom, _collision->geometry))
    {
      gzerr << "Unable to parse collision geometry element\n";
      return false;
    }
  }
  else
  {
    gzerr << "Collision is missing a geometry element.\n";
    return false;
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Sphere> &_sphere)
{
  _sphere->Clear();

  if (!_sphere->radius.Set(_config->Attribute("radius")))
  {
    gzerr << "Unable to parse sphere's radius attribute\n";
    return false;
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Box> &_box)
{
  _box->Clear();

  if (!_box->size.Set(_config->Attribute("size")))
  {
    gzerr << "Unable to parse sphere's size attribute\n";
    return false;
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Cylinder> &_cylinder)
{
  _cylinder->Clear();

  if (!_cylinder->length.Set(_config->Attribute("length")))
  {
    gzerr << "Unable to parse cylinder's length attribute\n";
    return false;
  }

  if (!_cylinder->radius.Set(_config->Attribute("radius")))
  {
    gzerr << "Unable to parse cylinder's radius attribute\n";
    return false;
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Mesh> &_mesh)
{
  _mesh->Clear();

  if (!_mesh->filename.Set(_config->Attribute("filename")))
  {
    gzerr << "Unable to parse mesh's filename attribute\n";
    return false;
  }

  if (!_mesh->scale.Set(_config->Attribute("scale")))
  {
    gzerr << "Unable to parse mesh's scale attribute\n";
    return false;
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Link> &_link)
{
  _link->Clear();

  if (!_link->name.Set(_config->Attribute("name")))
  {
    gzerr << "Unable to parse value for link name.\n";
    return false;
  }

  // Inertial (optional)
  TiXmlElement *i = _config->FirstChildElement("inertial");
  if (i)
  {
    _link->inertial.reset(new Inertial);
    if (!initXml(i,_link->inertial))
    {
      gzerr << "Could not parse inertial element for Link '" 
        << _link->name << "'\n";
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
    }
    else
    {
      gzerr << "Could not parse visual element for Link '" 
        << _link->name << "'\n";
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
    }
    else
    {
      gzerr << "Could not parse collision element for Link '" 
        <<  _link->name << "'\n";
      col.reset();
      return false;
    }
  }

  // Get all sensor elements
  for (TiXmlElement* sensorXml = _config->FirstChildElement("sensor"); 
      sensorXml; sensorXml = sensorXml->NextSiblingElement("sensor"))
  {
    boost::shared_ptr<Sensor> sensor;
    sensor.reset(new Sensor);

    if (initXml(sensorXml,sensor))
    {
      if (_link->GetSensor(sensor->name.GetValue()))
      {
        gzerr << "sensor '" << sensor->name << "' is not unique.\n";
        sensor.reset();
        return false;
      }
      else
      {
        _link->sensors.insert(make_pair(sensor->name.GetValue(),sensor));
      }
    }
    else
    {
      gzerr << "sensor xml is not initialized correctly\n";
      sensor.reset();
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Visual> &_visual)
{
  _visual->Clear();

  if (!_visual->name.Set( _config->Attribute("name")))
  {
    gzerr << "Unable to parse visual name attribute\n";
    return false;
  }

  // Origin
  TiXmlElement *o = _config->FirstChildElement("origin");
  if (!o)
  {
    gzwarn << "Origin tag not present for visual element, using default (Identity)\n";
    _visual->origin.Reset();
  }
  else if (!_visual->origin.Set(o->Attribute("xyz"),o->Attribute("rpy")))
  {
    gzerr << "Unable to parase visual origin element\n";
    _visual->origin.Reset();
    return false;
  }

  // Geometry
  TiXmlElement *geometryXml = _config->FirstChildElement("geometry");
  if (geometryXml)
  {
    _visual->geometry.reset(new Geometry);
    if (!initXml(geometryXml, _visual->geometry))
    {
      gzerr << "Unable to parse geometry for Visual element\n";
      return false;
    }
  }
  else
  {
    gzerr << "Visual is missing geometry element\n";
    return false;
  }


  // Material
  TiXmlElement *mat = _config->FirstChildElement("material");
  if (mat)
  {
    // try to parse material element in place
    _visual->material.reset(new Material);
    if (!initXml(mat, _visual->material))
    {
      gzerr << "Could not parse material element in Visual block, maybe defined outside.\n";
      _visual->material.reset();
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<JointDynamics> &_jointDynamics)
{
  _jointDynamics->Clear();

  if (!_jointDynamics->damping.Set(_config->Attribute("damping")))
  {
    gzerr << "Unable to parse joint dynamics damping attribute\n";
    return false;
  }

  if (!_jointDynamics->friction.Set(_config->Attribute("friction")))
  {
    gzerr << "Unable to parse joint dynamics friction attribute\n";
    return false;
  }

  return true;
}


bool initXml(TiXmlElement *_config, 
             boost::shared_ptr<JointLimits> &_jointLimits)
{
  _jointLimits->Clear();

  if (!_jointLimits->lower.Set(_config->Attribute("lower")))
  {
    gzerr << "Unable to parse joint limit lower\n";
    return false;
  }

  if (!_jointLimits->upper.Set(_config->Attribute("upper")))
  {
    gzerr << "Unable to parse joint limit upper\n";
    return false;
  }

  if (!_jointLimits->effort.Set(_config->Attribute("effort")))
  {
    gzerr << "Unable to parse joint limit effort\n";
    return false;
  }

  if (!_jointLimits->effort.Set(_config->Attribute("velocity")))
  {
    gzerr << "Unable to parse joint limit velocity\n";
    return false;
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Joint> &_joint)
{
  _joint->Clear();

  // Get Joint Name
  if (!_joint->name.Set(_config->Attribute("name")))
  {
    gzerr << "Unable to parse joint name attribute\n";
    return false;
  }

  // Get transform from Parent Link to Joint Frame
  TiXmlElement *originXml = _config->FirstChildElement("origin");
  if (!originXml)
  {
    _joint->origin.Reset();
  }
  else
  {
    if (!_joint->origin.Set(originXml->Attribute("xyz"),
                            originXml->Attribute("rpy")))
    {
      gzerr << "Unable to parse joint origin element\n";
      _joint->origin.Reset();
      return false;
    }
  }

  // Get Parent Link
  TiXmlElement *parentXml = _config->FirstChildElement("parent");
  if (parentXml)
  {
    if (!_joint->parentLinkName.Set(parentXml->Attribute("link")))
    {
      gzerr << "Unable to parse joint parent link name\n";
      return false;
    }
  }
  else
  {
    gzerr << "No parent link specified for joint[" << _joint->name << "]\n";
    return false;
  }

  // Get Child Link
  TiXmlElement *childXml = _config->FirstChildElement("child");
  if (childXml)
  {
    if (!_joint->childLinkName.Set(childXml->Attribute("link")))
    {
      gzerr << "Unable to parse joint child link name\n";
      return false;
    }
  }
  else
  {
    gzerr << "No child link specified for joint[" << _joint->name << "]\n";
    return false;
  }



  // Get Joint type
  const char* typeChar = _config->Attribute("type");
  if (!typeChar)
  {
    gzerr << "joint '" << _joint->name 
      << "' has no type, check to see if it's a reference.\n";
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
    gzerr << "Joint '" << _joint->name  << "' has no known type '" 
      << typeStr << "'.\n";
    return false;
  }

  // Get Joint Axis
  if (_joint->type != Joint::BALL)
  {
    // axis
    TiXmlElement *axisXml = _config->FirstChildElement("axis");
    if (!axisXml)
    {
      gzwarn << "no axis elemement for Joint link '" 
        << _joint->name << "', defaulting to (1,0,0) axis\n";
      _joint->axis.SetValue( Vector3(1.0, 0.0, 0.0));
    }
    else{
      if (!_joint->axis.Set( axisXml->Attribute("xyz")) )
      {
        gzerr << "Unable to parse axis for joint[" << _joint->name << "]\n";
        return false;
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
      gzerr << "Could not parse limit element for joint '" 
        << _joint->name << "'.\n";
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
      gzerr << "Could not parse joint_dynamics element for joint '" 
        << _joint->name << "'.\n";
      _joint->dynamics.reset();
      return false;
    }
  }

  return true;
}




////////////////////////////////////////////////////////////////////////////////
bool initFile(const std::string &_filename, boost::shared_ptr<Model> &_model)
{
  TiXmlDocument xmlDoc;
  xmlDoc.LoadFile(_filename);

  return initDoc(&xmlDoc,_model);
}


////////////////////////////////////////////////////////////////////////////////
bool initString(const std::string &_xmlString, boost::shared_ptr<Model> &_model)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());

  return initDoc(&xmlDoc, _model);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Load Model from TiXMLDocument
bool initDoc(TiXmlDocument *_xmlDoc, boost::shared_ptr<Model> &_model)
{
  if (!_xmlDoc)
  {
    gzerr << "Could not parse the xml\n";
    return false;
  }

  TiXmlElement *modelXml = _xmlDoc->FirstChildElement("model");
  if (!modelXml)
  {
    gzerr << "Could not find the 'model' element in the xml file\n";
    return false;
  }

  return initXml(modelXml, _model);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Load Model from TiXMLElement
bool initXml(TiXmlElement *_xml, boost::shared_ptr<Model> &_model)
{
  _model->Clear();

  if (!_xml) 
    return false;

  if (!_model->name.Set(_xml->Attribute("name")))
  {
    gzerr << "No name given for the model.\n";
    return false;
  }

  // Get all Link elements
  for (TiXmlElement* linkXml = _xml->FirstChildElement("link"); 
      linkXml; linkXml = linkXml->NextSiblingElement("link"))
  {
    boost::shared_ptr<Link> link;
    link.reset(new Link);

    if (initXml(linkXml,link))
    {
      if (_model->GetLink(link->name.GetValue()))
      {
        gzerr << "link '" << link->name << "' is not unique.\n";
        link.reset();
        return false;
      }
      else
      {
        _model->links.insert(make_pair(link->name.GetValue(),link));
      }
    }
    else
    {
      gzerr << "link xml is not initialized correctly\n";
      link.reset();
      return false;
    }
  }

  if (_model->links.empty())
  {
    gzerr << "No link elements found in xml file\n";
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
      if (_model->GetJoint(joint->name.GetValue()))
      {
        gzerr << "joint '" << joint->name << "' is not unique.\n";
        joint.reset();
        return false;
      }
      else
      {
        _model->joints.insert(make_pair(joint->name.GetValue(),joint));
      }
    }
    else
    {
      gzerr << "joint xml is not initialized correctly\n";
      joint.reset();
      return false;
    }
  }

  /// Get all the plugins
  getPlugins(_xml, _model->plugins);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool initXml(TiXmlElement *_config, boost::shared_ptr<Geometry> &_geometry)
{
  bool result = false;
  TiXmlElement *shape = NULL;

  if (!_config)
  {
    gzerr << "Null xml\n";
    return false;
  }

  shape = _config->FirstChildElement();
  if (!shape)
  {
    gzerr << "Geometry tag contains no child element.\n";
    return false;
  }

  std::string typeName = shape->ValueStr();
  if (typeName == "sphere")
  {
    boost::shared_ptr<Sphere> sphere(new Sphere);
    result = initXml(shape, sphere);
    _geometry = sphere;
  }
  else if (typeName == "box")
  {
    boost::shared_ptr<Box> box(new Box);
    result = initXml(shape, box);
    _geometry =  box;
  }
  else if (typeName == "cylinder")
  {
    boost::shared_ptr<Cylinder> cylinder(new Cylinder);
    result = initXml(shape, cylinder);
    _geometry = cylinder;
  }
  else if (typeName == "mesh")
  {
    boost::shared_ptr<Mesh> mesh(new Mesh);
    result = initXml(shape, mesh);
    _geometry = mesh;
  }
  else
  {
    gzerr << "Unknown geometry type '" << typeName << "'.\n";
    return false;
  }

  // clear geom object when fails to initialize
  if (!result)
  {
    gzerr << "Geometry failed to parse\n";
    _geometry.reset();
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Plugin> &_plugin)
{
  _plugin->Clear();

  if (!_plugin->name.Set(_config->Attribute("name")))
  {
    gzerr << "Unable to parse the plugin name.\n";
    return false;
  }

  if (!_plugin->filename.Set(_config->Attribute("filename")))
  {
    gzerr << "Unable to parse plugin filename.\n";
    return false;
  }

  return true;
}


////////////////////////////////////////////////////////////////////////////////
bool initFile(const std::string &_filename, boost::shared_ptr<World> &_world)
{
  TiXmlDocument xmlDoc;
  xmlDoc.LoadFile(_filename);

  return initDoc(&xmlDoc, _world);
}

////////////////////////////////////////////////////////////////////////////////
bool initString(const std::string &_xmlString, boost::shared_ptr<World> &_world)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());

  return initDoc(&xmlDoc,_world);
}

////////////////////////////////////////////////////////////////////////////////
bool initDoc(TiXmlDocument *_xmlDoc, boost::shared_ptr<World> &_world)
{
  if (!_xmlDoc)
  {
    gzerr << "Could not parse the xml\n";
    return false;
  }

  TiXmlElement *worldXml = _xmlDoc->FirstChildElement("world");
  if (!worldXml)
  {
    gzerr << "Could not find the 'world' element in the xml file\n";
    return false;
  }

  return initXml(worldXml, _world);
}

////////////////////////////////////////////////////////////////////////////////
bool initXml(TiXmlElement *_worldXml, boost::shared_ptr<World> &_world)
{
  _world->Clear();

  if (!_worldXml) 
  {
    gzerr << "Error: World XML is NULL\n";
    return false;
  }

  // Get world name
  const char *nameStr = _worldXml->Attribute("name");
  if (!nameStr)
  {
    gzerr << "No name given for the world->\n";
    return false;
  }

  if (!_world->name.Set(std::string(nameStr)))
  {
    gzerr << "Unable to parse world name\n";
    return false;
  }

  _world->scene.reset(new Scene);
  initXml(_worldXml->FirstChildElement("scene"),_world->scene);

  _world->physics.reset(new Physics);
  initXml(_worldXml->FirstChildElement("physics"),_world->physics);

  // Get all model elements
  for (TiXmlElement* modelXml = _worldXml->FirstChildElement("model"); 
      modelXml; modelXml = modelXml->NextSiblingElement("model"))
  {
    boost::shared_ptr<Model> model;
    model.reset(new Model);

    if (initXml(modelXml,model))
    {
      if (_world->GetModel(model->name.GetValue()))
      {
        gzerr << "model '" << model->name << "' is not unique.\n";
        model.reset();
        return false;
      }
      else
      {
        _world->models.insert(make_pair(model->name.GetValue(),model));
      }
    }
    else
    {
      gzerr << "model xml is not initialized correctly\n";
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
      if (_world->GetJoint(joint->name.GetValue()))
      {
        gzerr << "joint '" << joint->name << "s' is not unique.\n";
        joint.reset();
        return false;
      }
      else
      {
        _world->joints.insert(make_pair(joint->name.GetValue(),joint));
      }
    }
    else
    {
      gzerr << "joint xml is not initialized correctly\n";
      joint.reset();
      return false;
    }
  }

  /// Get all the plugins
  getPlugins(_worldXml, _world->plugins);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool initXml(TiXmlElement* _config, boost::shared_ptr<Scene> &_scene)
{
  _scene->Clear();

  TiXmlElement *ambient = _config->FirstChildElement("ambient");
  if (ambient)
  {
    if (!_scene->ambientColor.Set(ambient->Attribute("rgba")))
    {
      gzerr << "Unable to parse ambient color\n";
      return false;
    }
  }

  TiXmlElement *background = _config->FirstChildElement("background");
  if (background)
  {
    if (!_scene->backgroundColor.Set(background->Attribute("rgba")))
    {
      gzerr << "Unable to parse background rgba\n";
      return false;
    }


    TiXmlElement *sky = background->FirstChildElement("sky");
    if (sky)
    {
      if (!_scene->skyMaterial.Set(sky->Attribute("material")))
      {
        gzerr << "Unable to parse material for the sky.\n";
        return false;
      }
    }
  }

  TiXmlElement *shadow = _config->FirstChildElement("shadows");
  if (shadow)
  {
    if (!_scene->shadowEnabled.Set(shadow->Attribute("enabled")))
    {
      gzerr << "Shadow element requires an enabled attribute";
      return false;
    }

    if (!_scene->shadowColor.Set(shadow->Attribute("rgba")))
    {
      gzerr << "Unable to parse shadow rgba\n";
      return false;
    }


    if (!_scene->shadowType.Set(shadow->Attribute("type")))
    {
      gzerr << "Unable to parse shadow type attribute\n";
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<OpenDynamicsEngine> &_openDynamicsEngine)
{
  TiXmlElement *odeConfig = _config->FirstChildElement("ode");

  if ( !odeConfig )
  {
    gzerr << "Physics element missing <ode>\n";
    return false;
  }

  TiXmlElement *solverConfig = odeConfig->FirstChildElement("solver");
  if (!solverConfig)
  {
    gzerr << "ODE Physics missing solver element\n";
    return false;
  }

  if (!_openDynamicsEngine->solverType.Set( solverConfig->Attribute("type")))
  {
    gzerr << "ODE Physics missing solver type\n";
    return false;
  }

  if (!_openDynamicsEngine->dt.Set(solverConfig->Attribute("dt")))
  {
    gzerr << "ODE Physics solver unable to parse dt attribute\n";
    return false;
  }

  if (!_openDynamicsEngine->iters.Set(solverConfig->Attribute("iters")))
  {
    gzerr << "ODE Physics solver malformed iters attribute\n";
    return false;
  }

  if (!_openDynamicsEngine->sor.Set(solverConfig->Attribute("sor"))) 
  {
    gzerr << "ODE Physics solver unable to parse sor attribute\n)";
    return false;
  }


  // Contraints
  TiXmlElement *constraintsConfig = odeConfig->FirstChildElement("constraints");
  if (constraintsConfig)
  {

    if (!_openDynamicsEngine->cfm.Set(constraintsConfig->Attribute("cfm"))) 
    {
      gzerr << "ODE Physics contraints unable to parse cfm attribute\n";
      return false;
    }

    if (!_openDynamicsEngine->erp.Set(constraintsConfig->Attribute("erp"))) 
    {
      gzerr << "ODE Physics contraints unable to parse erp attribute\n";
      return false;
    }


    if (!_openDynamicsEngine->contactMaxCorrectingVel.Set(constraintsConfig->Attribute("contact_max_correcting_vel")))
    {
      gzerr << "ODE Physics contraints unable to parse contact_max_correcting_vel attribute\n";
      return false;
    } 

    if (!_openDynamicsEngine->contactSurfaceLayer.Set(constraintsConfig->Attribute("contact_surface_layer")))
    {
      gzerr << "ODE Physics contraints unable to parse contact_surface_layer attribute\n";
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
    gzerr << "xml config is NULL\n";
    return false;
  }

  if (!_physics->type.Set(_config->Attribute("type")))
  {
    gzerr << "Unable to parse physics type attribute\n";
    return false;
  }

  TiXmlElement *gravityElement = _config->FirstChildElement("gravity");
  if (gravityElement)
  {
    if (!_physics->gravity.Set(gravityElement->Attribute("xyz")))
    {
      gzerr << "Unable to parse physics gravity element \n";
      _physics->gravity.Reset();
      return false;
    }
  }

  if (_physics->type.GetValue() == "ode")
  {
    _physics->engine.reset(new OpenDynamicsEngine );

    boost::shared_ptr<OpenDynamicsEngine> open_dynamics_engine = boost::shared_static_cast<OpenDynamicsEngine>(_physics->engine);

    initXml(_config,open_dynamics_engine);
  }
  else
  {
    gzerr << "Unknown physics engine type[" << _physics->type << "].\n";
    return false;
  }

  return true;
}

bool initXml(TiXmlElement * /*_config*/, boost::shared_ptr<Contact> &/*_contact*/)
{
  return true;
}

}
