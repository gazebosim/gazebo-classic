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
    boost::shared_ptr<CameraSensor> cameraSensor = boost::shared_static_cast<CameraSensor>( _sensorType);
    return initXml(_config, cameraSensor);
  }
  else if (_sensorType->type == SensorType::RAY)
  {
    boost::shared_ptr<RaySensor> raySensor(boost::shared_static_cast<RaySensor>( _sensorType));
    return initXml(_config, raySensor);
  }
  else if (_sensorType->type == SensorType::CONTACT)
  {
    boost::shared_ptr<ContactSensor> contactSensor(boost::shared_static_cast<ContactSensor>( _sensorType));
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
    _sensor->sensorType.reset(new CameraSensor);
    initXml(_config, _sensor->sensorType);
  }
  else if (_sensor->type.GetValue() == "ray")
  {
    _sensor->sensorType.reset(new RaySensor);
    initXml(_config, _sensor->sensorType);
  }
  else if (_sensor->type.GetValue() == "contact")
  {
    _sensor->sensorType.reset(new ContactSensor);
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
    if (!_sensor->origin.Set( o->Attribute("pose") ))
    {
      gzerr << "Sensor has malformed orgin\n";
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<CameraSensor> &_sensor)
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

  if (!_sensor->imageHeight.Set( image->Attribute("height")))
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

bool initXml(TiXmlElement *_config, boost::shared_ptr<RaySensor> &_sensor)
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
    if (!_inertial->origin.Set(o->Attribute("pose")))
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
  else if (!_collision->origin.Set(o->Attribute("pose")))
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

  // Surface
  TiXmlElement *surface = _config->FirstChildElement("surface");
  if (surface)
  {
    _collision->surface.reset(new Surface);
    if (!initXml(surface, _collision->surface))
    {
      gzerr << "Unable to parse collision surface element\n";
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Surface> &_surface)
{
  _surface->Clear();
  TiXmlElement *friction = _config->FirstChildElement("friction");
  if (friction)
  {
    _surface->frictions.clear();

    for (TiXmlElement* frictionXml = friction->FirstChildElement(); 
        frictionXml; frictionXml = frictionXml->NextSiblingElement())
    {
      if (std::string(frictionXml->Value()) == "ode")
      {
        boost::shared_ptr<ODEFriction> odeFriction( new ODEFriction );
        if (!initXml(frictionXml, odeFriction))
        {
          gzerr << "Unable to parse ODE Friction element\n";
          return false;
        }
        _surface->frictions.push_back( odeFriction );
      }

    }
  }

  TiXmlElement *bounce = _config->FirstChildElement("bounce");
  if (bounce)
  {
    if (!_surface->bounceRestCoeff.Set( bounce->Attribute("restitution_coefficient") ))
    {
      gzerr << "Unable to set surface bouce restitution_coefficient\n";
      return false;
    }

    if (!_surface->bounceThreshold.Set( bounce->Attribute("threshold") ))
    {
      gzerr << "Unable to set surface bouce threshold\n";
      return false;
    }
  }

  TiXmlElement *contact = _config->FirstChildElement("contact");
  if (contact)
  {
    _surface->contacts.clear();

    for (TiXmlElement* contactXml = contact->FirstChildElement(); 
        contactXml; contactXml = contactXml->NextSiblingElement())
    {
      if (std::string(contactXml->Value()) == "ode")
      {
        boost::shared_ptr<ODEContact> odeContact( new ODEContact );
        if (!initXml(contactXml, odeContact))
        {
          gzerr << "Unable to parse ODE Contact element\n";
          return false;
        }
        _surface->contacts.push_back( odeContact );
      }
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<ODEFriction> &_friction)
{
  if (!_friction->mu.Set( _config->Attribute("mu") ))
  {
    gzerr << "Unable to parse ODE friction mu element\n";
    return false;
  }

  if (!_friction->mu2.Set( _config->Attribute("mu2") ))
  {
    gzerr << "Unable to parse ODE friction mu2 element\n";
    return false;
  }

  if (!_friction->fdir1.Set( _config->Attribute("fdir1") ))
  {
    gzerr << "Unable to parse ODE friction fdir1 element\n";
    return false;
  }

  if (!_friction->slip1.Set( _config->Attribute("slip1") ))
  {
    gzerr << "Unable to parse ODE friction slip1 element\n";
    return false;
  }

  if (!_friction->slip2.Set( _config->Attribute("slip2") ))
  {
    gzerr << "Unable to parse ODE friction slip2 element\n";
    return false;
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<ODEContact> &_contact)
{
  if (!_contact->softCFM.Set( _config->Attribute("soft_cfm") ))
  {
    gzerr << "Unable to parse ODE contact soft_cfm element\n";
    return false;
  }

  if (!_contact->kp.Set( _config->Attribute("kp") ))
  {
    gzerr << "Unable to parse ODE contact kp element\n";
    return false;
  }

  if (!_contact->kd.Set( _config->Attribute("kd") ))
  {
    gzerr << "Unable to parse ODE contact kd element\n";
    return false;
  }

  if (!_contact->maxVel.Set( _config->Attribute("max_vel") ))
  {
    gzerr << "Unable to parse ODE contact max_vel element\n";
    return false;
  }

  if (!_contact->minDepth.Set( _config->Attribute("min_depth") ))
  {
    gzerr << "Unable to parse ODE contact min_depth element\n";
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

  // Origin
  TiXmlElement *o = _config->FirstChildElement("origin");
  if (!o)
  {
    gzwarn << "Origin tag not present for link element, using default (Identity)\n";
    _link->origin.Reset();
  }
  else if (!_link->origin.Set(o->Attribute("pose")))
  {
    gzerr << "Unable to parase link origin element\n";
    _link->origin.Reset();
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

  if (!_visual->castShadows.Set( _config->Attribute("cast_shadows")))
  {
    gzerr << "Unable to parse visual cast_shadows attribute\n";
    return false;
  }

  // Origin
  TiXmlElement *o = _config->FirstChildElement("origin");
  if (!o)
  {
    gzwarn << "Origin tag not present for visual element, using default (Identity)\n";
    _visual->origin.Reset();
  }
  else if (!_visual->origin.Set(o->Attribute("pose")))
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
    if (!_joint->origin.Set(originXml->Attribute("pose")))
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
  if (!_joint->type.Set(_config->Attribute("type")))
  {
    gzerr << "joint '" << _joint->name 
      << "' has no type, check to see if it's a reference.\n";
    return false;
  }

  std::string typeStr = _joint->type.GetValue();
  if (typeStr == "piston")
    _joint->typeEnum = Joint::PISTON;
  else if (typeStr == "revolute2")
    _joint->typeEnum = Joint::REVOLUTE2;
  else if (typeStr == "revolute")
    _joint->typeEnum = Joint::REVOLUTE;
  else if (typeStr == "universal")
    _joint->typeEnum = Joint::UNIVERSAL;
  else if (typeStr == "prismatic")
    _joint->typeEnum = Joint::PRISMATIC;
  else if (typeStr == "ball")
    _joint->typeEnum = Joint::BALL;
  else
  {
    gzerr << "Joint '" << _joint->name  << "' has no known type '" 
      << typeStr << "'.\n";
    return false;
  }

  // Get Joint Axis
  if (_joint->typeEnum != Joint::BALL)
  {
    // axis
    TiXmlElement *axisXml = _config->FirstChildElement("axis");
    if (!axisXml)
    {
      gzwarn << "no axis elemement for Joint link '" 
        << _joint->name << "', defaulting to (1,0,0) axis\n";
      _joint->axis.SetValue( gazebo::math::Vector3(1.0, 0.0, 0.0));
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
  if (!_world->name.Set(_worldXml->Attribute("name")))
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

bool initXml(TiXmlElement * /*_config*/, 
             boost::shared_ptr<ContactSensor> &/*_contact*/)
{
  return true;
}

bool saveXml(const std::string &filename, const boost::shared_ptr<World> &_world)
{
  TiXmlDocument doc;

  TiXmlDeclaration *decl = new TiXmlDeclaration( "1.0", "", "" );
  doc.LinkEndChild(decl);

  TiXmlElement *root = new TiXmlElement("gazebo");
  doc.LinkEndChild(root);
  root->SetAttribute("version","1.0");

  TiXmlElement *worldElement = new TiXmlElement("world");
  root->LinkEndChild(worldElement);

  worldElement->SetAttribute(_world->name.GetKey(), 
                             _world->name.GetAsString());

  saveXml(worldElement, _world->scene); 
  saveXml(worldElement, _world->physics); 

  // Save the models
  std::map<std::string, boost::shared_ptr<Model> >::const_iterator miter;
  for (miter = _world->models.begin(); miter != _world->models.end(); miter++)
  {
    saveXml(worldElement, miter->second); 
  }

  // Save the joints
  std::map<std::string, boost::shared_ptr<Joint> >::const_iterator jiter;
  for (jiter = _world->joints.begin(); jiter != _world->joints.end(); jiter++)
  {
    saveXml(worldElement, jiter->second); 
  }

  // Save the plugins
  std::map<std::string, boost::shared_ptr<Plugin> >::const_iterator piter;
  for (piter = _world->plugins.begin(); piter != _world->plugins.end(); piter++)
  {
    saveXml(worldElement, piter->second); 
  }

  TiXmlPrinter printer;
  printer.SetIndent("  ");

  doc.Accept(&printer);

  std::fstream out(filename.c_str(), std::ios::out);
  out << printer.CStr();
  out.close();

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Scene> &_scene)
{
  TiXmlElement *sceneNode = new TiXmlElement("scene");
  _parent->LinkEndChild( sceneNode );

  TiXmlElement *ambientNode = new TiXmlElement("ambient");
  sceneNode->LinkEndChild( ambientNode );
  ambientNode->SetAttribute( _scene->ambientColor.GetKey(),
                             _scene->ambientColor.GetAsString() );

  TiXmlElement *backgroundNode = new TiXmlElement("background");
  sceneNode->LinkEndChild( backgroundNode );
  backgroundNode->SetAttribute( _scene->backgroundColor.GetKey(),
                                _scene->backgroundColor.GetAsString() );


  TiXmlElement *skyNode = new TiXmlElement("sky");
  backgroundNode->LinkEndChild( skyNode );
  skyNode->SetAttribute( _scene->skyMaterial.GetKey(),
                         _scene->skyMaterial.GetAsString());

  // Save shadows
  TiXmlElement *shadowNode = new TiXmlElement("shadows");
  sceneNode->LinkEndChild( shadowNode );
  shadowNode->SetAttribute( _scene->shadowEnabled.GetKey(),
                            _scene->shadowEnabled.GetAsString());
  shadowNode->SetAttribute( _scene->shadowColor.GetKey(),
                            _scene->shadowColor.GetAsString());
  shadowNode->SetAttribute( _scene->shadowType.GetKey(),
                            _scene->shadowType.GetAsString());

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Physics> &_physics)
{
  TiXmlElement *physicsNode = new TiXmlElement("physics");
  _parent->LinkEndChild( physicsNode );

  physicsNode->SetAttribute( _physics->type.GetKey(), 
                             _physics->type.GetAsString() );

  TiXmlElement *gravityNode = new TiXmlElement("gravity");
  physicsNode->LinkEndChild( gravityNode );
  gravityNode->SetAttribute( _physics->gravity.GetKey(),
                             _physics->gravity.GetAsString() );

  if (_physics->type.GetAsString() == "ode")
  {
    boost::shared_ptr<OpenDynamicsEngine> openDynamicsEngine = boost::shared_static_cast<OpenDynamicsEngine>(_physics->engine);
    saveXml( physicsNode, openDynamicsEngine );
  }

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<OpenDynamicsEngine> &_engine)
{
  TiXmlElement *odeNode = new TiXmlElement("ode");
  _parent->LinkEndChild( odeNode );

  TiXmlElement *solverNode = new TiXmlElement("solver");
  odeNode->LinkEndChild( solverNode );

  solverNode->SetAttribute( _engine->solverType.GetKey(),
                            _engine->solverType.GetAsString() );
  solverNode->SetAttribute( _engine->dt.GetKey(),
                            _engine->dt.GetAsString() );
  solverNode->SetAttribute( _engine->iters.GetKey(),
                            _engine->iters.GetAsString() );
  solverNode->SetAttribute( _engine->sor.GetKey(),
                            _engine->sor.GetAsString() );


  TiXmlElement *constraintsNode = new TiXmlElement("constraints");
  odeNode->LinkEndChild( constraintsNode );
  constraintsNode->SetAttribute( _engine->cfm.GetKey(),
                                 _engine->cfm.GetAsString() );
  constraintsNode->SetAttribute( _engine->erp.GetKey(),
                                 _engine->erp.GetAsString() );
  constraintsNode->SetAttribute( _engine->contactMaxCorrectingVel.GetKey(),
                                 _engine->contactMaxCorrectingVel.GetAsString() );
  constraintsNode->SetAttribute( _engine->contactSurfaceLayer.GetKey(),
                                 _engine->contactSurfaceLayer.GetAsString() );

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Joint> &_joint)
{
  TiXmlElement *jointNode = new TiXmlElement("joint");
  _parent->LinkEndChild( jointNode );

  jointNode->SetAttribute(_joint->name.GetKey(),
                          _joint->name.GetAsString());
  jointNode->SetAttribute(_joint->type.GetKey(),
                          _joint->type.GetAsString());

  TiXmlElement *parentNode = new TiXmlElement("parent");
  jointNode->LinkEndChild(parentNode);
  parentNode->SetAttribute( _joint->parentLinkName.GetKey(),
                            _joint->parentLinkName.GetAsString() );

  TiXmlElement *childNode = new TiXmlElement("child");
  jointNode->LinkEndChild(childNode);
  childNode->SetAttribute( _joint->childLinkName.GetKey(),
                           _joint->childLinkName.GetAsString() );

  jointNode->SetAttribute(_joint->origin.GetKey(),
                          _joint->origin.GetAsString());

  // Axis 1
  TiXmlElement *axisNode = new TiXmlElement("axis");
  jointNode->LinkEndChild(axisNode);
  axisNode->SetAttribute( _joint->axis.GetKey(), _joint->axis.GetAsString() );

  // Axis 2
  TiXmlElement *axis2Node = new TiXmlElement("axis2");
  jointNode->LinkEndChild(axis2Node);
  axis2Node->SetAttribute( _joint->axis2.GetKey(), _joint->axis2.GetAsString());

  // Dynamics
  if (_joint->dynamics)
  {
    TiXmlElement *dynamicsNode = new TiXmlElement("dynamics");
    jointNode->LinkEndChild(dynamicsNode);
    dynamicsNode->SetAttribute( _joint->dynamics->damping.GetKey(),
        _joint->dynamics->damping.GetAsString() );
    dynamicsNode->SetAttribute( _joint->dynamics->friction.GetKey(),
        _joint->dynamics->friction.GetAsString() );
  }

  // Limit
  if (_joint->limits)
  {
    TiXmlElement *limitNode = new TiXmlElement("limit");
    jointNode->LinkEndChild(limitNode);
    limitNode->SetAttribute( _joint->limits->lower.GetKey(),
        _joint->limits->lower.GetAsString() );
    limitNode->SetAttribute( _joint->limits->upper.GetKey(),
        _joint->limits->upper.GetAsString() );
    limitNode->SetAttribute( _joint->limits->effort.GetKey(),
        _joint->limits->effort.GetAsString() );
    limitNode->SetAttribute( _joint->limits->velocity.GetKey(),
        _joint->limits->velocity.GetAsString() );
  }

  return true;
}

bool saveXml(TiXmlElement *_parent, const gazebo::math::Vector3 &_vec)
{
  std::ostringstream stream;
  stream << _vec;
  _parent->SetAttribute("xyz", stream.str());
  return true;
}

bool saveXml(TiXmlElement *_parent, const gazebo::math::Quaternion &_rot)
{
  std::ostringstream stream;
  stream << _rot;
  _parent->SetAttribute("rot", stream.str());
  return true;
}

/*
bool saveXml(TiXmlElement *_parent, const ParamT<gazebo::math::Pose,true> &_pose)
{
  TiXmlElement *originNode = new TiXmlElement(_pose.GetKey());
  _parent->LinkEndChild( originNode );

  saveXml(originNode, _pose.GetValue());

  return true;
}

bool saveXml(TiXmlElement *_parent, const ParamT<gazebo::math::Pose,false> &_pose)
{
  TiXmlElement *originNode = new TiXmlElement(_pose.GetKey());
  _parent->LinkEndChild( originNode );

  saveXml(originNode, _pose.GetValue());

  return true;
}*/

/*bool saveXml(TiXmlElement *_parent, const gazebo::math::Pose &_pose)
{
  std::ostringstream poseStream;
  poseStream << _pose;
  _parent->SetAttribute("pose", poseStream.str());
  return true;
}
*/

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Model> &_model)
{
  TiXmlElement *modelNode = new TiXmlElement("model");
  _parent->LinkEndChild( modelNode );


  modelNode->SetAttribute( _model->name.GetKey(),
                           _model->name.GetAsString() );

  // Save the links
  std::map<std::string, boost::shared_ptr<Link> >::const_iterator liter;
  for (liter = _model->links.begin(); liter != _model->links.end(); liter++)
  {
    saveXml(modelNode, liter->second);
  }

  // Save the joints
  std::map<std::string, boost::shared_ptr<Joint> >::const_iterator jiter;
  for (jiter = _model->joints.begin(); jiter != _model->joints.end(); jiter++)
  {
    saveXml(modelNode, jiter->second);
  }

  // Save the plugins
  std::map<std::string, boost::shared_ptr<Plugin> >::const_iterator piter;
  for (piter = _model->plugins.begin(); piter != _model->plugins.end(); piter++)
  {
    saveXml(modelNode, piter->second);
  }

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Link> &_link)
{
  TiXmlElement *linkNode = new TiXmlElement("link");
  _parent->LinkEndChild( linkNode );

  linkNode->SetAttribute(_link->name.GetKey(),
                         _link->name.GetAsString());
  linkNode->SetAttribute(_link->selfCollide.GetKey(),
                         _link->selfCollide.GetAsString());
  linkNode->SetAttribute(_link->gravity.GetKey(),
                         _link->gravity.GetAsString());

  // Save interial
  saveXml(linkNode, _link->inertial);

  // Save visuals
  std::vector<boost::shared_ptr<Visual> >::const_iterator viter;
  for (viter = _link->visuals.begin(); viter != _link->visuals.end(); viter++)
  {
    saveXml(linkNode, *viter);
  }

  // Save collisions
  std::vector<boost::shared_ptr<Collision> >::const_iterator citer;
  for (citer = _link->collisions.begin(); citer != _link->collisions.end(); citer++)
  {
    saveXml(linkNode, *citer);
  }

  // Save sensors
  std::map<std::string, boost::shared_ptr<Sensor> >::const_iterator siter;
  for (siter = _link->sensors.begin(); siter != _link->sensors.end(); siter++)
  {
    saveXml(linkNode, siter->second);
  }

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Plugin> &_plugin)
{
  TiXmlElement *pluginNode = new TiXmlElement("plugin");
  _parent->LinkEndChild( pluginNode );
  pluginNode->SetAttribute(_plugin->name.GetKey(),
                           _plugin->name.GetAsString());
  pluginNode->SetAttribute(_plugin->filename.GetKey(),
                           _plugin->filename.GetAsString());

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Visual> &_visual)
{
  TiXmlElement *visualNode = new TiXmlElement("visual");
  _parent->LinkEndChild( visualNode );
  visualNode->SetAttribute( _visual->castShadows.GetKey(),
                            _visual->castShadows.GetAsString());
  visualNode->SetAttribute( _visual->name.GetKey(),
                            _visual->name.GetAsString());

  visualNode->SetAttribute( _visual->origin.GetKey(),
                            _visual->origin.GetAsString() );

  saveXml(visualNode, _visual->geometry);
  saveXml(visualNode, _visual->material);

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Geometry> &_geom)
{
  TiXmlElement *shapeNode = NULL;
  TiXmlElement *geomNode = new TiXmlElement("geometry");
  _parent->LinkEndChild( geomNode );

  if (_geom->type == Geometry::SPHERE)
  {
    boost::shared_ptr<Sphere> shape = boost::shared_static_cast<Sphere>(_geom);
    shapeNode = new TiXmlElement("sphere");
    shapeNode->SetAttribute(shape->radius.GetKey(),shape->radius.GetAsString());
  }
  else if (_geom->type == Geometry::BOX)
  {
    boost::shared_ptr<Box> shape = boost::shared_static_cast<Box>(_geom);
    shapeNode = new TiXmlElement("box");
    std::ostringstream stream;
    stream << shape->size;
    shapeNode->SetAttribute("size", stream.str());
  }
  else if (_geom->type == Geometry::CYLINDER)
  {
    boost::shared_ptr<Cylinder> shape = boost::shared_static_cast<Cylinder>(_geom);
    shapeNode = new TiXmlElement("cylinder");
    shapeNode->SetAttribute(shape->radius.GetKey(),shape->radius.GetAsString());
    shapeNode->SetAttribute(shape->length.GetKey(),shape->length.GetAsString());
  }
  else if (_geom->type == Geometry::MESH)
  {
    boost::shared_ptr<Mesh> shape = boost::shared_static_cast<Mesh>(_geom);
    shapeNode = new TiXmlElement("mesh");
    shapeNode->SetAttribute(shape->filename.GetKey(),
                            shape->filename.GetAsString());
    shapeNode->SetAttribute(shape->scale.GetKey(),
                            shape->scale.GetAsString());
  }
  else
  {
    gzerr << "Unknown geometry type[" << _geom->type << "]\n";
    return false;
  }

  geomNode->LinkEndChild( shapeNode );

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Material> &_mat)
{
  TiXmlElement *matNode = new TiXmlElement("material");
  _parent->LinkEndChild( matNode );
  matNode->SetAttribute( _mat->script.GetKey(), _mat->script.GetAsString());

  TiXmlElement *colorNode = new TiXmlElement("color");
  colorNode->SetAttribute( _mat->color.GetKey(), _mat->color.GetAsString());

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Collision> &_collision)
{
  TiXmlElement *collisionNode = new TiXmlElement("collision");
  _parent->LinkEndChild( collisionNode );
  collisionNode->SetAttribute( _collision->name.GetKey(),
                               _collision->name.GetAsString());

  collisionNode->SetAttribute( _collision->origin.GetKey(),
                               _collision->origin.GetAsString());

  saveXml(collisionNode, _collision->geometry);

  if (_collision->surface)
    saveXml(collisionNode, _collision->surface);

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Sensor> &_sensor)
{
  TiXmlElement *sensorNode = new TiXmlElement("sensor");
  _parent->LinkEndChild( sensorNode );

  sensorNode->SetAttribute( _sensor->name.GetKey(),
                            _sensor->name.GetAsString() );
  sensorNode->SetAttribute( _sensor->type.GetKey(),
                            _sensor->type.GetAsString() );
  sensorNode->SetAttribute( _sensor->alwaysOn.GetKey(),
                            _sensor->alwaysOn.GetAsString() );
  sensorNode->SetAttribute( _sensor->updateRate.GetKey(),
                            _sensor->updateRate.GetAsString() );
  sensorNode->SetAttribute( _sensor->origin.GetKey(),
                            _sensor->origin.GetAsString() );

  if (_sensor->sensorType->type == SensorType::CAMERA)
  {
    boost::shared_ptr<CameraSensor> camera = boost::shared_static_cast<CameraSensor>(_sensor->sensorType);
    saveXml( sensorNode, camera);
  }
  else if (_sensor->sensorType->type == SensorType::RAY)
  {
    boost::shared_ptr<RaySensor> ray = boost::shared_static_cast<RaySensor>(_sensor->sensorType);
    saveXml( sensorNode, ray);
  }
  else if (_sensor->sensorType->type != SensorType::CONTACT)
  {
    gzerr << "Unknown sensor type[" << _sensor->type << "]\n";
    return false;
  }

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<CameraSensor> &_camera)
{
  TiXmlElement *hfovNode = new TiXmlElement("horizontal_fov");
  _parent->LinkEndChild( hfovNode );
  hfovNode->SetAttribute( _camera->horizontalFov.GetKey(),
                          _camera->horizontalFov.GetAsString() );

  TiXmlElement *imageNode = new TiXmlElement("image");
  _parent->LinkEndChild( imageNode );
  imageNode->SetAttribute( _camera->imageWidth.GetKey(),
                           _camera->imageWidth.GetAsString() );
  imageNode->SetAttribute( _camera->imageHeight.GetKey(),
                           _camera->imageHeight.GetAsString() );

  TiXmlElement *clipNode = new TiXmlElement("clip");
  _parent->LinkEndChild( clipNode );
  clipNode->SetAttribute( _camera->clipNear.GetKey(),
                          _camera->clipNear.GetAsString() );
  clipNode->SetAttribute( _camera->clipFar.GetKey(),
                          _camera->clipFar.GetAsString() );

  TiXmlElement *saveNode = new TiXmlElement("save");
  _parent->LinkEndChild( saveNode );
  saveNode->SetAttribute( _camera->saveEnabled.GetKey(),
                          _camera->saveEnabled.GetAsString() );
  saveNode->SetAttribute( _camera->savePath.GetKey(),
                          _camera->savePath.GetAsString() );

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<RaySensor> &_ray)
{
  TiXmlElement *scanNode = new TiXmlElement("scan");
  _parent->LinkEndChild( scanNode );
  scanNode->SetAttribute( _ray->display.GetKey(),
                          _ray->display.GetAsString() );

  TiXmlElement *horizontalNode = new TiXmlElement("horizontal");
  _parent->LinkEndChild( horizontalNode );
  horizontalNode->SetAttribute( _ray->horizontalSamples.GetKey(),
                                _ray->horizontalSamples.GetAsString() );
  horizontalNode->SetAttribute( _ray->horizontalResolution.GetKey(),
                                _ray->horizontalResolution.GetAsString() );
  horizontalNode->SetAttribute( _ray->horizontalMinAngle.GetKey(),
                                _ray->horizontalMinAngle.GetAsString() );
  horizontalNode->SetAttribute( _ray->horizontalMaxAngle.GetKey(),
                                _ray->horizontalMaxAngle.GetAsString() );

  TiXmlElement *verticalNode = new TiXmlElement("vertical");
  _parent->LinkEndChild( verticalNode );
  verticalNode->SetAttribute( _ray->verticalSamples.GetKey(),
                              _ray->verticalSamples.GetAsString() );
  verticalNode->SetAttribute( _ray->verticalResolution.GetKey(),
                              _ray->verticalResolution.GetAsString() );
  verticalNode->SetAttribute( _ray->verticalMinAngle.GetKey(),
                              _ray->verticalMinAngle.GetAsString() );
  verticalNode->SetAttribute( _ray->verticalMaxAngle.GetKey(),
                              _ray->verticalMaxAngle.GetAsString() );

  TiXmlElement *rangeNode = new TiXmlElement("range");
  _parent->LinkEndChild( rangeNode );
  rangeNode->SetAttribute( _ray->rangeMin.GetKey(),
                           _ray->rangeMin.GetAsString() );
  rangeNode->SetAttribute( _ray->rangeMax.GetKey(),
                           _ray->rangeMax.GetAsString() );
  rangeNode->SetAttribute( _ray->rangeResolution.GetKey(),
                           _ray->rangeResolution.GetAsString() );
  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Inertial> &_inertial)
{
  TiXmlElement *inertialNode = new TiXmlElement("inertial");
  _parent->LinkEndChild( inertialNode );
  inertialNode->SetAttribute(_inertial->mass.GetKey(),
                             _inertial->mass.GetAsString());
  inertialNode->SetAttribute(_inertial->origin.GetKey(),
                             _inertial->origin.GetAsString());

  TiXmlElement *inertiaNode = new TiXmlElement("inertia");
  inertialNode->LinkEndChild(inertiaNode);
  inertiaNode->SetAttribute( _inertial->ixx.GetKey(),
                             _inertial->ixx.GetAsString() );
  inertiaNode->SetAttribute( _inertial->ixy.GetKey(),
                             _inertial->ixy.GetAsString() );
  inertiaNode->SetAttribute( _inertial->ixz.GetKey(),
                             _inertial->ixz.GetAsString() );
  inertiaNode->SetAttribute( _inertial->iyy.GetKey(),
                             _inertial->iyy.GetAsString() );
  inertiaNode->SetAttribute( _inertial->iyz.GetKey(),
                             _inertial->iyz.GetAsString() );
  inertiaNode->SetAttribute( _inertial->izz.GetKey(),
                             _inertial->izz.GetAsString() );

  return true;
}

bool saveXml(TiXmlElement *_parent, boost::shared_ptr<Surface> &_surface)
{
  TiXmlElement *surfaceNode = new TiXmlElement("surface");
  _parent->LinkEndChild( surfaceNode );

  TiXmlElement *bounceNode = new TiXmlElement("bounce");
  surfaceNode->LinkEndChild( bounceNode );
  bounceNode->SetAttribute( _surface->bounceRestCoeff.GetKey(),
                            _surface->bounceRestCoeff.GetAsString() );
  bounceNode->SetAttribute( _surface->bounceThreshold.GetKey(),
                            _surface->bounceThreshold.GetAsString() );

  if (_surface->frictions.size() > 0)
  {
    TiXmlElement *frictionNode = new TiXmlElement("friction");
    surfaceNode->LinkEndChild( frictionNode );

    std::vector< boost::shared_ptr<Friction> >::const_iterator fiter;
    for (fiter = _surface->frictions.begin(); fiter != _surface->frictions.end(); fiter++)
    {
      if ((*fiter)->type == Friction::ODE)
      {
        boost::shared_ptr<ODEFriction> odeFriction = boost::shared_static_cast<ODEFriction>(*fiter);
        saveXml(frictionNode, odeFriction);
      }
    }
  }

  if (_surface->contacts.size() > 0)
  {
    TiXmlElement *contactNode = new TiXmlElement("contact");
    surfaceNode->LinkEndChild( contactNode );

    std::vector< boost::shared_ptr<Contact> >::const_iterator citer;
    for (citer = _surface->contacts.begin(); citer != _surface->contacts.end(); citer++)
    {
      if ((*citer)->type == Contact::ODE)
      {
        boost::shared_ptr<ODEContact> odeContact = boost::shared_static_cast<ODEContact>(*citer);
        saveXml(contactNode, odeContact);
      }
    }
  }

  return true;
}

bool saveXml(TiXmlElement *_parent, boost::shared_ptr<ODEFriction> &_friction)
{
  TiXmlElement *odeNode = new TiXmlElement("ode");
  _parent->LinkEndChild( odeNode );

  odeNode->SetAttribute( _friction->mu.GetKey(),
                         _friction->mu.GetAsString() );
  odeNode->SetAttribute( _friction->mu2.GetKey(),
                         _friction->mu2.GetAsString() );
  odeNode->SetAttribute( _friction->fdir1.GetKey(),
                         _friction->fdir1.GetAsString() );
  odeNode->SetAttribute( _friction->slip1.GetKey(),
                         _friction->slip1.GetAsString() );
  odeNode->SetAttribute( _friction->slip2.GetKey(),
                         _friction->slip2.GetAsString() );
  return true;
}

bool saveXml(TiXmlElement *_parent, boost::shared_ptr<ODEContact> &_contact)
{
  TiXmlElement *odeNode = new TiXmlElement("ode");
  _parent->LinkEndChild( odeNode );

  odeNode->SetAttribute( _contact->softCFM.GetKey(),
                         _contact->softCFM.GetAsString() );
  odeNode->SetAttribute( _contact->kp.GetKey(),
                         _contact->kp.GetAsString() );
  odeNode->SetAttribute( _contact->kd.GetKey(),
                         _contact->kd.GetAsString() );
  odeNode->SetAttribute( _contact->maxVel.GetKey(),
                         _contact->maxVel.GetAsString() );
  odeNode->SetAttribute( _contact->minDepth.GetKey(),
                         _contact->minDepth.GetAsString() );

  return true;
}

}
