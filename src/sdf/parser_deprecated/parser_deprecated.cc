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
#include "sdf/parser_deprecated/parser_deprecated.hh"

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

bool getPlugins(xmlNodePtr _parentXml, 
                std::map<std::string, boost::shared_ptr<Plugin> > &_plugins)
{
  // Get all plugins 
  for (xmlNodePtr pluginXml = _parentXml->xmlChildrenNode;
      pluginXml != NULL; pluginXml = pluginXml->next)
  {
    if (pluginXml->ns && std::string("controller") == (const char*)pluginXml->ns->prefix)
    {
      boost::shared_ptr<Controller> controller;
      controller.reset(new Controller);

      if (initXml(pluginXml, controller))
      {
        if (_plugins.find(controller->name.GetValue()) != _plugins.end())
        {
          gzerr << "controller '" << controller->name.GetValue() << "' is not unique.\n";
          controller.reset();
          return false;
        }
        else
        {
          boost::shared_ptr<Plugin> plugin = boost::shared_static_cast<Plugin>(controller);
          _plugins.insert(make_pair(controller->name.GetValue(),plugin));
        }
      }
      else
      {
        gzerr << "controller xml is not initialized correctly\n";
        controller.reset();
        return false;
      }
    }
  }

  return true;
}

bool initXml(xmlNodePtr _config, boost::shared_ptr<SensorType> &_sensorType)
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

bool initXml(xmlNodePtr _config, boost::shared_ptr<Sensor> &_sensor)
{
  _sensor->Clear();

  /// Get all the plugins
  getPlugins(_config, _sensor->plugins);

  if (!_sensor->name.Set( (const char*)xmlGetProp(_config, (xmlChar*)"name")))
  {
     gzerr << "Unable to parse sensor name.\n";
    return false;
  }
     
  if (!_sensor->type.Set( (const char*)xmlGetProp(_config, (xmlChar*)"type")))
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

  if (!_sensor->alwaysOn.Set( (const char*)xmlGetProp(_config, (xmlChar*)"always_on")))
  {
    gzerr << "Unable to parse sensor always_on\n";
    return false;
  }

  if (!_sensor->updateRate.Set( (const char*)xmlGetProp(_config, (xmlChar*)"update_rate")))
  {
    gzerr << "Unable to parse sensor update_rate\n";
    return false;
  }

  // Origin
  xmlNodePtr o = FirstChildElement(_config, "origin");
  if (!o)
  {
    gzwarn << "Origin tag not present for sensor element, using default (Identity)\n";
    _sensor->origin.Reset();
  }
  else
  {
    if (!_sensor->origin.Set( (const char*)xmlGetProp(o,(xmlChar*)"xyz"), (const char*)xmlGetProp(o,(xmlChar*)"rpy") ))
    {
      gzerr << "Sensor has malformed orgin\n";
      return false;
    }
  }

  return true;
}

bool initXml(xmlNodePtr _config, boost::shared_ptr<CameraSensor> &_sensor)
{
  _sensor->Clear();

  // Horizontal field of view
  xmlNodePtr hfov = FirstChildElement(_config, "horizontal_fov");
  if(!hfov)
  {
    gzerr << "CameraSensor missing horizontal_fov element\n";
    return false;
  }

  if (!_sensor->horizontalFov.Set( (const char*)xmlGetProp(hfov,(xmlChar*)"angle")))
  {
    gzerr << "Unable to parse camera horizontal_fov angle attribute\n";
    return false;
  }

  // Image 
  xmlNodePtr image = FirstChildElement(_config, "image");
  if (!image)
  {
    gzerr << "CameraSensor sensor requires an image element \n";
    return false;
  }

  if (!_sensor->imageWidth.Set( (const char*)xmlGetProp(image,(xmlChar*)"width")))
  {
    gzerr << "Unable to parse sensor image width\n";
    return false;
  }

  if (!_sensor->imageHeight.Set( (const char*)xmlGetProp(image,(xmlChar*)"height")))
  {
    gzerr << "Unable to parse sensor image height\n";
    return false;
  }

  if (!_sensor->imageFormat.Set( (const char*)xmlGetProp(image,(xmlChar*)"format")))
  {
    gzerr << "Unable to parse sensor image format\n";
    return false;
  }

  // clip 
  xmlNodePtr clip = FirstChildElement(_config, "clip");
  if (!clip)
  {
    gzerr << "CameraSensor sensor requires an clip element \n";
    return false;
  }

  if (!_sensor->clipNear.Set( (const char*)xmlGetProp(clip,(xmlChar*)"near") ))
  {
    gzerr << "Unable to parse camera near clip";
    return false;
  }

  if (!_sensor->clipFar.Set( (const char*)xmlGetProp(clip,(xmlChar*)"far") ))
  {
    gzerr << "Unable to parse camera far clip";
    return false;
  }

  // save 
  xmlNodePtr save = FirstChildElement(_config, "save");
  if (save)
  {
    if (!_sensor->saveEnabled.Set( (const char*)xmlGetProp(save,(xmlChar*)"enabled")))
    {
      gzerr << "Unable to parse camera save enabled flag\n";
      return false;
    }

    if (!_sensor->savePath.Set( (const char*)xmlGetProp(save,(xmlChar*)"path")))
    {
      gzerr << "Unable to parse camera save path\n";
      return false;
    }
  }

  return true;
}

bool initXml(xmlNodePtr _config, boost::shared_ptr<RaySensor> &_sensor)
{
  _sensor->Clear();

  // scan 
  xmlNodePtr scan = FirstChildElement(_config, "scan");
  if (scan)
  {
    if (!_sensor->display.Set( (const char*)xmlGetProp(scan,(xmlChar*)"display")))
    {
      gzerr << "Unable to parse ray display flag\n";
      return false;
    }

    // Horizontal scans
    xmlNodePtr horizontal = FirstChildElement(scan, "horizontal");
    if (!horizontal)
    {
      gzerr << "missing horizontal element";
      return false;
    }

    if (!_sensor->horizontalSamples.Set( (const char*)xmlGetProp(horizontal,(xmlChar*)"samples")))
    {
      gzerr << "Unable to parse ray sensor horizontal samples";
      return false;
    }

    if (!_sensor->horizontalResolution.Set( (const char*)xmlGetProp(horizontal,(xmlChar*)"resolution")))
    {
      gzerr << "Unable to parse ray sensor horizontal resolution";
      return false;
    }

    if (!_sensor->horizontalMinAngle.Set( (const char*)xmlGetProp(horizontal,(xmlChar*)"min_angle")))
    {
      gzerr << "Unable to parse ray sensor horizontal min_angle";
      return false;
    }

    if (!_sensor->horizontalMaxAngle.Set( (const char*)xmlGetProp(horizontal,(xmlChar*)"max_angle")))
    {
      gzerr << "Unable to parse ray sensor horizontal max_angle";
      return false;
    }

    // Vertical scans
    xmlNodePtr vertical = FirstChildElement(_config, "vertical");
    if (!vertical)
    {
      gzerr << "missing vertical element\n";
      return false;
    }

    if (!_sensor->verticalSamples.Set( (const char*)xmlGetProp(vertical,(xmlChar*)"samples")))
    {
      gzerr << "Unable to parse ray sensor vertical samples";
      return false;
    }

    if (!_sensor->verticalResolution.Set( (const char*)xmlGetProp(vertical,(xmlChar*)"resolution")))
    {
      gzerr << "Unable to parse ray sensor vertical resolution";
      return false;
    }

    if (!_sensor->verticalMinAngle.Set( (const char*)xmlGetProp(vertical,(xmlChar*)"min_angle")))
    {
      gzerr << "Unable to parse ray sensor vertical min_angle";
      return false;
    }

    if (!_sensor->verticalMaxAngle.Set( (const char*)xmlGetProp(vertical,(xmlChar*)"max_angle")))
    {
      gzerr << "Unable to parse ray sensor vertical max_angle";
      return false;
    }


    // range 
    xmlNodePtr range = FirstChildElement(_config, "range");
    if (!range)
    {
      gzerr << "ray sensor missing range element\n";
      return false;
    }

    if (!_sensor->rangeMin.Set( (const char*)xmlGetProp(range,(xmlChar*)"min")))
    {
      gzerr << "Invalid min range\n";
      return false;
    }

    if (!_sensor->rangeMax.Set( (const char*)xmlGetProp(range,(xmlChar*)"max")))
    {
      gzerr << "Invalid max range\n";
      return false;
    }

    if (!_sensor->rangeResolution.Set( (const char*)xmlGetProp(range,(xmlChar*)"resolution")))
    {
      gzerr << "Invalid range resolution\n";
      return false;
    }
  }

  return true;
}

bool initXml(xmlNodePtr _config, boost::shared_ptr<Material> &_material)
{
  bool hasRGB = false;

  _material->Clear();

  if (!_material->script.Set( (const char*)xmlGetProp(_config,(xmlChar*)"script") ))
  {
    gzerr << "Unable to parse material script attribute\n";
    return false;
  }

  // color
  xmlNodePtr c = FirstChildElement(_config, "color");
  if (c)
  {
    if ( (const char*)xmlGetProp(c,(xmlChar*)"rgba"))
    {
      if (!_material->color.Set( (const char*)xmlGetProp(c,(xmlChar*)"rgba")))
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

bool initXml(xmlNodePtr _config, boost::shared_ptr<Inertial> &_inertial)
{
  _inertial->Clear();

  // Origin
  xmlNodePtr o = FirstChildElement(_config, "origin");
  if (!o)
  {
    gzwarn << "INFO: Origin tag not present for inertial element, using default (Identity)\n";
    _inertial->origin.Reset();
  }
  else
  {
    if (!_inertial->origin.Set( (const char*)xmlGetProp(o,(xmlChar*)"xyz"), (const char*)xmlGetProp(o,(xmlChar*)"rpy")))
    {
      gzerr << "Unable to parse origin tag\n";
      _inertial->origin.Reset();
      return false;
    }
  }

  if (! (const char*)xmlGetProp(_config,(xmlChar*)"mass"))
  {
    gzerr << "Inertial element must have mass attribute.";
    return false;
  }

  if (!_inertial->mass.Set( (const char*)xmlGetProp(_config,(xmlChar*)"mass")))
  {
    gzerr << "Unable to parse inerital mass attribute\n";
    return false;
  }

  xmlNodePtr inertiaXml = FirstChildElement(_config, "inertia");
  if (!inertiaXml)
  {
    gzerr << "Inertial element must have inertia element\n";
    return false;
  }

  if (!( (const char*)xmlGetProp(inertiaXml,(xmlChar*)"ixx") && (const char*)xmlGetProp(inertiaXml,(xmlChar*)"ixy") && 
         (const char*)xmlGetProp(inertiaXml,(xmlChar*)"ixz") && (const char*)xmlGetProp(inertiaXml,(xmlChar*)"iyy") && 
         (const char*)xmlGetProp(inertiaXml,(xmlChar*)"iyz") && (const char*)xmlGetProp(inertiaXml,(xmlChar*)"izz")))
  {
    gzerr << "Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes\n";
    return false;
  }

  if (!_inertial->ixx.Set( (const char*)xmlGetProp(inertiaXml,(xmlChar*)"ixx")) || 
      !_inertial->ixy.Set( (const char*)xmlGetProp(inertiaXml,(xmlChar*)"ixy")) || 
      !_inertial->ixz.Set( (const char*)xmlGetProp(inertiaXml,(xmlChar*)"ixz")) || 
      !_inertial->iyy.Set( (const char*)xmlGetProp(inertiaXml,(xmlChar*)"iyy")) || 
      !_inertial->iyz.Set( (const char*)xmlGetProp(inertiaXml,(xmlChar*)"iyz")) || 
      !_inertial->izz.Set( (const char*)xmlGetProp(inertiaXml,(xmlChar*)"izz")) )
  {
    gzerr << "one of the inertia elements: "
      << "ixx (" << (const char*)xmlGetProp(inertiaXml,(xmlChar*)"ixx") << ") "
      << "ixy (" << (const char*)xmlGetProp(inertiaXml,(xmlChar*)"ixy") << ") "
      << "ixz (" << (const char*)xmlGetProp(inertiaXml,(xmlChar*)"ixz")  << ") "
      << "iyy (" << (const char*)xmlGetProp(inertiaXml,(xmlChar*)"iyy")  << ") "
      << "iyz (" << (const char*)xmlGetProp(inertiaXml,(xmlChar*)"iyz")  << ") "
      << "izz (" << (const char*)xmlGetProp(inertiaXml,(xmlChar*)"izz")  << ") "
      << "is not a valid double.\n";
    return false;
  }

  return true;
}

bool initXml(xmlNodePtr _config, boost::shared_ptr<Collision> &_collision)
{
  _collision->Clear();

  if (!_collision->name.Set( (const char*)xmlGetProp(_config,(xmlChar*)"name")))
  {
    gzerr << "Unable to parse collision name\n";
    return false;
  }

  // Origin
  xmlNodePtr o = FirstChildElement(_config, "origin");
  if (!o)
  {
    gzwarn << "Origin tag not present for collision element, using default (Identity\n";
    _collision->origin.Reset();
  }
  else if (!_collision->origin.Set( (const char*)xmlGetProp(o,(xmlChar*)"xyz"), (const char*)xmlGetProp(o,(xmlChar*)"rpy")))
  {
    gzerr << "Unable to parse collision origin element\n";
    _collision->origin.Reset();
    return false;
  }

  // Geometry
  xmlNodePtr geom = FirstChildElement(_config, "geometry");
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

bool initXml(xmlNodePtr _config, boost::shared_ptr<Sphere> &_sphere)
{
  _sphere->Clear();

  if (!_sphere->radius.Set( (const char*)xmlGetProp(_config,(xmlChar*)"radius")))
  {
    gzerr << "Unable to parse sphere's radius attribute\n";
    return false;
  }

  return true;
}

bool initXml(xmlNodePtr _config, boost::shared_ptr<Box> &_box)
{
  _box->Clear();

  if (!_box->size.Set( (const char*)xmlGetProp(_config,(xmlChar*)"size")))
  {
    gzerr << "Unable to parse sphere's size attribute\n";
    return false;
  }

  return true;
}

bool initXml(xmlNodePtr _config, boost::shared_ptr<Cylinder> &_cylinder)
{
  _cylinder->Clear();

  if (!_cylinder->length.Set( (const char*)xmlGetProp(_config,(xmlChar*)"length")))
  {
    gzerr << "Unable to parse cylinder's length attribute\n";
    return false;
  }

  if (!_cylinder->radius.Set( (const char*)xmlGetProp(_config,(xmlChar*)"radius")))
  {
    gzerr << "Unable to parse cylinder's radius attribute\n";
    return false;
  }

  return true;
}

bool initXml(xmlNodePtr _config, boost::shared_ptr<Mesh> &_mesh)
{
  _mesh->Clear();

  if (!_mesh->filename.Set( (const char*)xmlGetProp(_config,(xmlChar*)"filename")))
  {
    gzerr << "Unable to parse mesh's filename attribute\n";
    return false;
  }

  if (!_mesh->scale.Set( (const char*)xmlGetProp(_config,(xmlChar*)"scale")))
  {
    gzerr << "Unable to parse mesh's scale attribute\n";
    return false;
  }

  return true;
}

bool initXml(xmlNodePtr _config, boost::shared_ptr<Link> &_link)
{
  _link->Clear();

  if (!_link->name.Set( (const char*)xmlGetProp(_config,(xmlChar*)"name")))
  {
    gzerr << "Unable to parse value for link name.\n";
    return false;
  }

  // Inertial (optional)
  xmlNodePtr i = FirstChildElement(_config, "inertial");
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
  for (xmlNodePtr  visXml = FirstChildElement(_config, "visual"); 
      visXml; visXml = NextSiblingElement(visXml, "visual"))
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
  for (xmlNodePtr  colXml = FirstChildElement(_config, "collision"); 
      colXml; colXml = NextSiblingElement(colXml, "collision"))
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
  for (xmlNodePtr  sensorXml = FirstChildElement(_config, "sensor"); 
      sensorXml; sensorXml = NextSiblingElement(sensorXml, "sensor"))
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

bool initXml(xmlNodePtr _config, boost::shared_ptr<Visual> &_visual)
{
  _visual->Clear();

  if (!_visual->name.Set( (const char*)xmlGetProp(_config,(xmlChar*)"name")))
  {
    gzerr << "Unable to parse visual name attribute\n";
    return false;
  }

  if (!_visual->castShadows.Set( (const char*)xmlGetProp(_config,(xmlChar*)"cast_shadows")))
  {
    gzerr << "Unable to parse visual cast_shadows attribute\n";
    return false;
  }

  // Origin
  xmlNodePtr o = FirstChildElement(_config, "origin");
  if (!o)
  {
    gzwarn << "Origin tag not present for visual element, using default (Identity)\n";
    _visual->origin.Reset();
  }
  else if (!_visual->origin.Set( (const char*)xmlGetProp(o,(xmlChar*)"xyz"), (const char*)xmlGetProp(o,(xmlChar*)"rpy")))
  {
    gzerr << "Unable to parase visual origin element\n";
    _visual->origin.Reset();
    return false;
  }

  // Geometry
  xmlNodePtr geometryXml = FirstChildElement(_config, "geometry");
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
  xmlNodePtr mat = FirstChildElement(_config, "material");
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

bool initXml(xmlNodePtr _config, boost::shared_ptr<JointDynamics> &_jointDynamics)
{
  _jointDynamics->Clear();

  if (!_jointDynamics->damping.Set( (const char*)xmlGetProp(_config,(xmlChar*)"damping")))
  {
    gzerr << "Unable to parse joint dynamics damping attribute\n";
    return false;
  }

  if (!_jointDynamics->friction.Set( (const char*)xmlGetProp(_config,(xmlChar*)"friction")))
  {
    gzerr << "Unable to parse joint dynamics friction attribute\n";
    return false;
  }

  return true;
}


bool initXml(xmlNodePtr _config, 
             boost::shared_ptr<JointLimits> &_jointLimits)
{
  _jointLimits->Clear();

  if (!_jointLimits->lower.Set( (const char*)xmlGetProp(_config,(xmlChar*)"lower")))
  {
    gzerr << "Unable to parse joint limit lower\n";
    return false;
  }

  if (!_jointLimits->upper.Set( (const char*)xmlGetProp(_config,(xmlChar*)"upper")))
  {
    gzerr << "Unable to parse joint limit upper\n";
    return false;
  }

  if (!_jointLimits->effort.Set( (const char*)xmlGetProp(_config,(xmlChar*)"effort")))
  {
    gzerr << "Unable to parse joint limit effort\n";
    return false;
  }

  if (!_jointLimits->effort.Set( (const char*)xmlGetProp(_config,(xmlChar*)"velocity")))
  {
    gzerr << "Unable to parse joint limit velocity\n";
    return false;
  }

  return true;
}

bool initXml(xmlNodePtr _config, boost::shared_ptr<Joint> &_joint)
{
  _joint->Clear();

  // Get Joint Name
  if (!_joint->name.Set( (const char*)xmlGetProp(_config,(xmlChar*)"name")))
  {
    gzerr << "Unable to parse joint name attribute\n";
    return false;
  }

  // Get transform from Parent Link to Joint Frame
  xmlNodePtr originXml = FirstChildElement(_config, "origin");
  if (!originXml)
  {
    _joint->origin.Reset();
  }
  else
  {
    if (!_joint->origin.Set( (const char*)xmlGetProp(originXml,(xmlChar*)"xyz"),
                             (const char*)xmlGetProp(originXml,(xmlChar*)"rpy")))
    {
      gzerr << "Unable to parse joint origin element\n";
      _joint->origin.Reset();
      return false;
    }
  }

  // Get Parent Link
  xmlNodePtr parentXml = FirstChildElement(_config, "parent");
  if (parentXml)
  {
    if (!_joint->parentLinkName.Set( (const char*)xmlGetProp(parentXml,(xmlChar*)"link")))
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
  xmlNodePtr childXml = FirstChildElement(_config, "child");
  if (childXml)
  {
    if (!_joint->childLinkName.Set( (const char*)xmlGetProp(childXml,(xmlChar*)"link")))
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
  if (!_joint->type.Set( (const char*)xmlGetProp(_config,(xmlChar*)"type")))
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
    xmlNodePtr axisXml = FirstChildElement(_config, "axis");
    if (!axisXml)
    {
      gzwarn << "no axis elemement for Joint link '" 
        << _joint->name << "', defaulting to (1,0,0) axis\n";
      _joint->axis.SetValue( Vector3(1.0, 0.0, 0.0));
    }
    else{
      if (!_joint->axis.Set( (const char*)xmlGetProp(axisXml,(xmlChar*)"xyz")) )
      {
        gzerr << "Unable to parse axis for joint[" << _joint->name << "]\n";
        return false;
      }
    }
  }

  // Get limit
  xmlNodePtr limitXml = FirstChildElement(_config, "limit");
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
  xmlNodePtr propXml = FirstChildElement(_config, "dynamics");
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
  std::ifstream fin;
  fin.open(_filename.c_str(), std::ios::in);
  if( !fin.is_open() )
  {
    gzerr << "The model file can not be opened, check path and permissions\n";
  }
  fin.close();

  // Enable line numbering
  xmlLineNumbersDefault( 1 );

  std::string output;
  PreParser(_filename, output);

  initString(output, _model);
  return true;
}


////////////////////////////////////////////////////////////////////////////////
bool initString(const std::string &_xmlString, boost::shared_ptr<Model> &_model)
{
  xmlDocPtr xmlDoc = xmlParseDoc((xmlChar*)_xmlString.c_str());

  return initDoc(xmlDoc, _model);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Load Model from xmlDoc
bool initDoc(xmlDocPtr _xmlDoc, boost::shared_ptr<Model> &_model)
{
  if (!_xmlDoc)
  {
    gzerr << "Could not parse the xml\n";
    return false;
  }

  xmlNodePtr modelXml = FirstChildElement(xmlDocGetRootElement(_xmlDoc), "model");
  if (!modelXml)
  {
    gzerr << "Could not find the 'model' element in the xml file\n";
    return false;
  }

  return initXml(modelXml, _model);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Load Model from xmlNode
bool initXml(xmlNodePtr _xml, boost::shared_ptr<Model> &_model)
{
  _model->Clear();

  if (!_xml) 
    return false;

  if (!_model->name.Set( (const char*)xmlGetProp(_xml,(xmlChar*)"name")))
  {
    gzerr << "No name given for the model.\n";
    return false;
  }

  // Get all Link elements
  for (xmlNodePtr  linkXml = FirstChildElement(_xml, "link"); 
      linkXml; linkXml = NextSiblingElement(linkXml, "link"))
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
  for (xmlNodePtr  jointXml = FirstChildElement(_xml, "joint"); 
      jointXml; jointXml = NextSiblingElement(jointXml, "joint"))
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
bool initXml(xmlNodePtr _config, boost::shared_ptr<Geometry> &_geometry)
{
  bool result = false;
  xmlNodePtr shape = NULL;

  if (!_config)
  {
    gzerr << "Null xml\n";
    return false;
  }

  shape = _config->xmlChildrenNode;
  if (!shape)
  {
    gzerr << "Geometry tag contains no child element.\n";
    return false;
  }

  std::string typeName = (const char*)shape->name;
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

bool initXml(xmlNodePtr _config, boost::shared_ptr<Controller> &_controller)
{
  _controller->Clear();

  _controller->type = (const char*)_config->name;

  if (!_controller->name.Set( (const char*)xmlGetProp(_config,(xmlChar*)"name")))
  {
    gzerr << "Unable to parse the plugin name.\n";
    return false;
  }

  if (!_controller->filename.Set( (const char*)xmlGetProp(_config,(xmlChar*)"filename")))
  {
    gzerr << "Unable to parse plugin filename.\n";
    return false;
  }

  return true;
}


////////////////////////////////////////////////////////////////////////////////
bool initFile(const std::string &_filename, boost::shared_ptr<World> &_world)
{
  std::ifstream fin;
  fin.open(_filename.c_str(), std::ios::in);
  if( !fin.is_open() )
  {
    gzerr << "The world file can not be opened, check path and permissions\n";
  }
  fin.close();

  // Enable line numbering
  xmlLineNumbersDefault( 1 );

  std::string output;
  PreParser(_filename, output);

  initString(output, _world);
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool initString(const std::string &_xmlString, boost::shared_ptr<World> &_world)
{
  xmlDocPtr xmlDoc = xmlParseDoc((xmlChar*)_xmlString.c_str());

  return initDoc(xmlDoc,_world);
}

////////////////////////////////////////////////////////////////////////////////
bool initDoc(xmlDocPtr _xmlDoc, boost::shared_ptr<World> &_world)
{
  if (!_xmlDoc)
  {
    gzerr << "Could not parse the xml\n";
    return false;
  }

  xmlNodePtr worldXml = FirstChildElement(xmlDocGetRootElement(_xmlDoc), "world");
  if (!worldXml)
  {
    gzerr << "Could not find the 'world' element in the xml file\n";
    return false;
  }

  return initXml(worldXml, _world);
}

////////////////////////////////////////////////////////////////////////////////
bool initXml(xmlNodePtr _worldXml, boost::shared_ptr<World> &_world)
{
  _world->Clear();

  if (!_worldXml) 
  {
    gzerr << "Error: World XML is NULL\n";
    return false;
  }

  // Get world name
  if (!_world->name.Set( (const char*)xmlGetProp(_worldXml,(xmlChar*)"name")))
  {
    gzerr << "Unable to parse world name\n";
    return false;
  }

  _world->scene.reset(new Scene);
  initXml(FirstChildElement(_worldXml, "scene"),_world->scene);

  _world->physics.reset(new Physics);
  initXml(FirstChildElement(_worldXml, "physics"),_world->physics);

  // Get all model elements
  for (xmlNodePtr  modelXml = FirstChildElement(_worldXml, "model"); 
      modelXml; modelXml = NextSiblingElement(modelXml, "model"))
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
  for (xmlNodePtr  jointXml = FirstChildElement(_worldXml, "joint"); 
      jointXml; jointXml = NextSiblingElement(jointXml, "joint"))
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
bool initXml(xmlNodePtr  _config, boost::shared_ptr<Scene> &_scene)
{
  _scene->Clear();

  xmlNodePtr ambient = FirstChildElement(_config, "ambient");
  if (ambient)
  {
    if (!_scene->ambientColor.Set( (const char*)xmlGetProp(ambient,(xmlChar*)"rgba")))
    {
      gzerr << "Unable to parse ambient color\n";
      return false;
    }
  }

  xmlNodePtr background = FirstChildElement(_config, "background");
  if (background)
  {
    if (!_scene->backgroundColor.Set( (const char*)xmlGetProp(background,(xmlChar*)"rgba")))
    {
      gzerr << "Unable to parse background rgba\n";
      return false;
    }


    xmlNodePtr sky = FirstChildElement(background, "sky");
    if (sky)
    {
      if (!_scene->skyMaterial.Set( (const char*)xmlGetProp(sky,(xmlChar*)"material")))
      {
        gzerr << "Unable to parse material for the sky.\n";
        return false;
      }
    }
  }

  xmlNodePtr shadow = FirstChildElement(_config, "shadows");
  if (shadow)
  {
    if (!_scene->shadowEnabled.Set( (const char*)xmlGetProp(shadow,(xmlChar*)"enabled")))
    {
      gzerr << "Shadow element requires an enabled attribute";
      return false;
    }

    if (!_scene->shadowColor.Set( (const char*)xmlGetProp(shadow,(xmlChar*)"rgba")))
    {
      gzerr << "Unable to parse shadow rgba\n";
      return false;
    }


    if (!_scene->shadowType.Set( (const char*)xmlGetProp(shadow,(xmlChar*)"type")))
    {
      gzerr << "Unable to parse shadow type attribute\n";
      return false;
    }
  }

  return true;
}

bool initXml(xmlNodePtr _config, boost::shared_ptr<OpenDynamicsEngine> &_openDynamicsEngine)
{
  xmlNodePtr odeConfig = FirstChildElement(_config, "ode");

  if ( !odeConfig )
  {
    gzerr << "Physics element missing <ode>\n";
    return false;
  }

  xmlNodePtr solverConfig = FirstChildElement(odeConfig, "solver");
  if (!solverConfig)
  {
    gzerr << "ODE Physics missing solver element\n";
    return false;
  }

  if (!_openDynamicsEngine->solverType.Set( (const char*)xmlGetProp(solverConfig,(xmlChar*)"type")))
  {
    gzerr << "ODE Physics missing solver type\n";
    return false;
  }

  if (!_openDynamicsEngine->dt.Set( (const char*)xmlGetProp(solverConfig,(xmlChar*)"dt")))
  {
    gzerr << "ODE Physics solver unable to parse dt attribute\n";
    return false;
  }

  if (!_openDynamicsEngine->iters.Set( (const char*)xmlGetProp(solverConfig,(xmlChar*)"iters")))
  {
    gzerr << "ODE Physics solver malformed iters attribute\n";
    return false;
  }

  if (!_openDynamicsEngine->sor.Set( (const char*)xmlGetProp(solverConfig,(xmlChar*)"sor"))) 
  {
    gzerr << "ODE Physics solver unable to parse sor attribute\n)";
    return false;
  }


  // Contraints
  xmlNodePtr constraintsConfig = FirstChildElement(odeConfig, "constraints");
  if (constraintsConfig)
  {

    if (!_openDynamicsEngine->cfm.Set( (const char*)xmlGetProp(constraintsConfig,(xmlChar*)"cfm"))) 
    {
      gzerr << "ODE Physics contraints unable to parse cfm attribute\n";
      return false;
    }

    if (!_openDynamicsEngine->erp.Set( (const char*)xmlGetProp(constraintsConfig,(xmlChar*)"erp"))) 
    {
      gzerr << "ODE Physics contraints unable to parse erp attribute\n";
      return false;
    }


    if (!_openDynamicsEngine->contactMaxCorrectingVel.Set( (const char*)xmlGetProp(constraintsConfig,(xmlChar*)"contact_max_correcting_vel")))
    {
      gzerr << "ODE Physics contraints unable to parse contact_max_correcting_vel attribute\n";
      return false;
    } 

    if (!_openDynamicsEngine->contactSurfaceLayer.Set( (const char*)xmlGetProp(constraintsConfig,(xmlChar*)"contact_surface_layer")))
    {
      gzerr << "ODE Physics contraints unable to parse contact_surface_layer attribute\n";
      return false;
    }
  }

  return true;
}

bool initXml(xmlNodePtr  _config, boost::shared_ptr<Physics> &_physics)
{
  _physics->Clear();

  if (!_config)
  {
    gzerr << "xml config is NULL\n";
    return false;
  }

  if (!_physics->type.Set( (const char*)xmlGetProp(_config,(xmlChar*)"type")))
  {
    gzerr << "Unable to parse physics type attribute\n";
    return false;
  }

  xmlNodePtr gravityElement = FirstChildElement(_config, "gravity");
  if (gravityElement)
  {
    if (!_physics->gravity.Set( (const char*)xmlGetProp(gravityElement,(xmlChar*)"xyz")))
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

bool initXml(xmlNodePtr  /*_config*/, 
             boost::shared_ptr<ContactSensor> &/*_contact*/)
{
  return true;
}

/*
bool saveXml(const std::string &filename, const boost::shared_ptr<World> &_world)
{
  xmlDoc doc;

  xmlDocPtr decl = new xmlDoc( "1.0", "", "" );
  doc.LinkEndChild(decl);

  xmlNodePtr root = new xmlNode("gazebo");
  doc.LinkEndChild(root);
  root->SetAttribute("version","1.0");

  xmlNodePtr worldElement = new xmlNode("world");
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

bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Scene> &_scene)
{
  xmlNodePtr sceneNode = new xmlNode("scene");
  _parent->LinkEndChild( sceneNode );

  xmlNodePtr ambientNode = new xmlNode("ambient");
  sceneNode->LinkEndChild( ambientNode );
  ambientNode->SetAttribute( _scene->ambientColor.GetKey(),
                             _scene->ambientColor.GetAsString() );

  xmlNodePtr backgroundNode = new xmlNode("background");
  sceneNode->LinkEndChild( backgroundNode );
  backgroundNode->SetAttribute( _scene->backgroundColor.GetKey(),
                                _scene->backgroundColor.GetAsString() );


  xmlNodePtr skyNode = new xmlNode("sky");
  backgroundNode->LinkEndChild( skyNode );
  skyNode->SetAttribute( _scene->skyMaterial.GetKey(),
                         _scene->skyMaterial.GetAsString());

  // Save shadows
  xmlNodePtr shadowNode = new xmlNode("shadows");
  sceneNode->LinkEndChild( shadowNode );
  shadowNode->SetAttribute( _scene->shadowEnabled.GetKey(),
                            _scene->shadowEnabled.GetAsString());
  shadowNode->SetAttribute( _scene->shadowColor.GetKey(),
                            _scene->shadowColor.GetAsString());
  shadowNode->SetAttribute( _scene->shadowType.GetKey(),
                            _scene->shadowType.GetAsString());

  return true;
}

bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Physics> &_physics)
{
  xmlNodePtr physicsNode = new xmlNode("physics");
  _parent->LinkEndChild( physicsNode );

  physicsNode->SetAttribute( _physics->type.GetKey(), 
                             _physics->type.GetAsString() );

  xmlNodePtr gravityNode = new xmlNode("gravity");
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

bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<OpenDynamicsEngine> &_engine)
{
  xmlNodePtr odeNode = new xmlNode("ode");
  _parent->LinkEndChild( odeNode );

  xmlNodePtr solverNode = new xmlNode("solver");
  odeNode->LinkEndChild( solverNode );

  solverNode->SetAttribute( _engine->solverType.GetKey(),
                            _engine->solverType.GetAsString() );
  solverNode->SetAttribute( _engine->dt.GetKey(),
                            _engine->dt.GetAsString() );
  solverNode->SetAttribute( _engine->iters.GetKey(),
                            _engine->iters.GetAsString() );
  solverNode->SetAttribute( _engine->sor.GetKey(),
                            _engine->sor.GetAsString() );


  xmlNodePtr constraintsNode = new xmlNode("constraints");
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

bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Joint> &_joint)
{
  xmlNodePtr jointNode = new xmlNode("joint");
  _parent->LinkEndChild( jointNode );

  jointNode->SetAttribute(_joint->name.GetKey(),
                          _joint->name.GetAsString());
  jointNode->SetAttribute(_joint->type.GetKey(),
                          _joint->type.GetAsString());

  xmlNodePtr parentNode = new xmlNode("parent");
  jointNode->LinkEndChild(parentNode);
  parentNode->SetAttribute( _joint->parentLinkName.GetKey(),
                            _joint->parentLinkName.GetAsString() );

  xmlNodePtr childNode = new xmlNode("child");
  jointNode->LinkEndChild(childNode);
  childNode->SetAttribute( _joint->childLinkName.GetKey(),
                           _joint->childLinkName.GetAsString() );

  saveXml(jointNode, _joint->origin);

  // Axis 1
  xmlNodePtr axisNode = new xmlNode("axis");
  jointNode->LinkEndChild(axisNode);
  axisNode->SetAttribute( _joint->axis.GetKey(), _joint->axis.GetAsString() );

  // Axis 2
  xmlNodePtr axis2Node = new xmlNode("axis2");
  jointNode->LinkEndChild(axis2Node);
  axis2Node->SetAttribute( _joint->axis2.GetKey(), _joint->axis2.GetAsString());

  // Dynamics
  if (_joint->dynamics)
  {
    xmlNodePtr dynamicsNode = new xmlNode("dynamics");
    jointNode->LinkEndChild(dynamicsNode);
    dynamicsNode->SetAttribute( _joint->dynamics->damping.GetKey(),
        _joint->dynamics->damping.GetAsString() );
    dynamicsNode->SetAttribute( _joint->dynamics->friction.GetKey(),
        _joint->dynamics->friction.GetAsString() );
  }

  // Limit
  if (_joint->limits)
  {
    xmlNodePtr limitNode = new xmlNode("limit");
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

bool saveXml(xmlNodePtr _parent, const Vector3 &_vec)
{
  std::ostringstream stream;
  stream << _vec;
  _parent->SetAttribute("xyz", stream.str());
  return true;
}

bool saveXml(xmlNodePtr _parent, const Rotation &_rot)
{
  std::ostringstream stream;
  stream << _rot;
  _parent->SetAttribute("rot", stream.str());
  return true;
}

bool saveXml(xmlNodePtr _parent, const ParamT<Pose,true> &_pose)
{
  xmlNodePtr originNode = new xmlNode(_pose.GetKey());
  _parent->LinkEndChild( originNode );

  saveXml(originNode, _pose.GetValue().position);
  saveXml(originNode, _pose.GetValue().rotation);

  return true;
}

bool saveXml(xmlNodePtr _parent, const ParamT<Pose,false> &_pose)
{
  xmlNodePtr originNode = new xmlNode(_pose.GetKey());
  _parent->LinkEndChild( originNode );

  std::ostringstream posStream, rotStream;

  posStream << _pose.GetValue().position;
  rotStream << _pose.GetValue().rotation;

  originNode->SetAttribute("xyz", posStream.str());
  originNode->SetAttribute("rpy", rotStream.str());

  return true;
}

bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Model> &_model)
{
  xmlNodePtr modelNode = new xmlNode("model");
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

bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Link> &_link)
{
  xmlNodePtr linkNode = new xmlNode("link");
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

bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Plugin> &_plugin)
{
  xmlNodePtr pluginNode = new xmlNode("plugin");
  _parent->LinkEndChild( pluginNode );
  pluginNode->SetAttribute(_plugin->name.GetKey(),
                           _plugin->name.GetAsString());
  pluginNode->SetAttribute(_plugin->filename.GetKey(),
                           _plugin->filename.GetAsString());

  return true;
}

bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Visual> &_visual)
{
  xmlNodePtr visualNode = new xmlNode("visual");
  _parent->LinkEndChild( visualNode );
  visualNode->SetAttribute( _visual->castShadows.GetKey(),
                            _visual->castShadows.GetAsString());
  visualNode->SetAttribute( _visual->name.GetKey(),
                            _visual->name.GetAsString());

  saveXml(visualNode, _visual->origin);
  saveXml(visualNode, _visual->geometry);
  saveXml(visualNode, _visual->material);

  return true;
}

bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Geometry> &_geom)
{
  xmlNodePtr shapeNode = NULL;
  xmlNodePtr geomNode = new xmlNode("geometry");
  _parent->LinkEndChild( geomNode );

  if (_geom->type == Geometry::SPHERE)
  {
    boost::shared_ptr<Sphere> shape = boost::shared_static_cast<Sphere>(_geom);
    shapeNode = new xmlNode("sphere");
    shapeNode->SetAttribute(shape->radius.GetKey(),shape->radius.GetAsString());
  }
  else if (_geom->type == Geometry::BOX)
  {
    boost::shared_ptr<Box> shape = boost::shared_static_cast<Box>(_geom);
    shapeNode = new xmlNode("box");
    std::ostringstream stream;
    stream << shape->size;
    shapeNode->SetAttribute("size", stream.str());
  }
  else if (_geom->type == Geometry::CYLINDER)
  {
    boost::shared_ptr<Cylinder> shape = boost::shared_static_cast<Cylinder>(_geom);
    shapeNode = new xmlNode("cylinder");
    shapeNode->SetAttribute(shape->radius.GetKey(),shape->radius.GetAsString());
    shapeNode->SetAttribute(shape->length.GetKey(),shape->length.GetAsString());
  }
  else if (_geom->type == Geometry::MESH)
  {
    boost::shared_ptr<Mesh> shape = boost::shared_static_cast<Mesh>(_geom);
    shapeNode = new xmlNode("mesh");
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

bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Material> &_mat)
{
  xmlNodePtr matNode = new xmlNode("material");
  _parent->LinkEndChild( matNode );
  matNode->SetAttribute( _mat->script.GetKey(), _mat->script.GetAsString());

  xmlNodePtr colorNode = new xmlNode("color");
  colorNode->SetAttribute( _mat->color.GetKey(), _mat->color.GetAsString());

  return true;
}

bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Collision> &_collision)
{
  xmlNodePtr collisionNode = new xmlNode("collision");
  _parent->LinkEndChild( collisionNode );
  collisionNode->SetAttribute( _collision->name.GetKey(),
                               _collision->name.GetAsString());

  saveXml(collisionNode, _collision->origin);
  saveXml(collisionNode, _collision->geometry);

  return true;
}

bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Sensor> &_sensor)
{
  xmlNodePtr sensorNode = new xmlNode("sensor");
  _parent->LinkEndChild( sensorNode );

  sensorNode->SetAttribute( _sensor->name.GetKey(),
                            _sensor->name.GetAsString() );
  sensorNode->SetAttribute( _sensor->type.GetKey(),
                            _sensor->type.GetAsString() );
  sensorNode->SetAttribute( _sensor->alwaysOn.GetKey(),
                            _sensor->alwaysOn.GetAsString() );
  sensorNode->SetAttribute( _sensor->updateRate.GetKey(),
                            _sensor->updateRate.GetAsString() );

  saveXml(sensorNode, _sensor->origin);

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

bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<CameraSensor> &_camera)
{
  xmlNodePtr hfovNode = new xmlNode("horizontal_fov");
  _parent->LinkEndChild( hfovNode );
  hfovNode->SetAttribute( _camera->horizontalFov.GetKey(),
                          _camera->horizontalFov.GetAsString() );

  xmlNodePtr imageNode = new xmlNode("image");
  _parent->LinkEndChild( imageNode );
  imageNode->SetAttribute( _camera->imageWidth.GetKey(),
                           _camera->imageWidth.GetAsString() );
  imageNode->SetAttribute( _camera->imageHeight.GetKey(),
                           _camera->imageHeight.GetAsString() );

  xmlNodePtr clipNode = new xmlNode("clip");
  _parent->LinkEndChild( clipNode );
  clipNode->SetAttribute( _camera->clipNear.GetKey(),
                          _camera->clipNear.GetAsString() );
  clipNode->SetAttribute( _camera->clipFar.GetKey(),
                          _camera->clipFar.GetAsString() );

  xmlNodePtr saveNode = new xmlNode("save");
  _parent->LinkEndChild( saveNode );
  saveNode->SetAttribute( _camera->saveEnabled.GetKey(),
                          _camera->saveEnabled.GetAsString() );
  saveNode->SetAttribute( _camera->savePath.GetKey(),
                          _camera->savePath.GetAsString() );

  return true;
}

bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<RaySensor> &_ray)
{
  xmlNodePtr scanNode = new xmlNode("scan");
  _parent->LinkEndChild( scanNode );
  scanNode->SetAttribute( _ray->display.GetKey(),
                          _ray->display.GetAsString() );

  xmlNodePtr horizontalNode = new xmlNode("horizontal");
  _parent->LinkEndChild( horizontalNode );
  horizontalNode->SetAttribute( _ray->horizontalSamples.GetKey(),
                                _ray->horizontalSamples.GetAsString() );
  horizontalNode->SetAttribute( _ray->horizontalResolution.GetKey(),
                                _ray->horizontalResolution.GetAsString() );
  horizontalNode->SetAttribute( _ray->horizontalMinAngle.GetKey(),
                                _ray->horizontalMinAngle.GetAsString() );
  horizontalNode->SetAttribute( _ray->horizontalMaxAngle.GetKey(),
                                _ray->horizontalMaxAngle.GetAsString() );

  xmlNodePtr verticalNode = new xmlNode("vertical");
  _parent->LinkEndChild( verticalNode );
  verticalNode->SetAttribute( _ray->verticalSamples.GetKey(),
                              _ray->verticalSamples.GetAsString() );
  verticalNode->SetAttribute( _ray->verticalResolution.GetKey(),
                              _ray->verticalResolution.GetAsString() );
  verticalNode->SetAttribute( _ray->verticalMinAngle.GetKey(),
                              _ray->verticalMinAngle.GetAsString() );
  verticalNode->SetAttribute( _ray->verticalMaxAngle.GetKey(),
                              _ray->verticalMaxAngle.GetAsString() );

  xmlNodePtr rangeNode = new xmlNode("range");
  _parent->LinkEndChild( rangeNode );
  rangeNode->SetAttribute( _ray->rangeMin.GetKey(),
                           _ray->rangeMin.GetAsString() );
  rangeNode->SetAttribute( _ray->rangeMax.GetKey(),
                           _ray->rangeMax.GetAsString() );
  rangeNode->SetAttribute( _ray->rangeResolution.GetKey(),
                           _ray->rangeResolution.GetAsString() );
  return true;
}

bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Inertial> &_inertial)
{
  xmlNodePtr inertialNode = new xmlNode("inertial");
  _parent->LinkEndChild( inertialNode );
  inertialNode->SetAttribute(_inertial->mass.GetKey(),
                             _inertial->mass.GetAsString());


  saveXml(inertialNode, _inertial->origin);

  xmlNodePtr inertiaNode = new xmlNode("inertia");
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
*/

}
