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
#include "sdf/interface/Param.hh"
#include "math/Pose.hh"
#include "math/Vector3.hh"
#include "math/Vector2d.hh"
#include "common/Color.hh"
#include "math/Quaternion.hh"


namespace deprecated_sdf
{

bool getPlugins(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{
  // Get all plugins 
  for (xmlNodePtr pluginXml = _config->xmlChildrenNode;
       pluginXml != NULL; pluginXml = pluginXml->next)
  {
    if (pluginXml->ns && 
        std::string("controller") == (const char*)pluginXml->ns->prefix)
    {
      sdf::ElementPtr sdfPlugin = _sdf->AddElement("plugin");
      initAttr(pluginXml, "name", sdfPlugin->GetAttribute("name"));

      if (xmlGetProp(pluginXml, (xmlChar*)"plugin"))
        initAttr(pluginXml, "plugin", sdfPlugin->GetAttribute("filename"));
      else
        sdfPlugin->GetAttribute("filename")->SetFromString((const char*)pluginXml->name);

      //gzdbg << "Getting plugin[" << sdfPlugin->GetAttribute("filename")->GetAsString() << "]\n";
      for (xmlNodePtr dataXml = pluginXml->xmlChildrenNode;
           dataXml != NULL; dataXml = dataXml->next)
      {
        if (!dataXml->ns || std::string( (const char*) dataXml->ns->prefix ) != "interface")
        {
          if (!getValue(dataXml).empty())
          {
            sdf::ElementPtr sdfData = sdfPlugin->AddElement("data");
            sdfData->GetAttribute("name")->SetFromString( (const char*)dataXml->name );
            sdfData->GetAttribute("value")->SetFromString( getValue(dataXml) );
          }
        }
      }
    }
    else if (std::string((const char*)pluginXml->name) == "plugin")
    {
      sdf::ElementPtr sdfPlugin = _sdf->AddElement("plugin");
      initAttr(pluginXml, "name", sdfPlugin->GetAttribute("name"));
      initAttr(pluginXml, "filename", sdfPlugin->GetAttribute("filename"));

      for (xmlNodePtr dataXml = pluginXml->xmlChildrenNode;
          dataXml != NULL; dataXml = dataXml->next)
      {
        if (std::string( (const char*) dataXml->ns->prefix ) != "interface")
        {
          sdf::ElementPtr sdfData = sdfPlugin->AddElement("data");
          sdfData->GetAttribute("value")->SetFromString( getValue(dataXml) );
        }
      }
    }
  }

  return true;
}

// light parsing
bool initLight(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{
  initOrigin(_config, _sdf);
  xmlNodePtr lightNode = firstChildElement(_config, "light");
  if (!lightNode)
  {
    gzerr << "Light is missing the <light> child node\n";
    return false;
  }

  initAttr(_config, "name", _sdf->GetAttribute("name"));
  if (firstChildElement(lightNode, "castShadows"))
    initAttr(lightNode, "castShadows", _sdf->GetAttribute("cast_shadows"));
  initAttr(lightNode, "type", _sdf->GetAttribute("type"));

  sdf::ElementPtr sdfDiffuse = _sdf->AddElement("diffuse");
  initAttr(lightNode, "diffuseColor", sdfDiffuse->GetAttribute("rgba"));

  sdf::ElementPtr sdfSpecular = _sdf->AddElement("specular");
  initAttr(lightNode, "specularColor", sdfDiffuse->GetAttribute("rgba"));

  sdf::ElementPtr sdfAttenuation = _sdf->AddElement("attenuation");
  initAttr(lightNode, "range", sdfAttenuation->GetAttribute("range"));
  sdfAttenuation->GetAttribute("constant")->SetFromString( getNodeTuple(lightNode, "attenuation",0));
  sdfAttenuation->GetAttribute("linear")->SetFromString( getNodeTuple(lightNode, "attenuation",1));
  sdfAttenuation->GetAttribute("quadratic")->SetFromString( getNodeTuple(lightNode, "attenuation",2));

  if (firstChildElement(lightNode,"sportCone"))
  {
    sdf::ElementPtr sdfSpot = _sdf->AddElement("spot");
    double innerAngle = boost::lexical_cast<double>(getNodeTuple(lightNode, "spotCone",0));
    double outerAngle = boost::lexical_cast<double>(getNodeTuple(lightNode, "spotCone",1));

    sdfSpot->GetAttribute("inner_angle")->SetFromString( boost::lexical_cast<std::string>(DTOR(innerAngle)) );
    sdfSpot->GetAttribute("outer_angle")->SetFromString( boost::lexical_cast<std::string>(DTOR(outerAngle)) );
    sdfSpot->GetAttribute("falloff")->SetFromString( getNodeTuple(lightNode, "spotCone",2));
  }

  return true;
}

// Sensor parsing
bool initSensor(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{
  initAttr(_config, "name", _sdf->GetAttribute("name"));
  initAttr(_config, "alwaysOn", _sdf->GetAttribute("always_on"));
  initAttr(_config, "updateRate", _sdf->GetAttribute("update_rate"));

  initOrigin(_config, _sdf);
 
  if ( std::string((const char*)_config->name) == "contact" )
  {
    sdf::ElementPtr contact = _sdf->AddElement("contact");
    initContact(_config, contact);
  }
  else if ( std::string((const char*)_config->name) == "camera" )
  {
    sdf::ElementPtr camera = _sdf->AddElement("camera");
    initCamera(_config, camera);
  }
  else if ( std::string((const char*)_config->name) == "camera" )
  {
    sdf::ElementPtr ray = _sdf->AddElement("ray");
    initRay(_config, ray);
  }

  /// Get all the plugins
  getPlugins(_config, _sdf);

  return true;
}

bool initCamera(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{

  sdf::ElementPtr sdfHFOV = _sdf->AddElement("horizontal_fov");
  initAttr(_config, "hfov", sdfHFOV->GetAttribute("angle"));

  sdf::ElementPtr sdfImage = _sdf->AddElement("image");
  initAttr(_config, "imageWidth", sdfImage->GetAttribute("width"));
  initAttr(_config, "imageHeight", sdfImage->GetAttribute("height"));
  initAttr(_config, "imageFormat", sdfImage->GetAttribute("format"));

  sdf::ElementPtr sdfClip = _sdf->AddElement("clip");
  initAttr(_config, "nearClip", sdfClip->GetAttribute("near"));
  initAttr(_config, "farClip", sdfClip->GetAttribute("far"));

  // save 
  if (firstChildElement(_config, "saveFrames"))
  {
    sdf::ElementPtr sdfSave = _sdf->AddElement("save");
    initAttr(_config, "saveFrames", sdfSave->GetAttribute("enabled"));
    initAttr(_config, "saveFramePath", sdfSave->GetAttribute("path"));
  }

  return true;
}

bool initRay(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{
  sdf::ElementPtr sdfScan = _sdf->AddElement("scan");

  if (firstChildElement(_config, "displayRays"))
  {
    std::string display = getNodeValue(_config, "displayRays");
    if (display != "false")
      sdfScan->GetAttribute("display")->SetFromString("false");
    else
      sdfScan->GetAttribute("display")->SetFromString("true");
  }

  sdf::ElementPtr sdfHoriz = sdfScan->AddElement("horizontal");

  initAttr(_config, "rangeCount", sdfHoriz->GetAttribute("samples"));

  int rangeCount = boost::lexical_cast<int>(getNodeValue(_config,"rangeCount"));
  int rayCount = boost::lexical_cast<int>(getNodeValue(_config, "rayCount"));

  if (!sdfHoriz->GetAttribute("resolution")->SetFromString( 
        boost::lexical_cast<std::string>(rangeCount / rayCount) ))
  {
    gzerr << "Unable to parse ray sensor rayCount\n";
    return false;
  }

  double minAngle = boost::lexical_cast<double>(getNodeValue(_config,"minAngle"));
  double maxAngle = boost::lexical_cast<double>(getNodeValue(_config,"maxAngle"));

  if (!sdfHoriz->GetAttribute("min_angle")->SetFromString( 
        boost::lexical_cast<std::string>(DTOR(minAngle)) ))
  {
    gzerr << "Unable to parse min_angle\n";
    return false;
  } 

  if (!sdfHoriz->GetAttribute("max_angle")->SetFromString( 
        boost::lexical_cast<std::string>(DTOR(maxAngle)) ))
  {
    gzerr << "Unable to parse max_angle\n";
    return false;
  } 

  sdf::ElementPtr sdfRange = _sdf->AddElement("range");
  initAttr(_config, "minRange", sdfRange->GetAttribute("min"));
  initAttr(_config, "maxRange", sdfRange->GetAttribute("max"));
  initAttr(_config, "resRange", sdfRange->GetAttribute("resolution"));

  return true;
}

bool initContact(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{
  sdf::ElementPtr sdfCollision = _sdf->AddElement("collision");

  initAttr(_config, "geom", sdfCollision->GetAttribute("name"));

  return true;
}

// _config = <body>
// _sdf = <inertial>
bool initInertial(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{
  // Origin (old gazebo xml supports only cx,cy,cz translations, no rotation
  // xyz and rpy under body:... is for the link frame
  xmlNodePtr cx_xml = firstChildElement(_config, "cx");
  xmlNodePtr cy_xml = firstChildElement(_config, "cy");
  xmlNodePtr cz_xml = firstChildElement(_config, "cz");

  std::string poseString;
  if (cx_xml) 
    poseString += getValue(cx_xml) + " ";
  else
  {
    gzerr << "Missing cx\n";
    return false;
  }

  if (cy_xml) 
    poseString += getValue(cy_xml) + " ";
  else
  {
    gzerr << "Missing cy\n";
    return false;
  }

  if (cz_xml) 
    poseString += getValue(cz_xml) + " ";
  else
  {
    gzerr << "Missing cz\n";
     return false;
  } 

  // Put in the rpy values
  poseString += "0 0 0";

  sdf::ElementPtr sdfOrigin = _sdf->AddElement("origin");
  sdfOrigin->GetAttribute("pose")->SetFromString(poseString);

  initAttr(_config, "mass", _sdf->GetAttribute("mass"));

  sdf::ElementPtr sdfInertia = _sdf->AddElement("inertia");

  xmlNodePtr ixx_xml = firstChildElement(_config, "ixx");
  xmlNodePtr ixy_xml = firstChildElement(_config, "ixy");
  xmlNodePtr ixz_xml = firstChildElement(_config, "ixz");
  xmlNodePtr iyy_xml = firstChildElement(_config, "iyy");
  xmlNodePtr iyz_xml = firstChildElement(_config, "iyz");
  xmlNodePtr izz_xml = firstChildElement(_config, "izz");
  if (!ixx_xml || !ixy_xml || !ixz_xml || !iyy_xml || !iyz_xml || !izz_xml) 
  {
    gzerr << "Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes\n";
    return false;
  }
  if (!sdfInertia->GetAttribute("ixx")->SetFromString( getValue(ixx_xml) ) ||
      !sdfInertia->GetAttribute("ixy")->SetFromString( getValue(ixy_xml) ) ||
      !sdfInertia->GetAttribute("ixz")->SetFromString( getValue(ixz_xml) ) ||
      !sdfInertia->GetAttribute("iyy")->SetFromString( getValue(iyy_xml) ) ||
      !sdfInertia->GetAttribute("iyz")->SetFromString( getValue(iyz_xml) ) ||
      !sdfInertia->GetAttribute("izz")->SetFromString( getValue(izz_xml) ) )
  {
    gzerr << "one of the inertia elements: "
      << "ixx (" << getValue(ixx_xml) << ") "
      << "ixy (" << getValue(ixy_xml) << ") "
      << "ixz (" << getValue(ixz_xml) << ") "
      << "iyy (" << getValue(iyy_xml) << ") "
      << "iyz (" << getValue(iyz_xml) << ") "
      << "izz (" << getValue(izz_xml) << ") "
      << "is not a valid double.\n";
    return false;
  }

  return true;
}

// _conifg = "geom"
// _sdf = "collision"
bool initCollision(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{
  initAttr(_config, "name", _sdf->GetAttribute("name"));

  // Origin
  initOrigin(_config, _sdf);

  sdf::ElementPtr sdfGeom = _sdf->AddElement("geometry");
  if (std::string((const char *)_config->name) == "plane")
  {
    sdf::ElementPtr sdfPlane = sdfGeom->AddElement("plane");
    sdfPlane->GetAttribute("normal")->SetFromString( getNodeValue(_config, "normal") );
  }
  else if (std::string((const char *)_config->name) == "box")
  {
    sdf::ElementPtr sdfBox = sdfGeom->AddElement("box");
    sdfBox->GetAttribute("size")->SetFromString( getNodeValue(_config, "size") );
  }
  else if (std::string((const char *)_config->name) == "sphere")
  {
    sdf::ElementPtr sdfSphere = sdfGeom->AddElement("sphere");
    sdfSphere->GetAttribute("radius")->SetFromString( getNodeValue(_config, "radius") );
  }
  else if (std::string((const char *)_config->name) == "cylinder")
  {
    sdf::ElementPtr sdfCylinder = sdfGeom->AddElement("cylinder");
    if (firstChildElement(_config,"size"))
    {
      sdfCylinder->GetAttribute("radius")->SetFromString( getNodeTuple(_config, "size",0) );
      sdfCylinder->GetAttribute("length")->SetFromString( getNodeTuple(_config, "size",1) );
    }
  }
  else if (std::string((const char *)_config->name) == "trimesh")
  {
    sdf::ElementPtr sdfMesh = sdfGeom->AddElement("mesh");
    sdfMesh->GetAttribute("filename")->SetFromString( getNodeValue(_config,"mesh") );

    initAttr(_config, "scale", sdfMesh->GetAttribute("scale"));
  }

  return true;
}

// _config = a node with an xyz and/or rpy children
// _sdf = an sdf element that has an origin child element
bool initOrigin(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{
  // Origin
  xmlNodePtr xyz_xml = firstChildElement(_config, "xyz");
  xmlNodePtr rpy_xml = firstChildElement(_config, "rpy");

  sdf::ElementPtr origin = _sdf->AddElement("origin");
  std::string poseStr;

  if (xyz_xml) 
    poseStr += getValue(xyz_xml) + " ";
  else
    poseStr += "0 0 0 ";

  if (rpy_xml) 
    poseStr += getValue(rpy_xml);
  else
    poseStr += "0 0 0";

  origin->GetAttribute("pose")->SetFromString( poseStr );

  return true;
}

// _config = <body>
// _sdf = <link>
bool initLink(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{
  initAttr(_config, "name", _sdf->GetAttribute("name"));
  initOrigin(_config, _sdf);

  // Inertial (optional)
  xmlNodePtr mm = firstChildElement(_config, "massMatrix");
  if (mm)
  {
    sdf::ParamT<bool> custom_mass_matrix("mass","false",false);
    custom_mass_matrix.SetFromString( getValue(mm).c_str() );
    if (custom_mass_matrix.GetValue())
    {
      sdf::ElementPtr sdfInertial = _sdf->AddElement("inertial");
      if (!initInertial(_config, sdfInertial))
      {
        gzerr << "Could not parse inertial element for Link '" 
          << _sdf->GetAttribute("name")->GetAsString() << "'\n";
        return false;
      }
    }
  }

  // Multiple Collisions (optional)
  for (xmlNodePtr  collision_xml = getChildByNSPrefix(_config, "geom"); 
      collision_xml; collision_xml = getNextByNSPrefix(collision_xml, "geom"))
  {
    sdf::ElementPtr sdfCollision = _sdf->AddElement("collision");
    if (!initCollision(collision_xml,sdfCollision))
    {
      gzerr << "Unable to parser geom\n";
      return false;
    }

    for (xmlNodePtr  visual_xml = firstChildElement(collision_xml, "visual"); 
        visual_xml; visual_xml = nextSiblingElement(visual_xml, "visual"))
    {
      sdf::ElementPtr sdfVisual = _sdf->AddElement("visual");

      if (!initVisual(visual_xml, sdfVisual))
      {
        gzerr << "Unable to parse visual\n";
        return false;
      }

      // In order to parse old gazebo xml (nested format)
      // to new sdf, we need to unwrap visual pose from within collision.
      // take origin of visual, multiply it by collision's transform
      gazebo::math::Pose col_pose = sdfCollision->GetElement("origin")->GetValuePose("pose");
      gazebo::math::Pose vis_pose = sdfVisual->GetElement("origin")->GetValuePose("pose");
      vis_pose = col_pose*vis_pose;
      // update the sdf pose
      sdfVisual->GetElement("origin")->GetAttribute("pose")->Set(vis_pose);
    }
    // TODO: check for duplicate geoms
  }

  // Get all sensor elements
  // FIXME: instead of child elements, get namespace == sensor blocks
  for (xmlNodePtr  sensorXml = firstChildElement(_config, "sensor"); 
      sensorXml; sensorXml = nextSiblingElement(sensorXml, "sensor"))
  {
    sdf::ElementPtr sdfSensor = _sdf->AddElement("sensor");
    if (!initSensor(sensorXml,sdfSensor))
    {
      gzerr << "Unable to parse sensor\n";
      return false;
    }
    // TODO: check for duplicate sensors
  }

  return true;
}

/// _config = <visual>
/// _sdf = visual
bool initVisual(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{
  _sdf->GetAttribute("name")->SetFromString("old_gazebo_xml_visual");
  _sdf->GetAttribute("cast_shadows")->SetFromString("true");

  initOrigin(_config, _sdf);

  sdf::ElementPtr sdfGeom = _sdf->AddElement("geometry");

  std::string mesh_attribute = getNodeValue(_config,"mesh");
  // check each mesh type
  if (mesh_attribute == "unit_box")
  {
    sdf::ElementPtr sdfBox = sdfGeom->AddElement("box");
    if (firstChildElement(_config,"scale"))
      sdfBox->GetAttribute("size")->SetFromString( getNodeValue(_config, "scale") );
    else
      sdfBox->GetAttribute("size")->SetFromString("1 1 1");
  }
  else if (mesh_attribute == "unit_sphere")
  {
    sdf::ElementPtr sdfSphere = sdfGeom->AddElement("sphere");
    if (firstChildElement(_config,"scale"))
      sdfSphere->GetAttribute("radius")->SetFromString( getNodeTuple(_config, "scale", 0) );
    else
      sdfSphere->GetAttribute("radius")->SetFromString( "1.0" );
  }
  else if (mesh_attribute == "unit_cylinder")
  {
    sdf::ElementPtr sdfCylinder = sdfGeom->AddElement("cylinder");

    if (firstChildElement(_config,"scale"))
    {
      sdfCylinder->GetAttribute("radius")->SetFromString( getNodeTuple(_config, "scale", 0) );
      sdfCylinder->GetAttribute("length")->SetFromString( getNodeTuple(_config, "scale", 1) );
    }
    else
    {
      sdfCylinder->GetAttribute("radius")->SetFromString( "1" );
      sdfCylinder->GetAttribute("length")->SetFromString( "1" );
    }
  }
  else if (!mesh_attribute.empty())
  {
    sdf::ElementPtr sdfMesh = sdfGeom->AddElement("mesh");
    sdfMesh->GetAttribute("filename")->SetFromString( mesh_attribute );

    if (firstChildElement(_config, "scale"))
      sdfMesh->GetAttribute("scale")->SetFromString( getNodeValue(_config,"scale") );
  }
  else
  {
    sdf::ElementPtr sdfPlane = sdfGeom->AddElement("plane");
    sdfPlane->GetAttribute("normal")->SetFromString( getNodeValue(_config, "normal") );
  }

  // Material
  xmlNodePtr mat_xml = firstChildElement(_config, "material");
  if (mat_xml)
  {
    sdf::ElementPtr sdfMat = _sdf->AddElement("material");
    initAttr(_config, "material", sdfMat->GetAttribute("script"));
  }

  return true;
}

// _config = <joint>
// _sdf = joint
bool initJoint(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{
  initAttr(_config, "name", _sdf->GetAttribute("name"));
  initOrigin(_config, _sdf);

  sdf::ElementPtr sdfParent = _sdf->AddElement("parent");
  sdf::ElementPtr sdfChild = _sdf->AddElement("child");

  // Get Parent Link
  // parent is specified by <anchor> element in old xml
  // once anchor is found, <body1> or <body2> are parsed
  // as child and parent
  if (!firstChildElement(_config, "anchor"))
  {
    gzerr << "No parent link specified for joint[" 
          << _sdf->GetAttribute("name")->GetAsString() << "]\n";
    return false;
  }
  initAttr(_config, "anchor", sdfChild->GetAttribute("link"));

  // Get Child Link
  xmlNodePtr body1Xml = firstChildElement(_config, "body1");
  xmlNodePtr body2Xml = firstChildElement(_config, "body2");
  if (body1Xml && body2Xml)
  {
    if (sdfChild->GetAttribute("link")->GetAsString() == getValue(body1Xml))
    {
      initAttr(_config, "body2", sdfParent->GetAttribute("link"));
    }
    else if (sdfChild->GetAttribute("link")->GetAsString() == getValue(body2Xml))
    {
      initAttr(_config, "body1", sdfParent->GetAttribute("link"));
    }
    else
    {
      gzerr << "body1 and body2 does not match anchor, not sure which one is parent.\n";
      return false;
    }
  }
  else
  {
    gzerr << "No child link specified for joint["
          << _sdf->GetAttribute("name")->GetAsString() << "]\n";
    return false;
  }

  if (std::string((const char*)_config->name) == "hinge")
    _sdf->GetAttribute("type")->SetFromString("revolute");
  else if (std::string((const char*)_config->name) == "hinge2")
    _sdf->GetAttribute("type")->SetFromString("revolute2");
  else if (std::string((const char*)_config->name) == "slider")
    _sdf->GetAttribute("type")->SetFromString("prismatic");
  else if (std::string((const char*)_config->name) == "ball")
    _sdf->GetAttribute("type")->SetFromString("ball");
  else if (std::string((const char*)_config->name) == "universal")
    _sdf->GetAttribute("type")->SetFromString("universal");
  else
    gzerr << "Unknown joint type[" << (const char*)_config->name << "]\n";

  if ( firstChildElement(_config,"axis"))
  {
    sdf::ElementPtr sdfAxis = _sdf->AddElement("axis");
    initAttr(_config, "axis", sdfAxis->GetAttribute("xyz"));

    sdf::ElementPtr sdfDynamics = sdfAxis->AddElement("dynamics");
    if (firstChildElement(_config, "damping"))
    {
      initAttr(_config, "damping", sdfDynamics->GetAttribute("damping"));
    }

    sdf::ElementPtr sdfLimit = sdfAxis->AddElement("limit");

    // Get limit
    if (firstChildElement(_config, "lowStop"))
    {
      double stop_angle = boost::lexical_cast<double>(getNodeValue(_config,"lowStop"));
      sdfLimit->GetAttribute("lower")->Set(DTOR(stop_angle));
    }
    if (firstChildElement(_config, "highStop"))
    {
      double stop_angle = boost::lexical_cast<double>(getNodeValue(_config,"highStop"));
      sdfLimit->GetAttribute("upper")->Set(DTOR(stop_angle));
    }

  }

  if ( firstChildElement(_config,"axis2"))
  {
    sdf::ElementPtr sdfAxis = _sdf->AddElement("axis2");
    initAttr(_config, "axis", sdfAxis->GetAttribute("xyz"));

    sdf::ElementPtr sdfDynamics = sdfAxis->AddElement("dynamics");
    if (firstChildElement(_config, "damping"))
    {
      initAttr(_config, "damping", sdfDynamics->GetAttribute("damping"));
    }

    sdf::ElementPtr sdfLimit = sdfAxis->AddElement("limit");

    // Get limit
    if (firstChildElement(_config, "lowStop"))
    {
      double stop_angle = boost::lexical_cast<double>(getNodeValue(_config,"lowStop"));
      sdfLimit->GetAttribute("lower")->Set(DTOR(stop_angle));
    }
    if (firstChildElement(_config, "highStop"))
    {
      double stop_angle = boost::lexical_cast<double>(getNodeValue(_config,"highStop"));
      sdfLimit->GetAttribute("upper")->Set(DTOR(stop_angle));
    }

  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Load Model from xmlNode
bool initModel(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{

  initAttr(_config, "name", _sdf->GetAttribute("name"));
  initAttr(_config, "static", _sdf->GetAttribute("static"));
  initOrigin(_config, _sdf);

  // Get all Link elements
  for (xmlNodePtr  linkXml = getChildByNSPrefix(_config, "body"); 
      linkXml; linkXml = getNextByNSPrefix(linkXml, "body"))
  {
    sdf::ElementPtr sdfLink =_sdf->AddElement("link"); 
    if (!initLink(linkXml, sdfLink))
    {
      gzerr << "link xml is not initialized correctly\n";
      return false;
    }
  }

  // Get all Joint elements
  for (xmlNodePtr  jointXml = getChildByNSPrefix(_config, "joint"); 
       jointXml; jointXml = getNextByNSPrefix(jointXml, "joint"))
  {
    sdf::ElementPtr sdfJoint =_sdf->AddElement("joint"); 
    if (!initJoint(jointXml, sdfJoint))
    {
      gzerr << "joint xml is not initialized correctly\n";
      return false;
    }
  }

  /// Get all the plugins
  getPlugins(_config, _sdf);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool initWorld(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{
  // Set world name
  if (!_sdf->GetAttribute("name")->SetFromString( "old_gazebo_xml_deprecated" ))
  {
    gzerr << "Unable to set world name\n";
    return false;
  }

  sdf::ElementPtr sdfScene = _sdf->AddElement("scene");
  initScene(firstChildElement(_config, "ogre"), sdfScene);

  sdf::ElementPtr sdfPhysics = _sdf->AddElement("physics");
  initPhysics(firstChildElement(_config, "ode"),sdfPhysics);

  // Get all model elements
  for (xmlNodePtr  modelXml = getChildByNSPrefix(_config, "model"); 
       modelXml; modelXml = getNextByNSPrefix(modelXml, "model"))
  {

    if (strcmp((const char*)modelXml->name, "renderable")==0)
    {
      sdf::ElementPtr sdfLight = _sdf->AddElement("light");
      if (!initLight(modelXml, sdfLight))
      {
        gzerr << "light xml is not initialized correctly\n";
        return false;
      }
    }
    else 
    {
      sdf::ElementPtr sdfModel = _sdf->AddElement("model");
      if (!initModel(modelXml,sdfModel))
      {
        gzerr << "model xml is not initialized correctly\n";
        return false;
      }
    }
  }

  /// Get all the plugins
  getPlugins(_config, _sdf);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool initScene(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{

  sdf::ElementPtr sdfAmbient = _sdf->AddElement("ambient");
  initAttr(_config, "ambient", sdfAmbient->GetAttribute("rgba"));

  sdf::ElementPtr sdfBackground = _sdf->AddElement("background");
  initAttr(_config, "background", sdfBackground->GetAttribute("rgba"));

  xmlNodePtr sky = firstChildElement(_config, "sky");
  if (sky)
  {
    sdf::ElementPtr sdfSky = sdfBackground->AddElement("sky");
    initAttr(sky, "material", sdfSky->GetAttribute("material"));
  }

  sdf::ElementPtr sdfShadow = _sdf->AddElement("shadows");

  initAttr(_config, "shadows", sdfShadow->GetAttribute("enabled"));

  //  per pixel shading does not allow options
  //sdfShadow->GetAttribute("rgba")->SetFromString("0 0 0 0");
  //initAttr(_config, "shadowTechnique", sdfShadow->GetAttribute("type"));

  return true;
}

//_config = physics:ode
//_sdf = physics
bool initPhysics(xmlNodePtr _config, sdf::ElementPtr &_sdf)
{
  _sdf->GetAttribute("type")->SetFromString("ode");

  sdf::ElementPtr sdfGravity = _sdf->AddElement("gravity");
  sdf::ElementPtr sdfODE = _sdf->AddElement("ode");
  sdf::ElementPtr sdfODESolver = sdfODE->AddElement("solver");
  sdf::ElementPtr sdfODEConstraints = sdfODE->AddElement("constraints");

  initAttr(_config, "gravity", sdfGravity->GetAttribute("xyz"));

  initAttr(_config, "stepType", sdfODESolver->GetAttribute("type"));
  initAttr(_config, "stepTime", sdfODESolver->GetAttribute("dt"));
  

  if (sdfODESolver->GetAttribute("type")->GetAsString() == "quick")
  {
    initAttr(_config, "stepIters", sdfODESolver->GetAttribute("iters"));
    initAttr(_config, "stepW", sdfODESolver->GetAttribute("sor"));
  }


  // Contraints
  initAttr(_config, "cfm", sdfODEConstraints->GetAttribute("cfm"));
  initAttr(_config, "erp", sdfODEConstraints->GetAttribute("erp"));
  initAttr(_config, "contactMaxCorrectingVel", 
      sdfODEConstraints->GetAttribute("contact_max_correcting_vel"));
  initAttr(_config, "contactSurfaceLayer", 
      sdfODEConstraints->GetAttribute("contact_surface_layer"));

  return true;
}


bool initAttr(xmlNodePtr _node, const std::string _key, 
              sdf::ParamPtr _attr)
{
  if (_node)
  {
    std::string value = getNodeValue(_node, _key);
    if (value.empty())
    {
      gzwarn << "Node[" << _node->name << "] Has empty key value[" 
            << _key << "]\n";
      return false;
    }
    if (!_attr->SetFromString( value )) 
    {
      gzerr << "Unable to set attribute from  node[" 
            << _node->name << "] and key[" << _key << "]\n";
      return false;
    }
  }
  else
  {
    gzerr << "Unable to get attribute. Node is null\n";
    return false;
  }

  return true;
}
////////////////////////////////////////////////////////////////////////////////
bool initModelFile(const std::string &_filename, sdf::ElementPtr &_sdf)
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

  initModelString(output, _sdf);
  return true;
}


////////////////////////////////////////////////////////////////////////////////
bool initModelString(const std::string &_xmlString, sdf::ElementPtr &_sdf)
{
  xmlDocPtr xmlDoc = xmlParseDoc((xmlChar*)_xmlString.c_str());

  return initModelDoc(xmlDoc, _sdf);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Load Model from xmlDoc
bool initModelDoc(xmlDocPtr _xmlDoc, sdf::ElementPtr &_sdf)
{
  if (!_xmlDoc)
  {
    gzerr << "Could not parse the xml\n";
    return false;
  }

  xmlNodePtr modelXml = firstChildElement(xmlDocGetRootElement(_xmlDoc), "model");
  if (!modelXml)
  {
    gzerr << "Could not find the 'model' element in the xml file\n";
    return false;
  }

  return initModel(modelXml, _sdf);
}


////////////////////////////////////////////////////////////////////////////////
bool initWorldFile(const std::string &_filename, 
    sdf::SDFPtr &_sdf)
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

  initWorldString(output, _sdf);
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool initWorldString(const std::string &_xmlString, 
                     sdf::SDFPtr &_sdf)
{
  xmlDocPtr xmlDoc = xmlParseDoc((xmlChar*)_xmlString.c_str());

  return initWorldDoc(xmlDoc,_sdf);
}

////////////////////////////////////////////////////////////////////////////////
bool initWorldDoc(xmlDocPtr _xmlDoc, sdf::SDFPtr &_sdf)
{
  if (!_xmlDoc)
  {
    gzerr << "Could not parse the xml\n";
    return false;
  }

  // Old world files do not have a the <gazebo> ... </gazebo> structure
  _sdf->root->GetAttribute("version")->SetFromString("1.0");

  xmlNodePtr worldXml = firstChildElement(_xmlDoc, "world" );
  while (worldXml)
  {
    sdf::ElementPtr world = _sdf->root->AddElement("world");
    initWorld(worldXml, world);

    worldXml = nextSiblingElement(worldXml,"world");
  }

  return true;
}

}
