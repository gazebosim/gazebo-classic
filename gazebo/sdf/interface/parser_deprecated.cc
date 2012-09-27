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
#include "sdf/interface/parser_deprecated.hh"
#include "sdf/interface/Param.hh"
#include "math/Pose.hh"
#include "math/Vector3.hh"
#include "math/Vector2d.hh"
#include "common/Color.hh"
#include "math/Quaternion.hh"


namespace deprecated_sdf
{
void copyBlockChildren(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  // Iterate over all the child elements
  for (xmlNodePtr elemXml = xmlFirstElementChild(_config);
       elemXml != NULL; elemXml = xmlNextElementSibling(elemXml))
  {
    // copy name (skip prefixed elements if exist)?
    std::string prefix;
    if (elemXml->ns) prefix = (const char*)elemXml->ns->prefix;
    if (prefix.empty())
    {
      std::string elem_name((const char*)elemXml->name);
      if (_sdf->HasElementDescription(elem_name))
      {
        // gzdbg << "has element [" << elem_name << "] defined, copying.\n";
        sdf::ElementPtr element = _sdf->AddElement(elem_name);

        // copy attributes
        for (xmlAttrPtr attrXml = elemXml->properties;
             attrXml && attrXml->name && attrXml->children;
             attrXml = attrXml->next)
        {
          initAttr(elemXml, (const char*)attrXml->name,
              element->GetAttribute((const char*)attrXml->name));
        }
        // copy value
        std::string value = getValue(elemXml);
        if (!value.empty())
            element->GetValue()->SetFromString(value);
      }
      else
      {
        // gzdbg << "undefined element [" << elem_name
        //       << "], copy elements as string type.\n";
        sdf::ElementPtr element(new sdf::Element);
        element->SetParent(_sdf);
        element->SetName((const char*)elemXml->name);

        // copy attributes
        for (xmlAttrPtr attrXml = elemXml->properties;
             attrXml && attrXml->name && attrXml->children;
             attrXml = attrXml->next)
        {
          element->AddAttribute((const char*)attrXml->name, "string",
              "defaultvalue", false);
          initAttr(elemXml, (const char*)attrXml->name,
              element->GetAttribute((const char*)attrXml->name));
        }

        // copy value
        std::string value = getValue(elemXml);
        if (!value.empty())
            element->AddValue("string", value, "1");

        _sdf->InsertElement(element);
      }
    }
    else
    {
      // gzwarn << "skipped: deprecated prefixed element ["
      //        << prefix << ":"
      //        << (const char*)elemXml->name << "] is not used.\n";
      // sdf::ElementPtr element(new sdf::Element);
      // element->SetParent(_sdf);
      // element->SetName(prefix + ":" + (const char*)elemXml->name);
    }
  }
}

bool controller2Plugins(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  // Get all controllers and convert to plugins
  for (xmlNodePtr pluginXml = _config->xmlChildrenNode;
       pluginXml != NULL; pluginXml = pluginXml->next)
  {
    if (pluginXml->ns &&
         (const char*)pluginXml->ns->prefix == std::string("controller"))
    {
      sdf::ElementPtr sdfPlugin = _sdf->AddElement("plugin");
      initAttr(pluginXml, "name", sdfPlugin->GetAttribute("name"));

      if (xmlGetProp(pluginXml, reinterpret_cast<const xmlChar*>("plugin")))
        initAttr(pluginXml, "plugin", sdfPlugin->GetAttribute("filename"));
      else
        // sdfPlugin->GetAttribute("filename")->SetFromString(
        // (const char*)pluginXml->name);
        initAttr(pluginXml, "filename", sdfPlugin->GetAttribute("filename"));

      deprecated_sdf::copyBlockChildren(pluginXml, sdfPlugin);
    }
  }

  return true;
}

bool getProjectors(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  // Get all projectors
  for (xmlNodePtr projectorXml = _config->xmlChildrenNode;
       projectorXml != NULL; projectorXml = projectorXml->next)
  {
    if (projectorXml->name &&
        std::string((const char*)projectorXml->name) == "projector")
    {
      sdf::ElementPtr sdfProjector = _sdf->AddElement("projector");
      initAttr(projectorXml, "name", sdfProjector->GetAttribute("name"));

      deprecated_sdf::copyBlockChildren(projectorXml, sdfProjector);
    }
  }

  return true;
}

bool getGrippers(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  // Get all grippers
  for (xmlNodePtr gripperXml = _config->xmlChildrenNode;
       gripperXml != NULL; gripperXml = gripperXml->next)
  {
    if (gripperXml->name &&
        std::string((const char*)gripperXml->name) == "gripper")
    {
      sdf::ElementPtr sdfGripper = _sdf->AddElement("gripper");
      initAttr(gripperXml, "name", sdfGripper->GetAttribute("name"));

      deprecated_sdf::copyBlockChildren(gripperXml, sdfGripper);
    }
  }

  return true;
}

// light parsing
bool initLight(xmlNodePtr _config, sdf::ElementPtr _sdf)
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
  initAttr(lightNode, "diffuse", sdfDiffuse->GetAttribute("rgba"));

  sdf::ElementPtr sdfSpecular = _sdf->AddElement("specular");
  initAttr(lightNode, "specular", sdfDiffuse->GetAttribute("rgba"));

  sdf::ElementPtr sdfAttenuation = _sdf->AddElement("attenuation");
  initAttr(lightNode, "range", sdfAttenuation->GetAttribute("range"));
  sdfAttenuation->GetAttribute("constant")->SetFromString(
      getNodeTuple(lightNode, "attenuation", 0));
  sdfAttenuation->GetAttribute("linear")->SetFromString(
      getNodeTuple(lightNode, "attenuation", 1));
  sdfAttenuation->GetAttribute("quadratic")->SetFromString(
      getNodeTuple(lightNode, "attenuation", 2));

  sdf::ElementPtr sdfDirection = _sdf->AddElement("direction");
  initAttr(lightNode, "direction", sdfDirection->GetAttribute("xyz"));

  if (firstChildElement(lightNode, "sportCone"))
  {
    sdf::ElementPtr sdfSpot = _sdf->AddElement("spot");
    double innerAngle =
      boost::lexical_cast<double>(getNodeTuple(lightNode, "spotCone", 0));
    double outerAngle =
      boost::lexical_cast<double>(getNodeTuple(lightNode, "spotCone", 1));

    sdfSpot->GetAttribute("inner_angle")->SetFromString(
        boost::lexical_cast<std::string>(GZ_DTOR(innerAngle)));
    sdfSpot->GetAttribute("outer_angle")->SetFromString(
        boost::lexical_cast<std::string>(GZ_DTOR(outerAngle)));
    sdfSpot->GetAttribute("falloff")->SetFromString(
        getNodeTuple(lightNode, "spotCone", 2));
  }

  return true;
}

// Sensor parsing
bool initSensor(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  initAttr(_config, "name", _sdf->GetAttribute("name"));
  initAttr(_config, "alwaysOn", _sdf->GetAttribute("always_on"));
  initAttr(_config, "updateRate", _sdf->GetAttribute("update_rate"));

  initOrigin(_config, _sdf);

  if (std::string((const char*)_config->name) == "contact")
  {
    sdf::ElementPtr contact = _sdf->AddElement("contact");
    initContact(_config, contact);

    if (!_sdf->GetAttribute("type")->SetFromString("contact"))
    {
      gzerr << "Unable to set type to contact\n";
      return false;
    }
  }
  else if (std::string((const char*)_config->name) == "camera")
  {
    sdf::ElementPtr camera = _sdf->AddElement("camera");
    initCamera(_config, camera);

    // convert all camera to depth cameras so we can get point cloud if needed
    if (!_sdf->GetAttribute("type")->SetFromString("depth"))
    {
      gzerr << "Unable to set type to camera\n";
      return false;
    }
  }
  else if (std::string((const char*)_config->name) == "ray")
  {
    sdf::ElementPtr ray = _sdf->AddElement("ray");
    initRay(_config, ray);

    if (!_sdf->GetAttribute("type")->SetFromString("ray"))
    {
      gzerr << "Unable to set type to ray\n";
      return false;
    }
  }

  /// Get all the plugins
  controller2Plugins(_config, _sdf);

  return true;
}

bool initCamera(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  sdf::ElementPtr sdfHFOV = _sdf->AddElement("horizontal_fov");
  double hfov = boost::lexical_cast<double>(getNodeValue(_config, "hfov"));
  if (!sdfHFOV->GetAttribute("angle")->SetFromString(
        boost::lexical_cast<std::string>(GZ_DTOR(hfov))))
  {
    gzerr << "Unable to parse hfov angle\n";
    return false;
  }

  sdf::ElementPtr sdfImage = _sdf->AddElement("image");

  // parse imageSize
  std::string image_size_str = getNodeValue(_config, "imageSize");
  std::vector<unsigned int> sizes;
  std::vector<std::string> pieces;
  boost::split(pieces, image_size_str, boost::is_any_of(" "));
  for (unsigned int i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        sizes.push_back(boost::lexical_cast<unsigned int>(pieces[i].c_str()));
      }
      catch(boost::bad_lexical_cast &e)
      {
        gzerr << "<imageSize> value ["
              << pieces[i] << "] is not a valid unsigned int from a 2-tuple\n";
        return false;
      }
    }
  }

  if (sizes.size() != 2)
  {
    gzerr << "Vector contains [" << static_cast<int>(sizes.size())
          << "] elements instead of 3 elements\n";
    return false;
  }

  sdfImage->GetAttribute("width")->SetFromString(pieces[0]);
  sdfImage->GetAttribute("height")->SetFromString(pieces[1]);


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

bool initRay(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  sdf::ElementPtr sdfScan = _sdf->AddElement("scan");

  /* FIXME: moved up to sensor/visualize attribute
  if (firstChildElement(_config, "displayRays"))
  {
    std::string display = getNodeValue(_config, "displayRays");
    if (display != "false")
      sdfScan->GetAttribute("display")->SetFromString("false");
    else
      sdfScan->GetAttribute("display")->SetFromString("true");
  }
  */

  sdf::ElementPtr sdfHoriz = sdfScan->AddElement("horizontal");
  sdf::ElementPtr sdfVerti = sdfScan->AddElement("vertical");

  initAttr(_config, "rangeCount", sdfHoriz->GetAttribute("samples"));
  initAttr(_config, "verticalRangeCount", sdfVerti->GetAttribute("samples"));

  int rangeCount =
    boost::lexical_cast<int>(getNodeValue(_config, "rangeCount"));
  int rayCount = boost::lexical_cast<int>(getNodeValue(_config, "rayCount"));

  if (!sdfHoriz->GetAttribute("resolution")->SetFromString(
        boost::lexical_cast<std::string>(rangeCount / rayCount)))
  {
    gzerr << "Unable to parse ray sensor rayCount\n";
    return false;
  }

  try
  {
    int verticalRangeCount =
      boost::lexical_cast<int>(getNodeValue(_config, "verticalRangeCount"));
    int verticalRayCount = boost::lexical_cast<int>(getNodeValue(_config,
          "verticalRayCount"));

    if (!sdfVerti->GetAttribute("resolution")->SetFromString(
          boost::lexical_cast<std::string>(verticalRangeCount /
                                           verticalRayCount)))
    {
      gzerr << "Unable to parse ray sensor verticalRayCount";
      return false;
    }
  }
  catch(boost::bad_lexical_cast& e)
  {
    gzerr << "verticalRayCount not parsable\n";
  }

  double minAngle =
    boost::lexical_cast<double>(getNodeValue(_config, "minAngle"));
  double maxAngle =
    boost::lexical_cast<double>(getNodeValue(_config, "maxAngle"));

  if (!sdfHoriz->GetAttribute("min_angle")->SetFromString(
        boost::lexical_cast<std::string>(GZ_DTOR(minAngle))))
  {
    gzerr << "Unable to parse min_angle\n";
    return false;
  }

  if (!sdfHoriz->GetAttribute("max_angle")->SetFromString(
        boost::lexical_cast<std::string>(GZ_DTOR(maxAngle))))
  {
    gzerr << "Unable to parse max_angle\n";
    return false;
  }

  try
  {
    double verticalMinAngle =
      boost::lexical_cast<double>(getNodeValue(_config, "verticalMinAngle"));
    double verticalMaxAngle =
      boost::lexical_cast<double>(getNodeValue(_config, "verticalMaxAngle"));

    if (!sdfVerti->GetAttribute("min_angle")->SetFromString(
          boost::lexical_cast<std::string>(GZ_DTOR(verticalMinAngle))))
    {
      gzerr << "Unable to parse vertical min_angle\n";
      return false;
    }

    if (!sdfVerti->GetAttribute("max_angle")->SetFromString(
          boost::lexical_cast<std::string>(GZ_DTOR(verticalMaxAngle))))
    {
      gzerr << "Unable to parse vertical max_angle\n";
      return false;
    }
  }
  catch(boost::bad_lexical_cast& e)
  {
    gzerr << "max_angle not parsable\n";
  }

  sdf::ElementPtr sdfRange = _sdf->AddElement("range");
  initAttr(_config, "minRange", sdfRange->GetAttribute("min"));
  initAttr(_config, "maxRange", sdfRange->GetAttribute("max"));
  initAttr(_config, "resRange", sdfRange->GetAttribute("resolution"));

  return true;
}

bool initContact(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  sdf::ElementPtr sdfCollision = _sdf->AddElement("collision");

  initAttr(_config, "geom", sdfCollision->GetAttribute("name"));

  return true;
}

// _config = <body>
// _sdf = <inertial>
bool initInertial(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  // Origin (old gazebo xml supports only cx, cy, cz translations, no rotation
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
    gzerr << "Inertial: inertia element must have ixx,"
          << " ixy, ixz, iyy, iyz, izz attributes\n";
    return false;
  }
  if (!sdfInertia->GetAttribute("ixx")->SetFromString(getValue(ixx_xml)) ||
      !sdfInertia->GetAttribute("ixy")->SetFromString(getValue(ixy_xml)) ||
      !sdfInertia->GetAttribute("ixz")->SetFromString(getValue(ixz_xml)) ||
      !sdfInertia->GetAttribute("iyy")->SetFromString(getValue(iyy_xml)) ||
      !sdfInertia->GetAttribute("iyz")->SetFromString(getValue(iyz_xml)) ||
      !sdfInertia->GetAttribute("izz")->SetFromString(getValue(izz_xml)))
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
bool initCollision(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  initAttr(_config, "name", _sdf->GetAttribute("name"));

  // Origin
  initOrigin(_config, _sdf);

  sdf::ElementPtr sdfGeom = _sdf->AddElement("geometry");
  if (std::string((const char *)_config->name) == "plane")
  {
    sdf::ElementPtr sdfPlane = sdfGeom->AddElement("plane");
    sdfPlane->GetAttribute("normal")->SetFromString(
        getNodeValue(_config, "normal"));
  }
  else if (std::string((const char *)_config->name) == "box")
  {
    sdf::ElementPtr sdfBox = sdfGeom->AddElement("box");
    sdfBox->GetAttribute("size")->SetFromString(getNodeValue(_config, "size"));
  }
  else if (std::string((const char *)_config->name) == "sphere")
  {
    sdf::ElementPtr sdfSphere = sdfGeom->AddElement("sphere");
    sdfSphere->GetAttribute("radius")->SetFromString(
        getNodeValue(_config, "size"));
  }
  else if (std::string((const char *)_config->name) == "cylinder")
  {
    sdf::ElementPtr sdfCylinder = sdfGeom->AddElement("cylinder");
    if (firstChildElement(_config, "size"))
    {
      sdfCylinder->GetAttribute("radius")->SetFromString(
          getNodeTuple(_config, "size", 0));
      sdfCylinder->GetAttribute("length")->SetFromString(
          getNodeTuple(_config, "size", 1));
    }
  }
  else if (std::string((const char *)_config->name) == "trimesh")
  {
    sdf::ElementPtr sdfMesh = sdfGeom->AddElement("mesh");
    sdfMesh->GetAttribute("filename")->SetFromString(
        getNodeValue(_config, "mesh"));

    initAttr(_config, "scale", sdfMesh->GetAttribute("scale"));
  }

  //
  // TODO: parse surface properties
  //
  sdf::ElementPtr sdfSurface = _sdf->AddElement("surface");
  // friction ode has mu, mu2, fdir1, slip1, slip2 attributes
  sdf::ElementPtr sdfSurfaceFriction = sdfSurface->AddElement("friction");
  sdf::ElementPtr sdfSurfaceFrictionOde = sdfSurfaceFriction->AddElement("ode");
  // mu1 --> mu
  initAttr(_config, "mu1", sdfSurfaceFrictionOde->GetAttribute("mu"));
  // mu2 --> mu2
  initAttr(_config, "mu2", sdfSurfaceFrictionOde->GetAttribute("mu2"));
  // fdir1 --> fdir1
  initAttr(_config, "fdir1", sdfSurfaceFrictionOde->GetAttribute("fdir1"));
  // slip1 --> slip1
  initAttr(_config, "slip1", sdfSurfaceFrictionOde->GetAttribute("slip1"));
  // slip2 --> slip2
  initAttr(_config, "slip2", sdfSurfaceFrictionOde->GetAttribute("slip2"));

  // bounce has restitution_coefficient and threshold attributes
  sdf::ElementPtr sdfSurfaceBounce = sdfSurface->AddElement("bounce");
  // bounce --> restitution_coefficient
  initAttr(_config, "bounce", sdfSurfaceBounce->GetAttribute(
        "restitution_coefficient"));
  // bounceVel --> threshold
  initAttr(_config, "bounceVel", sdfSurfaceBounce->GetAttribute("threshold"));

  // contact ode has soft_cfm, kp, kd, max_vel, min_depth attributes
  sdf::ElementPtr sdfSurfaceContact = sdfSurface->AddElement("contact");
  sdf::ElementPtr sdfSurfaceContactOde = sdfSurfaceContact->AddElement("ode");
  // kp --> kp
  initAttr(_config, "kp", sdfSurfaceContactOde->GetAttribute("kp"));
  // kd --> kd
  initAttr(_config, "kd", sdfSurfaceContactOde->GetAttribute("kd"));
  // softCFM --> soft_cfm
  initAttr(_config, "softCFM", sdfSurfaceContactOde->GetAttribute("soft_cfm"));
  // maxVel --> max_vel
  initAttr(_config, "maxVel", sdfSurfaceContactOde->GetAttribute("max_vel"));
  // minDepth --> min_depth
  initAttr(_config, "minDepth",
      sdfSurfaceContactOde->GetAttribute("min_depth"));

  return true;
}

// _config = a node with an xyz and/or rpy children
// _sdf = an sdf element that has an origin child element
bool initOrigin(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  // goes from _config
  //   <xyz>0 0 0</xyz>
  //   <rpy>0 0 0</rpy>
  // to _sdf
  //   <origin pose="0 0 0 0 0 0" />

  // Origin
  xmlNodePtr xyz_xml = firstChildElement(_config, "xyz");

  // parse xyz
  sdf::ElementPtr origin = _sdf->AddElement("origin");
  std::string poseStr;

  if (xyz_xml)
    poseStr += getValue(xyz_xml) + " ";
  else
    poseStr += "0 0 0 ";

  // parse rpy
  xmlNodePtr rpy_xml = firstChildElement(_config, "rpy");
  if (rpy_xml != NULL)
  {
    std::string rpy_str = getNodeValue(rpy_xml, "rpy");
    std::vector<double> degrees;
    std::vector<std::string> pieces;
    boost::split(pieces, rpy_str, boost::is_any_of(" "));
    for (unsigned int i = 0; i < pieces.size(); ++i)
    {
      if (pieces[i] != "")
      {
        try
        {
          degrees.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
        }
        catch(boost::bad_lexical_cast &e)
        {
          gzerr << "rpy [" << rpy_str << "][" << i << "] value ["
                << pieces[i] << "] is not a valid double from a 3-tuple\n";
          return false;
        }
      }
    }

    if (degrees.empty())
    {
      poseStr += "0 0 0";
    }
    else
    {
      if (degrees.size() != 3)
      {
        gzerr << "Vector contains [" << static_cast<int>(degrees.size())
              << "] elements instead of 3 elements\n";
        return false;
      }
      // convert degrees to radian
      std::ostringstream rpy_stream;
      rpy_stream << GZ_DTOR(degrees[0]) << " "
                 << GZ_DTOR(degrees[1]) << " "
                 << GZ_DTOR(degrees[2]);
      if (rpy_stream.str().empty())
      {
        gzerr << "rpy_stream is empty, something is wrong\n";
        return false;
      }
      poseStr += rpy_stream.str();
    }
  }
  else
      poseStr += "0 0 0";

  origin->GetAttribute("pose")->SetFromString(poseStr);

  return true;
}

// _config = <body>
// _sdf = <link>
bool initLink(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  initAttr(_config, "name", _sdf->GetAttribute("name"));
  initOrigin(_config, _sdf);

  // optional features (turnGravityOff, selfCollide)
  initAttr(_config, "selfCollide", _sdf->GetAttribute("self_collide"));

  // kind of tricky, new attribute gravity is opposite of turnGravityOff
  initAttr(_config, "", _sdf->GetAttribute("gravity"));
  xmlNodePtr tgo = firstChildElement(_config, "turnGravityOff");
  if (tgo)
  {
    sdf::ParamT<bool> tgoP("turnGravity", "false", false);
    tgoP.SetFromString(getValue(tgo).c_str());
    _sdf->GetAttribute("gravity")->Set(!tgoP.GetValue());
  }

  // Inertial (optional)
  xmlNodePtr mm = firstChildElement(_config, "massMatrix");
  if (mm)
  {
    sdf::ParamT<bool> custom_mass_matrix("mass", "false", false);
    custom_mass_matrix.SetFromString(getValue(mm).c_str());
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
    if (!initCollision(collision_xml, sdfCollision))
    {
      gzerr << "Unable to parser geom\n";
      return false;
    }

    for (xmlNodePtr  visual_xml = firstChildElement(collision_xml, "visual");
        visual_xml; visual_xml = nextSiblingElement(visual_xml, "visual"))
    {
      sdf::ElementPtr sdfVisual = _sdf->AddElement("visual");

      // set name to geom(collision) name append _visual
      sdfVisual->GetAttribute("name")->SetFromString(
          sdfCollision->GetAttribute("name")->GetAsString() + "_visual");

      if (!initVisual(visual_xml, sdfVisual))
      {
        gzerr << "Unable to parse visual\n";
        return false;
      }

      // In order to parse old gazebo xml (nested format)
      // to new sdf, we need to unwrap visual pose from within collision.
      // take origin of visual, multiply it by collision's transform
      gazebo::math::Pose col_pose =
        sdfCollision->GetElement("origin")->GetValuePose("pose");
      gazebo::math::Pose vis_pose =
        sdfVisual->GetElement("origin")->GetValuePose("pose");

      // aggregate poses
      vis_pose.pos = col_pose.pos +
        col_pose.rot.RotateVector(vis_pose.pos);
      vis_pose.rot = col_pose.rot * vis_pose.rot;

      // update the sdf pose
      sdfVisual->GetElement("origin")->GetAttribute("pose")->Set(vis_pose);
    }
    // TODO: check for duplicate geoms
  }

  // Get all sensor elements
  for (xmlNodePtr  sensor_xml = getChildByNSPrefix(_config, "sensor");
      sensor_xml; sensor_xml = getNextByNSPrefix(sensor_xml, "sensor"))
  {
    sdf::ElementPtr sdfSensor = _sdf->AddElement("sensor");
    if (!initSensor(sensor_xml, sdfSensor))
    {
      gzerr << "Unable to parse sensor\n";
      return false;
    }
    // TODO: check for duplicate sensors
  }

  // Get projector elements
  getProjectors(_config, _sdf);

  return true;
}

/// _config = <visual>
/// _sdf = visual
bool initVisual(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  _sdf->GetAttribute("cast_shadows")->SetFromString("true");

  initOrigin(_config, _sdf);

  sdf::ElementPtr sdfGeom = _sdf->AddElement("geometry");

  std::string mesh_attribute = getNodeValue(_config, "mesh");
  // check each mesh type
  if (mesh_attribute == "unit_box")
  {
    sdf::ElementPtr sdfBox = sdfGeom->AddElement("box");
    if (firstChildElement(_config, "scale"))
    {
      sdfBox->GetAttribute("size")->SetFromString(
          getNodeValue(_config, "scale"));
    }
    else if (firstChildElement(_config, "size"))
    {
      sdfBox->GetAttribute("size")->SetFromString(
          getNodeValue(_config, "size"));
    }
    else
    {
      sdfBox->GetAttribute("size")->SetFromString("1 1 1");
    }
  }
  else if (mesh_attribute == "unit_sphere")
  {
    sdf::ElementPtr sdfSphere = sdfGeom->AddElement("sphere");
    if (firstChildElement(_config, "scale"))
    {
      // FIXME: using first elem
      double sx =
        boost::lexical_cast<double>(getNodeTuple(_config, "scale", 0));
      sdfSphere->GetAttribute("radius")->Set(0.5*sx);
    }
    else if (firstChildElement(_config, "size"))
    {
      // FIXME: using first elem
      double sx =
        boost::lexical_cast<double>(getNodeTuple(_config, "size", 0));
      sdfSphere->GetAttribute("radius")->Set(sx);
    }
    else
      sdfSphere->GetAttribute("radius")->SetFromString("1.0");
  }
  else if (mesh_attribute == "unit_cylinder")
  {
    sdf::ElementPtr sdfCylinder = sdfGeom->AddElement("cylinder");

    if (firstChildElement(_config, "scale"))
    {
      double sx =
        boost::lexical_cast<double>(getNodeTuple(_config, "scale", 0));
      sdfCylinder->GetAttribute("radius")->Set(0.5*sx);
      sdfCylinder->GetAttribute("length")->SetFromString(
          getNodeTuple(_config, "scale", 2));
    }
    else if (firstChildElement(_config, "size"))
    {
      double sx = boost::lexical_cast<double>(getNodeTuple(_config, "size", 0));
      sdfCylinder->GetAttribute("radius")->Set(0.5*sx);
      sdfCylinder->GetAttribute("length")->SetFromString(
          getNodeTuple(_config, "size", 2));
    }
    else
    {
      sdfCylinder->GetAttribute("radius")->SetFromString("1");
      sdfCylinder->GetAttribute("length")->SetFromString("1");
    }
  }
  else if (!mesh_attribute.empty())
  {
    sdf::ElementPtr sdfMesh = sdfGeom->AddElement("mesh");
    sdfMesh->GetAttribute("filename")->SetFromString(mesh_attribute);

    if (firstChildElement(_config, "scale"))
    {
      sdfMesh->GetAttribute("scale")->SetFromString(
          getNodeValue(_config, "scale"));
    }
  }
  else
  {
    sdf::ElementPtr sdfPlane = sdfGeom->AddElement("plane");
    sdfPlane->GetAttribute("normal")->SetFromString(
        getNodeValue(_config, "normal"));
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
bool initJoint(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  initAttr(_config, "name", _sdf->GetAttribute("name"));

  // old <anchorOffset> translates to origin in the new sdf context
  xmlNodePtr anchor_offset_xml = firstChildElement(_config, "anchorOffset");
  std::string poseStr;
  if (anchor_offset_xml)
    poseStr += getValue(anchor_offset_xml) + " ";
  else
    poseStr += "0 0 0 ";
  // for rpy, which doesn't exist in old model xml
  poseStr += "0 0 0";
  sdf::ElementPtr origin = _sdf->AddElement("origin");
  origin->GetAttribute("pose")->SetFromString(poseStr);


  // setup parent / child links
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
    else if (sdfChild->GetAttribute("link")->GetAsString() ==
             getValue(body2Xml))
    {
      initAttr(_config, "body1", sdfParent->GetAttribute("link"));
    }
    else
    {
      gzerr << "body1 and body2 does not match anchor, "
            << "not sure which one is parent.\n";
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
  else if (std::string((const char*)_config->name) == "screw")
    _sdf->GetAttribute("type")->SetFromString("screw");
  else
    gzerr << "Unknown joint type[" << (const char*)_config->name << "]\n";

  // for screw joints, if threadPitch exists, translate to
  // "thread_pitch" in sdf
  xmlNodePtr threadPitchXml = firstChildElement(_config, "threadPitch");
  if (threadPitchXml)
    _sdf->GetElement("thread_pitch")->GetValue()->SetFromString(
        getValue(threadPitchXml));
    // _sdf->GetElement("thread_pitch")->value->SetFromString(
    // getValue(threadPitchXml));

  initAttr(_config, "anchor", sdfChild->GetAttribute("link"));
  if (firstChildElement(_config, "axis"))
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
      double stop_angle =
        boost::lexical_cast<double>(getNodeValue(_config, "lowStop"));
      if ((std::string((const char*)_config->name) == "slider") ||
          (std::string((const char*)_config->name) == "screw"))
        sdfLimit->GetAttribute("lower")->Set(stop_angle);
      else
        sdfLimit->GetAttribute("lower")->Set(GZ_DTOR(stop_angle));
    }
    if (firstChildElement(_config, "highStop"))
    {
      double stop_angle = boost::lexical_cast<double>(getNodeValue(_config,
            "highStop"));
      if ((std::string((const char*)_config->name) == "slider") ||
          (std::string((const char*)_config->name) == "screw"))
        sdfLimit->GetAttribute("upper")->Set(stop_angle);
      else
        sdfLimit->GetAttribute("upper")->Set(GZ_DTOR(stop_angle));
    }
  }

  if (firstChildElement(_config, "axis2"))
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
      double stop_angle = boost::lexical_cast<double>(getNodeValue(_config,
            "lowStop"));
      if ((std::string((const char*)_config->name) == "slider") ||
          (std::string((const char*)_config->name) == "screw"))
        sdfLimit->GetAttribute("lower")->Set(stop_angle);
      else
        sdfLimit->GetAttribute("lower")->Set(GZ_DTOR(stop_angle));
    }
    if (firstChildElement(_config, "highStop"))
    {
      double stop_angle =
        boost::lexical_cast<double>(getNodeValue(_config, "highStop"));
      if ((std::string((const char*)_config->name) == "slider") ||
          (std::string((const char*)_config->name) == "screw"))
        sdfLimit->GetAttribute("upper")->Set(stop_angle);
      else
        sdfLimit->GetAttribute("upper")->Set(GZ_DTOR(stop_angle));
    }
  }
  return true;
}

//////////////////////////////////////////////////
bool initModel(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  initAttr(_config, "name", _sdf->GetAttribute("name"));
  initAttr(_config, "static", _sdf->GetAttribute("static"));
  initOrigin(_config, _sdf);

  // Get all Link elements
  for (xmlNodePtr  linkXml = getChildByNSPrefix(_config, "body");
      linkXml; linkXml = getNextByNSPrefix(linkXml, "body"))
  {
    sdf::ElementPtr sdfLink = _sdf->AddElement("link");
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
    sdf::ElementPtr sdfJoint = _sdf->AddElement("joint");
    if (!initJoint(jointXml, sdfJoint))
    {
      gzerr << "joint xml is not initialized correctly\n";
      return false;
    }
  }

  /// Get all the plugins
  controller2Plugins(_config, _sdf);
  getGrippers(_config, _sdf);

  return true;
}

//////////////////////////////////////////////////
bool initWorld(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  // Set world name
  if (!_sdf->GetAttribute("name")->SetFromString("default"))
  {
    gzerr << "Unable to set world name\n";
    return false;
  }

  sdf::ElementPtr sdfScene = _sdf->AddElement("scene");
  initScene(firstChildElement(_config, "ogre"), sdfScene);

  sdf::ElementPtr sdfPhysics = _sdf->AddElement("physics");
  initPhysics(firstChildElement(_config, "ode"), sdfPhysics);

  // Get all model elements
  for (xmlNodePtr  modelXml = getChildByNSPrefix(_config, "model");
       modelXml; modelXml = getNextByNSPrefix(modelXml, "model"))
  {
    if (strcmp((const char*)modelXml->name, "renderable")== 0)
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
      if (!initModel(modelXml, sdfModel))
      {
        gzerr << "model xml is not initialized correctly\n";
        return false;
      }
    }
  }

  /// Get all the plugins
  controller2Plugins(_config, _sdf);

  return true;
}

//////////////////////////////////////////////////
bool initScene(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  sdf::ElementPtr sdfAmbient = _sdf->AddElement("ambient");
  if (sdfAmbient)
    initAttr(_config, "ambient", sdfAmbient->GetAttribute("rgba"));

  sdf::ElementPtr sdfBackground = _sdf->AddElement("background");
  if (sdfBackground)
    initAttr(_config, "background", sdfBackground->GetAttribute("rgba"));

  xmlNodePtr sky = firstChildElement(_config, "sky");
  if (sky)
  {
    sdf::ElementPtr sdfSky = sdfBackground->AddElement("sky");
    if (sdfSky)
      initAttr(sky, "material", sdfSky->GetAttribute("material"));
  }

  sdf::ElementPtr sdfShadow = _sdf->AddElement("shadows");

  if (sdfShadow)
    initAttr(_config, "shadows", sdfShadow->GetAttribute("enabled"));

  //  per pixel shading does not allow options
  // sdfShadow->GetAttribute("rgba")->SetFromString("0 0 0 0");
  // initAttr(_config, "shadowTechnique", sdfShadow->GetAttribute("type"));

  return true;
}

// _config = physics:ode
// _sdf = physics
bool initPhysics(xmlNodePtr _config, sdf::ElementPtr _sdf)
{
  _sdf->GetAttribute("type")->SetFromString("ode");

  sdf::ElementPtr sdfGravity = _sdf->AddElement("gravity");
  sdf::ElementPtr sdfODE = _sdf->AddElement("ode");
  sdf::ElementPtr sdfODESolver = sdfODE->AddElement("solver");
  sdf::ElementPtr sdfODEConstraints = sdfODE->AddElement("constraints");

  if (sdfGravity)
    initAttr(_config, "gravity", sdfGravity->GetAttribute("xyz"));

  if (sdfODESolver)
  {
    initAttr(_config, "stepType", sdfODESolver->GetAttribute("type"));
    initAttr(_config, "stepTime", sdfODESolver->GetAttribute("dt"));

    if (sdfODESolver->GetAttribute("type")->GetAsString() == "quick")
    {
      initAttr(_config, "stepIters", sdfODESolver->GetAttribute("iters"));
      initAttr(_config, "stepW", sdfODESolver->GetAttribute("sor"));
    }

    if (sdfODESolver->GetAttribute("type")->GetAsString() == "pgs")
    {
      initAttr(_config, "stepIters", sdfODESolver->GetAttribute("iters"));
      initAttr(_config, "stepW", sdfODESolver->GetAttribute("sor"));
    }
  }


  // Contraints
  if (sdfODEConstraints)
  {
    initAttr(_config, "cfm", sdfODEConstraints->GetAttribute("cfm"));
    initAttr(_config, "erp", sdfODEConstraints->GetAttribute("erp"));
    initAttr(_config, "contactMaxCorrectingVel",
        sdfODEConstraints->GetAttribute("contact_max_correcting_vel"));
    initAttr(_config, "contactSurfaceLayer",
        sdfODEConstraints->GetAttribute("contact_surface_layer"));
  }
  return true;
}


bool initAttr(xmlNodePtr _node, const std::string &_key,
              sdf::ParamPtr _attr)
{
  if (_node)
  {
    std::string value = getNodeValue(_node, _key);
    if (value.empty())
    {
      // gzdbg << "Node[" << _node->name << "] does not have key value["
      //       << _key << "] defined, will use default value.\n";
      return false;
    }
    if (!_attr->SetFromString(value))
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

//////////////////////////////////////////////////
bool initElem(xmlNodePtr /*_node*/, const std::string &/*_key*/,
              sdf::ElementPtr /*_attr*/)
{
  // to be implemented
  return false;
}

//////////////////////////////////////////////////
bool initModelFile(const std::string &_filename, sdf::SDFPtr &_sdf)
{
  std::ifstream fin;
  fin.open(_filename.c_str(), std::ios::in);
  if (!fin.is_open())
  {
    gzerr << "The model file can not be opened, check path and permissions\n";
    fin.close();
    return false;
  }
  fin.close();

  // Enable line numbering
  xmlLineNumbersDefault(1);

  std::string output;
  PreParser(_filename, output);

  return initModelString(output, _sdf);
}


//////////////////////////////////////////////////
bool initModelString(const std::string &_xmlString, sdf::SDFPtr &_sdf)
{
  xmlDocPtr xmlDoc =
    xmlParseDoc(reinterpret_cast<const xmlChar*>(_xmlString.c_str()));

  return initModelDoc(xmlDoc, _sdf);
}

//////////////////////////////////////////////////
bool initModelDoc(xmlDocPtr _xmlDoc, sdf::SDFPtr &_sdf)
{
  if (!_xmlDoc)
  {
    gzerr << "Could not parse the xml\n";
    return false;
  }

  ExpandIncludes(xmlDocGetRootElement(_xmlDoc));

  bool model_initialized = false;
  xmlNodePtr modelXml = firstChildElement(_xmlDoc, "physical");
  while (modelXml)
  {
    sdf::ElementPtr model = _sdf->root->AddElement("model");
    initModel(modelXml, model);

    modelXml = nextSiblingElement(modelXml, "physical");
    // need at least one model, otherwise, return false and try as model
    model_initialized = true;
  }

  if (model_initialized)
    return true;
  else
    return false;
}

//////////////////////////////////////////////////
bool initWorldFile(const std::string &_filename, sdf::SDFPtr &_sdf)
{
  std::ifstream fin;
  fin.open(_filename.c_str(), std::ios::in);
  if (!fin.is_open())
  {
    gzerr << "The world file can not be opened, check path and permissions\n";
  }
  fin.close();

  // Enable line numbering
  xmlLineNumbersDefault(1);

  std::string output;
  PreParser(_filename, output);

  return initWorldString(output, _sdf);
}

//////////////////////////////////////////////////
bool initWorldString(const std::string &_xmlString,
                     sdf::SDFPtr &_sdf)
{
  xmlDocPtr xmlDoc =
    xmlParseDoc(reinterpret_cast<const xmlChar*>(_xmlString.c_str()));

  return initWorldDoc(xmlDoc, _sdf);
}

//////////////////////////////////////////////////
bool initWorldDoc(xmlDocPtr _xmlDoc, sdf::SDFPtr &_sdf)
{
  if (!_xmlDoc)
  {
    gzerr << "Could not parse the xml\n";
    return false;
  }

  ExpandIncludes(xmlDocGetRootElement(_xmlDoc));

  // add or set version string if needed
  if (_sdf->root->GetAttribute("version"))
    _sdf->root->GetAttribute("version")->SetFromString("1.0");
  else
    _sdf->root->AddAttribute("version", "string", "1.0", false);

  bool world_initialized = false;
  xmlNodePtr worldXml = firstChildElement(_xmlDoc, "world");
  while (worldXml)
  {
    sdf::ElementPtr world = _sdf->root->AddElement("world");
    world_initialized = initWorld(worldXml, world);
    worldXml = nextSiblingElement(worldXml, "world");
  }

  // need all worlds successfully initialized, otherwise,
  // return false and try as model
  if (world_initialized)
    return true;
  else
    return false;
}

xmlNodePtr firstChildElement(xmlDocPtr node, const std::string &name)
{
  xmlNodePtr tmp;
  for (tmp = node->xmlChildrenNode; tmp != NULL; tmp = tmp->next)
    if (tmp->name && name == (const char*)tmp->name) break;

  return tmp;
}
xmlNodePtr firstChildElement(xmlNodePtr node, const std::string &name)
{
  xmlNodePtr tmp;
  for (tmp = xmlFirstElementChild(node);
       tmp != NULL; tmp = xmlNextElementSibling(tmp))
    if (tmp->name && (name == (const char*)tmp->name)) break;

  return tmp;
}

xmlNodePtr nextSiblingElement(xmlNodePtr node, const std::string &name)
{
  xmlNodePtr tmp;
  for (tmp = xmlNextElementSibling(node);
       tmp != NULL; tmp = xmlNextElementSibling(tmp))
    if (tmp->name && (name == (const char*)tmp->name)) break;

  return tmp;
}

xmlNodePtr getNextByNSPrefix(xmlNodePtr node, const std::string &prefix)
{
  xmlNodePtr tmp;
  for (tmp = xmlNextElementSibling(node);
       tmp != NULL; tmp = xmlNextElementSibling(tmp))
    if (tmp->ns && prefix == (const char*)tmp->ns->prefix)
      break;
  return tmp;
}

xmlNodePtr getChildByNSPrefix(xmlNodePtr node, const std::string &prefix)
{
  xmlNodePtr tmp;
  for (tmp = node->xmlChildrenNode;
       tmp != NULL; tmp = xmlNextElementSibling(tmp))
    if (tmp->ns && prefix == (const char*)tmp->ns->prefix)
      break;
  return tmp;
}

std::string getNodeValue(xmlNodePtr node, const std::string &key)
{
  std::string result;
  xmlChar *value = NULL;

  // First check if the key is an attribute
  if (xmlHasProp(node, reinterpret_cast<const xmlChar*>(key.c_str())))
  {
    value = xmlGetProp(node, reinterpret_cast<const xmlChar*>(key.c_str()));

    // If not an attribute, then it should be a child node
  }
  else if (key == reinterpret_cast<const char*>(node->name))
  {
    value = xmlNodeListGetString(node->doc, node->xmlChildrenNode, 1);
  }
  else
  {
    xmlNodePtr currNode;

    currNode = node->xmlChildrenNode;

    // Loop through children
    while (currNode)
    {
      // If the name matches, then return its value
      if (key == reinterpret_cast<const char*>(currNode->name))
      {
        value = xmlNodeListGetString(node->doc, currNode->xmlChildrenNode, 1);
        break;
      }

      currNode = currNode->next;
    }
  }

  if (value)
  {
    result = reinterpret_cast<char*>(value);
    boost::trim(result);

    xmlFree(value);
  }

  return result;
}
std::string getNodeTuple(xmlNodePtr node, const std::string &key, int index)
{
  std::string value;
  std::string nvalue;
  int i, a, b, state, count;

  value = getNodeValue(node, key);

  if (value.empty())
    return std::string();

  state = 0;
  count = 0;
  a = b = 0;

  for (i = 0; i < static_cast<int>(value.size()); i++)
  {
    // Look for start of element
    if (state == 0)
    {
      if (!isspace(value[i]))
      {
        a = i;
        state = 1;
      }
    }

    // Look for end of element
    else if (state == 1)
    {
      if (isspace(value[i]))
      {
        state = 0;
        b = i - 1;
        count++;
        if (count > index)
          break;
      }
    }
  }
  if (state == 1)
  {
    b = i - 1;
    count++;
  }

  if (count == index + 1)
  {
    const char *s = value.c_str() + a;
    size_t size = b-a+2;
    const char *end = reinterpret_cast<const char *>(memchr(s, 0, size));

    if (end)
      size = end - s + 1;

    char *r = static_cast<char *>(malloc(size));

    if (size)
    {
      memcpy(r, s, size-1);
      r[size-1] = '\0';
    }

    nvalue = r;
  }

  return nvalue;
}

std::string getValue(xmlNodePtr node)
{
  const char *v =
    (const char*)xmlNodeListGetString(node->doc, node->xmlChildrenNode, 1);
  if (v)
    return std::string(v);
  else
    return std::string();
}


void PreParser(const std::string &fname, std::string &output)
{
  std::ifstream ifs(fname.c_str(), std::ios::in);
  std::string line;

  while (ifs.good())
  {
    std::getline(ifs, line);
    boost::trim(line);
    output += line + "\n";
  }
  ifs.close();
}

void ExpandIncludes(xmlNodePtr _xml)
{
  // walk through all children nodes
  for (xmlNodePtr elemXml = xmlFirstElementChild(_xml);
       elemXml != NULL; elemXml = xmlNextElementSibling(elemXml))
  {
    xmlNodePtr includeXml = firstChildElement(_xml, "include");
    while (includeXml)
    {
      xmlNodePtr includeXiXml = firstChildElement(includeXml, "include");
      if (includeXiXml)
      {
        std::string filename = getNodeValue(includeXiXml, "href");
        std::ifstream fin;
        fin.open(filename.c_str(), std::ios::in);
        if (!fin.is_open())
        {
          gzerr << "The included file [" << filename
                << "] cannot be opened, exists? permitted?\n";
        }
        else
        {
          std::string output;
          PreParser(filename, output);
          fin.close();
          xmlDocPtr xmlHref =
            xmlParseDoc(reinterpret_cast<const xmlChar*>(output.c_str()));

          // only take first model in included .model file (potentially more?)
          xmlNodePtr includePhysical = firstChildElement(xmlHref, "physical");

          for (xmlNodePtr c = xmlFirstElementChild(includePhysical);
               c != NULL; c = xmlNextElementSibling(c))
            xmlAddChild(_xml, c);

          // debug
          // gzerr << "-------------------\n" << ToString(_xml)
          //      << "\n======================\n";
        }
      }

      // look for next include block and remove includeXml node
      xmlNodePtr includeXml2 = nextSiblingElement(includeXml, "include");
      xmlUnlinkNode(includeXml);
      xmlFreeNode(includeXml);
      includeXml = includeXml2;
    }

    // call recursively
    ExpandIncludes(elemXml);
  }
}

std::string ToString(xmlNodePtr _xml)
{
  xmlDocPtr doc = _xml->doc;
  // this doesn't appear to work, do i need to allocate some memory here?
  // or do something else?
  xmlOutputBufferPtr output = NULL;
  xmlNodeDumpOutput(output, doc, _xml, 0, 1, "UTF-8");

  // somehow convert output to a string?
  xmlChar *s;
  int size;
  xmlDocDumpMemory(doc, &s, &size);
  std::string xmlString = (char *)s;
  xmlFree(s);

  return xmlString;
}

std::string ToString(xmlDocPtr _doc)
{
  // this doesn't appear to work, do i need to allocate some memory here?
  // or do something else?
  xmlOutputBufferPtr output = NULL;
  xmlNodeDumpOutput(output, _doc, xmlDocGetRootElement(_doc), 0, 1, "UTF-8");

  // somehow convert output to a string?
  xmlChar *s;
  int size;
  xmlDocDumpMemory(_doc, &s, &size);
  std::string xmlString = (char *)s;
  xmlFree(s);

  return xmlString;
}
}

