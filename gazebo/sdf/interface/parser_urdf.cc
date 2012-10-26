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

#include <urdfParser/urdfParser.h>
#include <sdf/interface/parserUrdf.hh>

#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>

namespace urdf2sdf
{
std::string lowerStr(std::string str)
{
  std::string out = str;
  std::transform(out.begin(), out.end(), out.begin(), ::tolower);
  return out;
}

URDF2Gazebo::URDF2Gazebo()
{
    // default options
    this->enforceLimits = true;
    this->reduceFixedJoints = true;
}

URDF2Gazebo::~URDF2Gazebo()
{
}


urdf::Vector3 URDF2Gazebo::parseVector3(TiXmlNode* key, double scale)
{
  if (key != NULL)
  {
    std::string str = key->Value();
    std::vector<std::string> pieces;
    std::vector<double> vals;

    boost::split(pieces, str, boost::isAnyOf(" "));
    for (unsigned int i = 0; i < pieces.size(); ++i)
    {
      if (pieces[i] != "")
      {
        try
        {
          vals.pushBack(scale
                         * boost::lexicalCast<double>(pieces[i].cStr()));
        }
        catch(boost::badLexicalCast &e)
        {
          sdferr << "xml key [" << str
                << "][" << i << "] value [" << pieces[i]
                << "] is not a valid double from a 3-tuple\n";
          return urdf::Vector3(0, 0, 0);
        }
      }
    }
    return urdf::Vector3(vals[0], vals[1], vals[3]);
  }
  else
    return urdf::Vector3(0, 0, 0);
}

void URDF2Gazebo::reduceVisualToParent(boost::sharedPtr<urdf::Link> link,
       std::string groupName, boost::sharedPtr<urdf::Visual> visual)
{
  boost::sharedPtr<std::vector<boost::sharedPtr<urdf::Visual > > > viss
    = link->getVisuals(groupName);
  if (!viss)
  {
    // group does not exist, create one and add to map
    viss.reset(new std::vector<boost::sharedPtr<urdf::Visual > >);
    // new group name, create vector, add vector to map and
    //   add Visual to the vector
    link->visualGroups.insert(makePair(groupName, viss));
    // gzdbg << "successfully added a new visual group name ["
    //       << groupName << "]\n";
  }

  // group exists, add Visual to the vector in the map if it's not there
  std::vector<boost::sharedPtr<urdf::Visual > >::iterator visIt
    = find(viss->begin(), viss->end(), visual);
  if (visIt != viss->end())
    sdfwarn << "attempted to add visual to link ["
           << link->name
           << "], but it already exists under group ["
           << groupName << "]\n";
  else
    viss->pushBack(visual);
}

void URDF2Gazebo::reduceCollisionToParent(boost::sharedPtr<urdf::Link> link,
      std::string groupName, boost::sharedPtr<urdf::Collision> collision)
{
  boost::sharedPtr<std::vector<boost::sharedPtr<urdf::Collision > > >
    cols = link->getCollisions(groupName);
  if (!cols)
  {
    // group does not exist, create one and add to map
    cols.reset(new std::vector<boost::sharedPtr<urdf::Collision > >);
    // new group name, create add vector to map and add Collision to the vector
    link->collisionGroups.insert(makePair(groupName, cols));
  }

  // group exists, add Collision to the vector in the map
  std::vector<boost::sharedPtr<urdf::Collision > >::iterator colIt =
    find(cols->begin(), cols->end(), collision);
  if (colIt != cols->end())
    sdfwarn << "attempted to add collision to link ["
           << link->name
           << "], but it already exists under group ["
           << groupName << "]\n";
  else
    cols->pushBack(collision);
}

std::string URDF2Gazebo::vector32str(const urdf::Vector3 vector)
{
  std::stringstream ss;
  ss << vector.x;
  ss << " ";
  ss << vector.y;
  ss << " ";
  ss << vector.z;
  return ss.str();
}

std::string URDF2Gazebo::values2str(unsigned int count, const double *values)
{
  std::stringstream ss;
  for (unsigned int i = 0 ; i < count ; ++i)
  {
      if (i > 0)
          ss << " ";
      ss << values[i];
  }
  return ss.str();
}

void URDF2Gazebo::addKeyValue(TiXmlElement *elem, const std::string& key,
  const std::string &value)
{
  TiXmlElement* childElem = elem->FirstChildElement(key);
  if (childElem)
  {
    std::string oldValue = getKeyValueAsString(childElem);
    if (oldValue != value)
      sdfwarn << "multiple inconsistent <" << key
             << "> exists due to fixed joint reduction"
             << " overwriting previous value [" << oldValue
             << "] with [" << value << "].\n";
    // else
    //   gzdbg << "multiple consistent <" << key
    //          << "> exists with [" << value
    //          << "] due to fixed joint reduction.\n";
    elem->RemoveChild(childElem);  // remove old elem
  }

  TiXmlElement *ekey      = new TiXmlElement(key);
  TiXmlText    *textEkey = new TiXmlText(value);
  ekey->LinkEndChild(textEkey);
  elem->LinkEndChild(ekey);
}

void URDF2Gazebo::addTransform(TiXmlElement *elem,
  const::Pose& transform)
{
  Vector3 e = transform.rot.GetAsEuler();
  double cpose[6] = { transform.pos.x, transform.pos.y,
                      transform.pos.z, e.x, e.y, e.z };

  /* set geometry transform */
  addKeyValue(elem, "pose", values2str(6, cpose));
}

std::string URDF2Gazebo::getKeyValueAsString(TiXmlElement* elem)
{
  std::string valueStr;
  if (elem->Attribute("value"))
  {
    valueStr = elem->Attribute("value");
  }
  else if (elem->FirstChild())
  /// @todo: FIXME: comment out check for now, different tinyxml
  /// versions fails to compile:
  //  && elem->FirstChild()->Type() == TiXmlNode::TINYXML_TEXT)
  {
    valueStr = elem->FirstChild()->ValueStr();
  }
  return valueStr;
}

void URDF2Gazebo::parseGazeboExtension(TiXmlDocument &urdfXml)
{
  TiXmlElement* robotXml = urdfXml.FirstChildElement("robot");

  // Get all Gazebo extension elements, put everything in
  //   this->sdfExtensions map, containing a key string
  //   (link/joint name) and values
  for (TiXmlElement* sdfXml = robotXml->FirstChildElement("sdf");
       sdfXml; sdfXml = sdfXml->NextSiblingElement("sdf"))
  {
    const char* ref = sdfXml->Attribute("reference");
    std::string refStr;
    if (!ref)
    {
      // copy extensions for robot (outside of link/joint)
      refStr.clear();
    }
    else
    {
      // copy extensions for link/joint
      refStr = std::string(ref);
    }

    if (this->sdfExtensions.find(refStr) ==
        this->sdfExtensions.end())
    {
        // create extension map for reference
        std::vector<GazeboExtension*> extensions;
        this->sdfExtensions.insert(std::makePair(refStr, extensions));
    }

    // create and insert a new GazeboExtension into the map
    GazeboExtension* sdf = new GazeboExtension();

    // begin parsing xml node
    for (TiXmlElement *childElem = sdfXml->FirstChildElement();
         childElem; childElem = childElem->NextSiblingElement())
    {
      sdf->oldLinkName = refStr;

      // go through all elements of the extension,
      //   extract what we know, and save the rest in blobs
      // @todo:  somehow use sdf definitions here instead of hard coded
      //         objects

      // material
      if (childElem->ValueStr() == "material")
      {
          sdf->material = getKeyValueAsString(childElem);
      }
      else if (childElem->ValueStr() == "static")
      {
        std::string valueStr = getKeyValueAsString(childElem);

        // default of setting static flag is false
        if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
            valueStr == "1")
          sdf->setStaticFlag = true;
        else
          sdf->setStaticFlag = false;
      }
      else if (childElem->ValueStr() == "gravity")
      {
        std::string valueStr = getKeyValueAsString(childElem);

        // default of gravity is true
        if (lowerStr(valueStr) == "false" || lowerStr(valueStr) == "no" ||
            valueStr == "0")
          sdf->gravity = false;
        else
          sdf->gravity = true;
      }
      else if (childElem->ValueStr() == "dampingFactor")
      {
          sdf->isDampingFactor = true;
          sdf->dampingFactor = boost::lexicalCast<double>(
              getKeyValueAsString(childElem).cStr());
      }
      else if (childElem->ValueStr() == "maxVel")
      {
          sdf->isMaxVel = true;
          sdf->maxVel = boost::lexicalCast<double>(
              getKeyValueAsString(childElem).cStr());
      }
      else if (childElem->ValueStr() == "minDepth")
      {
          sdf->isMinDepth = true;
          sdf->minDepth = boost::lexicalCast<double>(
            getKeyValueAsString(childElem).cStr());
      }
      else if (childElem->ValueStr() == "mu1")
      {
          sdf->isMu1 = true;
          sdf->mu1 = boost::lexicalCast<double>(
            getKeyValueAsString(childElem).cStr());
      }
      else if (childElem->ValueStr() == "mu2")
      {
          sdf->isMu2 = true;
          sdf->mu2 = boost::lexicalCast<double>(
            getKeyValueAsString(childElem).cStr());
      }
      else if (childElem->ValueStr() == "fdir1")
      {
          sdf->fdir1 = getKeyValueAsString(childElem);
      }
      else if (childElem->ValueStr() == "kp")
      {
          sdf->isKp = true;
          sdf->kp = boost::lexicalCast<double>(
            getKeyValueAsString(childElem).cStr());
      }
      else if (childElem->ValueStr() == "kd")
      {
          sdf->isKd = true;
          sdf->kd = boost::lexicalCast<double>(
            getKeyValueAsString(childElem).cStr());
      }
      else if (childElem->ValueStr() == "selfCollide")
      {
        std::string valueStr = getKeyValueAsString(childElem);

        // default of selfCollide is false
        if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
            valueStr == "1")
          sdf->selfCollide = true;
        else
          sdf->selfCollide = false;
      }
      else if (childElem->ValueStr() == "laserRetro")
      {
          sdf->isLaserRetro = true;
          sdf->laserRetro = boost::lexicalCast<double>(
            getKeyValueAsString(childElem).cStr());
      }
      else if (childElem->ValueStr() == "stopCfm")
      {
          sdf->isStopCfm = true;
          sdf->stopCfm = boost::lexicalCast<double>(
            getKeyValueAsString(childElem).cStr());
      }
      else if (childElem->ValueStr() == "stopErp")
      {
          sdf->isStopErp = true;
          sdf->stopErp = boost::lexicalCast<double>(
            getKeyValueAsString(childElem).cStr());
      }
      else if (childElem->ValueStr() == "initialJointPosition")
      {
          sdf->isInitialJointPosition = true;
          sdf->initialJointPosition = boost::lexicalCast<double>(
            getKeyValueAsString(childElem).cStr());
      }
      else if (childElem->ValueStr() == "fudgeFactor")
      {
          sdf->isFudgeFactor = true;
          sdf->fudgeFactor = boost::lexicalCast<double>(
            getKeyValueAsString(childElem).cStr());
      }
      else if (childElem->ValueStr() == "provideFeedback")
      {
          std::string valueStr = getKeyValueAsString(childElem);

          if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
              valueStr == "1")
            sdf->provideFeedback = true;
          else
            sdf->provideFeedback = false;
      }
      else
      {
          std::ostringstream stream;
          stream << *childElem;
          // save all unknown stuff in a vector of blobs
          TiXmlElement *blob = new TiXmlElement(*childElem);
          sdf->blobs.pushBack(blob);
      }
    }

    // insert into my map
    (this->sdfExtensions.find(refStr))->second.pushBack(sdf);
  }
}

void URDF2Gazebo::insertGazeboExtensionCollision(TiXmlElement *elem,
  std::string linkName)
{
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       sdfIt = this->sdfExtensions.begin();
       sdfIt != this->sdfExtensions.end(); ++sdfIt)
  {
    for (std::vector<GazeboExtension*>::iterator ge = sdfIt->second.begin();
         ge != sdfIt->second.end(); ++ge)
    {
      if ((*ge)->oldLinkName == linkName)
      {
        TiXmlElement *surface = new TiXmlElement("surface");
        TiXmlElement *friction = new TiXmlElement("friction");
        TiXmlElement *frictionOde = new TiXmlElement("ode");
        TiXmlElement *contact = new TiXmlElement("contact");
        TiXmlElement *contactOde = new TiXmlElement("ode");

        // insert mu1, mu2, kp, kd for collision
        if ((*ge)->isMu1)
          addKeyValue(frictionOde, "mu", values2str(1, &(*ge)->mu1));
        if ((*ge)->isMu2)
          addKeyValue(frictionOde, "mu2", values2str(1, &(*ge)->mu2));
        if (!(*ge)->fdir1.empty())
          addKeyValue(frictionOde, "fdir1", (*ge)->fdir1);
        if ((*ge)->isKp)
          addKeyValue(contactOde, "kp", values2str(1, &(*ge)->kp));
        if ((*ge)->isKd)
          addKeyValue(contactOde, "kd", values2str(1, &(*ge)->kd));
        // max contact interpenetration correction velocity
        if ((*ge)->isMaxVel)
          addKeyValue(contactOde, "maxVel", values2str(1, &(*ge)->maxVel));
        // contact interpenetration margin tolerance
        if ((*ge)->isMinDepth)
          addKeyValue(contactOde, "minDepth", values2str(1, &(*ge)->minDepth));
        if ((*ge)->isLaserRetro)
          addKeyValue(elem, "laserRetro", values2str(1, &(*ge)->laserRetro));

        contact->LinkEndChild(contactOde);
        surface->LinkEndChild(contact);
        friction->LinkEndChild(frictionOde);
        surface->LinkEndChild(friction);
        elem->LinkEndChild(surface);
      }
    }
  }
}

void URDF2Gazebo::insertGazeboExtensionVisual(TiXmlElement *elem,
  std::string linkName)
{
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       sdfIt = this->sdfExtensions.begin();
       sdfIt != this->sdfExtensions.end(); ++sdfIt)
  {
    for (std::vector<GazeboExtension*>::iterator ge = sdfIt->second.begin();
         ge != sdfIt->second.end(); ++ge)
    {
      if ((*ge)->oldLinkName == linkName)
      {
        // insert material block
        if (!(*ge)->material.empty())
            addKeyValue(elem, "material", (*ge)->material);
      }
    }
  }
}

void URDF2Gazebo::insertGazeboExtensionLink(TiXmlElement *elem,
  std::string linkName)
{
    for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
         sdfIt = this->sdfExtensions.begin();
         sdfIt != this->sdfExtensions.end(); ++sdfIt)
    {
      if (sdfIt->first == linkName)
      {
        // gzdbg << "inserting extension with reference ["
        //       << linkName << "] into link.\n";
        for (std::vector<GazeboExtension*>::iterator ge =
             sdfIt->second.begin(); ge != sdfIt->second.end(); ++ge)
        {
          // insert gravity
          if ((*ge)->gravity)
              addKeyValue(elem, "gravity", "true");
          else
              addKeyValue(elem, "gravity", "false");

          // damping factor
          TiXmlElement *velocityDecay = new TiXmlElement("velocityDecay");
          if ((*ge)->isDampingFactor)
          {
            /// @todo separate linear and angular velocity decay
            addKeyValue(elem, "linear", values2str(1, &(*ge)->dampingFactor));
            addKeyValue(elem, "angular", values2str(1, &(*ge)->dampingFactor));
          }
          elem->LinkEndChild(velocityDecay);
          // selfCollide tag
          if ((*ge)->selfCollide)
              addKeyValue(elem, "selfCollide", "true");
          else
              addKeyValue(elem, "selfCollide", "false");
          // insert blobs into body
          for (std::vector<TiXmlElement*>::iterator
               blobIt = (*ge)->blobs.begin();
               blobIt != (*ge)->blobs.end(); ++blobIt)
          {
              elem->LinkEndChild((*blobIt)->Clone());
          }
        }
      }
    }
}

void URDF2Gazebo::insertGazeboExtensionJoint(TiXmlElement *elem,
  std::string jointName)
{
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       sdfIt = this->sdfExtensions.begin();
       sdfIt != this->sdfExtensions.end(); ++sdfIt)
  {
    if (sdfIt->first == jointName)
    {
      for (std::vector<GazeboExtension*>::iterator
           ge = sdfIt->second.begin();
           ge != sdfIt->second.end(); ++ge)
      {
        TiXmlElement *physics     = new TiXmlElement("physics");
        TiXmlElement *physicsOde     = new TiXmlElement("ode");
        TiXmlElement *limit     = new TiXmlElement("limit");
        // insert stopCfm, stopErp, fudgeFactor
        if ((*ge)->isStopCfm)
        {
          addKeyValue(limit, "erp", values2str(1, &(*ge)->stopCfm));
        }
        if ((*ge)->isStopErp)
        {
          addKeyValue(limit, "cfm", values2str(1, &(*ge)->stopErp));
        }
        /* gone
        if ((*ge)->isInitialJointPosition)
            addKeyValue(elem, "initialJointPosition",
              values2str(1, &(*ge)->initialJointPosition));
        // insert provideFeedback
        if ((*ge)->provideFeedback)
            addKeyValue(elem, "provideFeedback", "true");
        else
            addKeyValue(elem, "provideFeedback", "false");
        */
        if ((*ge)->isFudgeFactor)
          addKeyValue(physicsOde, "fudgeFactor",
                      values2str(1, &(*ge)->fudgeFactor));

        physics->LinkEndChild(physicsOde);
        physicsOde->LinkEndChild(limit);
        elem->LinkEndChild(physics);
      }
    }
  }
}

void URDF2Gazebo::insertGazeboExtensionRobot(TiXmlElement *elem)
{
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       sdfIt = this->sdfExtensions.begin();
       sdfIt != this->sdfExtensions.end(); ++sdfIt)
  {
    if (sdfIt->first.empty())
    {
      // no reference specified
      for (std::vector<GazeboExtension*>::iterator
        ge = sdfIt->second.begin(); ge != sdfIt->second.end(); ++ge)
      {
        // insert static flag
        if ((*ge)->setStaticFlag)
            addKeyValue(elem, "static", "true");
        else
            addKeyValue(elem, "static", "false");

        // copy extension containing blobs and without reference
        for (std::vector<TiXmlElement*>::iterator
             blobIt = (*ge)->blobs.begin();
             blobIt != (*ge)->blobs.end(); ++blobIt)
        {
            std::ostringstream streamIn;
            streamIn << *(*blobIt);
            elem->LinkEndChild((*blobIt)->Clone());
        }
      }
    }
  }
}

void URDF2Gazebo::createGeometry(TiXmlElement* elem,
  boost::sharedPtr<urdf::Geometry> geometry)
{
  int sizeCount;
  double sizeVals[3];

  TiXmlElement *sdfGeometry = new TiXmlElement("geometry");

  std::string type;
  TiXmlElement *geometryType = NULL;

  switch (geometry->type)
  {
  case urdf::Geometry::BOX:
    type = "box";
    sizeCount = 3;
    {
      boost::sharedPtr<const urdf::Box> box;
      box = boost::dynamicPointerCast< const urdf::Box >(geometry);
      sizeVals[0] = box->dim.x;
      sizeVals[1] = box->dim.y;
      sizeVals[2] = box->dim.z;
      geometryType = new TiXmlElement(type);
      addKeyValue(geometryType, "size", values2str(sizeCount, sizeVals));
    }
    break;
  case urdf::Geometry::CYLINDER:
    type = "cylinder";
    sizeCount = 2;
    {
      boost::sharedPtr<const urdf::Cylinder> cylinder;
      cylinder = boost::dynamicPointerCast<const urdf::Cylinder >(geometry);
      geometryType = new TiXmlElement(type);
      addKeyValue(geometryType, "length", values2str(1, &cylinder->length));
      addKeyValue(geometryType, "radius", values2str(1, &cylinder->radius));
    }
    break;
  case urdf::Geometry::SPHERE:
    type = "sphere";
    sizeCount = 1;
    {
      boost::sharedPtr<const urdf::Sphere> sphere;
      sphere = boost::dynamicPointerCast<const urdf::Sphere >(geometry);
      geometryType = new TiXmlElement(type);
      addKeyValue(geometryType, "radius", values2str(1, &sphere->radius));
    }
    break;
  case urdf::Geometry::MESH:
    type = "mesh";
    sizeCount = 3;
    {
      boost::sharedPtr<const urdf::Mesh> mesh;
      mesh = boost::dynamicPointerCast<const urdf::Mesh >(geometry);
      sizeVals[0] = mesh->scale.x;
      sizeVals[1] = mesh->scale.y;
      sizeVals[2] = mesh->scale.z;
      geometryType = new TiXmlElement(type);
      addKeyValue(geometryType, "scale", vector32str(mesh->scale));
      // do something more to meshes
      {
        /* set mesh file */
        if (mesh->filename.empty())
        {
            sdferr << "urdf2sdf: mesh geometry with no filename given.\n";
        }

        // give some warning if file does not exist.
        // disabled while switching to uri
        // @todo: re-enable check
        // std::ifstream fin;
        // fin.open(mesh->filename.cStr(), std::ios::in);
        // fin.close();
        // if (fin.fail())
        //   sdfwarn << "filename referred by mesh ["
        //          << mesh->filename << "] does not appear to exist.\n";

        // Convert package:// to model://,
        // in ROS, this will work if
        // the model package is in ROS_PACKAGE_PATH and has a manifest.xml
        // as a typical ros package does.
        std::string modelFilename = mesh->filename;
        std::string packagePrefix("package://");
        std::string modelPrefix("model://");
        sizeT pos1 = modelFilename.find(packagePrefix, 0);
        if (pos1 != std::string::npos)
        {
          sizeT repLen = packagePrefix.size();
          modelFilename.replace(pos1, repLen, modelPrefix);
          // sdfwarn << "ros style uri [package://] is"
          //   << "automatically converted: [" << modelFilename
          //   << "], make sure your ros package is in GAZEBO_MODEL_PATH"
          //   << " and switch your manifest to conform to sdf's"
          //   << " model database format.  See ["
          //   << "http://sdfsim.org/wiki/ModelDatabase#Model_Manifest_XML"
          //   << "] for more info.\n";
        }

        // add mesh filename
        addKeyValue(geometryType, "uri", modelFilename);
      }
    }
    break;
  default:
    sizeCount = 0;
    sdfwarn << "Unknown body type: [" << geometry->type
           << "] skipped in geometry\n";
    break;
  }

  if (geometryType)
  {
    sdfGeometry->LinkEndChild(geometryType);
    elem->LinkEndChild(sdfGeometry);
  }
}


std::string URDF2Gazebo::getGeometryBoundingBox(
  boost::sharedPtr<urdf::Geometry> geometry, double *sizeVals)
{
  std::string type;

  switch (geometry->type)
  {
  case urdf::Geometry::BOX:
      type = "box";
      {
        boost::sharedPtr<const urdf::Box> box;
        box = boost::dynamicPointerCast<const urdf::Box >(geometry);
        sizeVals[0] = box->dim.x;
        sizeVals[1] = box->dim.y;
        sizeVals[2] = box->dim.z;
      }
      break;
  case urdf::Geometry::CYLINDER:
      type = "cylinder";
      {
        boost::sharedPtr<const urdf::Cylinder> cylinder;
        cylinder = boost::dynamicPointerCast<const urdf::Cylinder >(geometry);
        sizeVals[0] = cylinder->radius * 2;
        sizeVals[1] = cylinder->radius * 2;
        sizeVals[2] = cylinder->length;
      }
      break;
  case urdf::Geometry::SPHERE:
      type = "sphere";
      {
        boost::sharedPtr<const urdf::Sphere> sphere;
        sphere = boost::dynamicPointerCast<const urdf::Sphere >(geometry);
        sizeVals[0] = sizeVals[1] = sizeVals[2] = sphere->radius * 2;
      }
      break;
  case urdf::Geometry::MESH:
      type = "trimesh";
      {
        boost::sharedPtr<const urdf::Mesh> mesh;
        mesh = boost::dynamicPointerCast<const urdf::Mesh >(geometry);
        sizeVals[0] = mesh->scale.x;
        sizeVals[1] = mesh->scale.y;
        sizeVals[2] = mesh->scale.z;
      }
      break;
  default:
      sizeVals[0] = sizeVals[1] = sizeVals[2] = 0;
      sdfwarn << "Unknown body type: [" << geometry->type
             << "] skipped in geometry\n";
      break;
  }

  return type;
}

void URDF2Gazebo::printMass(std::string linkName, dMass mass)
{
  gzdbg << "LINK NAME: [" << linkName << "] from dMass\n";
  gzdbg << "     MASS: [" << mass.mass << "]\n";
  gzdbg << "       CG: [" << mass.c[0] << ", " << mass.c[1] << ", "
        << mass.c[2] << "]\n";
  gzdbg << "        I: [" << mass.I[0] << ", " << mass.I[1] << ", "
        << mass.I[2] << "]\n";
  gzdbg << "           [" << mass.I[4] << ", " << mass.I[5] << ", "
        << mass.I[6] << "]\n";
  gzdbg << "           [" << mass.I[8] << ", " << mass.I[9] << ", "
        << mass.I[10] << "]\n";
}

void URDF2Gazebo::printMass(boost::sharedPtr<urdf::Link> link)
{
  gzdbg << "LINK NAME: [" << link->name << "] from dMass\n";
  gzdbg << "     MASS: [" << link->inertial->mass << "]\n";
  gzdbg << "       CG: [" << link->inertial->origin.position.x << ", "
                          << link->inertial->origin.position.y << ", "
                          << link->inertial->origin.position.z << "]\n";
  gzdbg << "        I: [" << link->inertial->ixx << ", "
                          << link->inertial->ixy << ", "
                          << link->inertial->ixz << "]\n";
  gzdbg << "           [" << link->inertial->ixy << ", "
                          << link->inertial->iyy << ", "
                          << link->inertial->iyz << "]\n";
  gzdbg << "           [" << link->inertial->ixz << ", "
                          << link->inertial->iyz << ", "
                          << link->inertial->izz << "]\n";
}

void URDF2Gazebo::reduceFixedJoints(TiXmlElement *root,
  boost::sharedPtr<urdf::Link> link)
{
  // if child is attached to self by fixed link first go up the tree,
  //   check it's children recursively
  for (unsigned int i = 0 ; i < link->childLinks.size() ; ++i)
    if (link->childLinks[i]->parentJoint->type == urdf::Joint::FIXED)
      reduceFixedJoints(root, link->childLinks[i]);

  // reduce this link's stuff up the tree to parent but skip first joint
  //   if it's the world
  if (link->getParent() && link->getParent()->name != "world" &&
      link->parentJoint && link->parentJoint->type == urdf::Joint::FIXED)
  {
    // gzdbg << "Fixed Joint Reduction: extension lumping from ["
    //       << link->name << "] to [" << link->getParent()->name << "]\n";

    // lump sdf extensions to parent, (give them new reference link names)
    reduceGazeboExtensionToParent(link);

    // reduce link elements to parent
    this->reduceInertialToParent(link);
    this->reduceVisualsToParent(link);
    this->reduceCollisionsToParent(link);
    this->reduceJointsToParent(link);
  }

  // continue down the tree for non-fixed joints
  for (unsigned int i = 0 ; i < link->childLinks.size() ; ++i)
    if (link->childLinks[i]->parentJoint->type != urdf::Joint::FIXED)
      reduceFixedJoints(root, link->childLinks[i]);
}

void URDF2Gazebo::printCollisionGroups(boost::sharedPtr<urdf::Link> link)
{
  gzdbg << "COLLISION LUMPING: link: [" << link->name << "] contains ["
        << staticCast<int>(link->collisionGroups.size())
        << "] collisions.\n";
  for (std::map<std::string,
    boost::sharedPtr<std::vector<CollisionPtr > > >::iterator
    colsIt = link->collisionGroups.begin();
    colsIt != link->collisionGroups.end(); ++colsIt)
  {
    gzdbg << "    collisionGroups: [" << colsIt->first << "] has ["
          << staticCast<int>(colsIt->second->size())
          << "] Collision objects\n";
  }
}

/// \brief returns the same transform in parent frame
urdf::Pose  URDF2Gazebo::transformToParentFrame(
  urdf::Pose transformInLinkFrame, urdf::Pose parentToLinkTransform)
{
  // transform to Pose then call transformToParentFrame
  Pose p1 = URDF2Gazebo::copyPose(transformInLinkFrame);
  Pose p2 = URDF2Gazebo::copyPose(parentToLinkTransform);
  return URDF2Gazebo::copyPose(transformToParentFrame(p1, p2));
}
Pose  URDF2Gazebo::transformToParentFrame(
  Pose transformInLinkFrame,
  urdf::Pose parentToLinkTransform)
{
  // transform to Pose then call transformToParentFrame
  Pose p2 = URDF2Gazebo::copyPose(parentToLinkTransform);
  return transformToParentFrame(transformInLinkFrame, p2);
}

Pose  URDF2Gazebo::transformToParentFrame(
  Pose transformInLinkFrame,
  Pose parentToLinkTransform)
{
  Pose transformInParentLinkFrame;
  // rotate link pose to parentLink frame
  transformInParentLinkFrame.pos =
    parentToLinkTransform.rot * transformInLinkFrame.pos;
  transformInParentLinkFrame.rot =
    parentToLinkTransform.rot * transformInLinkFrame.rot;
  // translate link to parentLink frame
  transformInParentLinkFrame.pos =
    parentToLinkTransform.pos + transformInParentLinkFrame.pos;

  return transformInParentLinkFrame;
}

Pose  URDF2Gazebo::inverseTransformToParentFrame(
  Pose transformInLinkFrame,
  urdf::Pose parentToLinkTransform)
{
  Pose transformInParentLinkFrame;
  //   rotate link pose to parentLink frame
  urdf::Rotation ri = parentToLinkTransform.rotation.GetInverse();
  Quaternion q1(ri.w, ri.x, ri.y, ri.z);
  transformInParentLinkFrame.pos = q1 * transformInLinkFrame.pos;
  urdf::Rotation r2 = parentToLinkTransform.rotation.GetInverse();
  Quaternion q3(r2.w, r2.x, r2.y, r2.z);
  transformInParentLinkFrame.rot = q3 * transformInLinkFrame.rot;
  //   translate link to parentLink frame
  transformInParentLinkFrame.pos.x = transformInParentLinkFrame.pos.x
    - parentToLinkTransform.position.x;
  transformInParentLinkFrame.pos.y = transformInParentLinkFrame.pos.y
    - parentToLinkTransform.position.y;
  transformInParentLinkFrame.pos.z = transformInParentLinkFrame.pos.z
    - parentToLinkTransform.position.z;

  return transformInParentLinkFrame;
}

/// Take the link's existing list of sdf extensions, transfer them
/// into parent link.  Along the way, update local transforms by adding
/// the additional transform to parent.  Also, look through all
/// referenced link names with plugins and update references to current
/// link to the parent link. (reduceGazeboExtensionFrameReplace())
void URDF2Gazebo::reduceGazeboExtensionToParent(
  boost::sharedPtr<urdf::Link> link)
{
  /// @todo: this is a very complicated module that updates the plugins
  /// based on fixed joint reduction really wish this could be a lot cleaner

  std::string linkName = link->name;

  // update extension map with references to linkName
  // this->listGazeboExtensions();
  std::map<std::string, std::vector<GazeboExtension*> >::iterator ext =
    this->sdfExtensions.find(linkName);
  if (ext != this->sdfExtensions.end())
  {
    // gzdbg << "  REDUCE EXTENSION: moving reference from ["
    //       << linkName << "] to [" << link->getParent()->name << "]\n";

    // update reduction transform (for rays, cameras for now).
    //   FIXME: contact frames too?
    for (std::vector<GazeboExtension*>::iterator ge = ext->second.begin();
         ge != ext->second.end(); ++ge)
    {
      (*ge)->reductionTransform = transformToParentFrame(
        (*ge)->reductionTransform,
        link->parentJoint->parentToJointOriginTransform);
      // for sensor and projector blocks only
      reduceGazeboExtensionsTransformReduction((*ge));
    }

    // find pointer to the existing extension with the new link reference
    std::string newLinkName = link->getParent()->name;
    std::map<std::string, std::vector<GazeboExtension*> >::iterator
      newExt = this->sdfExtensions.find(newLinkName);

    // if none exist, create newExtension with newLinkName
    if (newExt == this->sdfExtensions.end())
    {
      std::vector<GazeboExtension*> extensions;
      this->sdfExtensions.insert(std::makePair(
        newLinkName, extensions));
      newExt = this->sdfExtensions.find(newLinkName);
    }

    // move sdf extensions from link into the parent link's extensions
    for (std::vector<GazeboExtension*>::iterator ge = ext->second.begin();
         ge != ext->second.end(); ++ge)
      newExt->second.pushBack(*ge);
    ext->second.clear();
  }

  // for extensions with empty reference, search and replace
  // link name patterns within the plugin with new link name
  // and assign the proper reduction transform for the link name pattern
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       sdfIt = this->sdfExtensions.begin();
       sdfIt != this->sdfExtensions.end(); ++sdfIt)
    {
      // update reduction transform (for contacts, rays, cameras for now).
      for (std::vector<GazeboExtension*>::iterator
        ge = sdfIt->second.begin(); ge != sdfIt->second.end(); ++ge)
        reduceGazeboExtensionFrameReplace(*ge, link);
    }

  // this->listGazeboExtensions();
}

void URDF2Gazebo::reduceGazeboExtensionFrameReplace(GazeboExtension* ge,
  boost::sharedPtr<urdf::Link> link)
{
  // std::string linkName = link->name;
  // std::string newLinkName = link->getParent()->name;
  std::vector<TiXmlElement*> blobs = ge->blobs;
  Pose reductionTransform = ge->reductionTransform;

  // HACK: need to do this more generally, but we also need to replace
  //       all instances of link name with new link name
  //       e.g. contact sensor refers to
  //         <collision>baseLinkCollision</collision>
  //         and it needs to be reparented to
  //         <collision>baseFootprintCollision</collision>
  // gzdbg << "  STRING REPLACE: instances of link name ["
  //       << linkName << "] with [" << newLinkName << "]\n";
  for (std::vector<TiXmlElement*>::iterator blobIt = blobs.begin();
       blobIt != blobs.end(); ++blobIt)
  {
    std::ostringstream debugStreamIn;
    debugStreamIn << *(*blobIt);
    // std::string debugBlob = debugStreamIn.str();
    // gzdbg << "        INITIAL STRING link ["
    //       << linkName << "]-->[" << newLinkName << "]: ["
    //       << debugBlob << "]\n";

    this->reduceGazeboExtensionContactSensorFrameReplace(blobIt, link);
    this->reduceGazeboExtensionPluginFrameReplace(blobIt, link,
      "plugin", "bodyName", reductionTransform);
    this->reduceGazeboExtensionPluginFrameReplace(blobIt, link,
      "plugin", "frameName", reductionTransform);
    this->reduceGazeboExtensionProjectorFrameReplace(blobIt, link);
    this->reduceGazeboExtensionGripperFrameReplace(blobIt, link);
    this->reduceGazeboExtensionJointFrameReplace(blobIt, link);

    std::ostringstream debugStreamOut;
    debugStreamOut << *(*blobIt);
  }
}

void URDF2Gazebo::reduceGazeboExtensionsTransformReduction(GazeboExtension* ge)
{
  Pose reductionTransform = ge->reductionTransform;
  for (std::vector<TiXmlElement*>::iterator blobIt = ge->blobs.begin();
       blobIt != ge->blobs.end(); ++blobIt)
  {
    /// @todo make sure we are not missing any additional transform reductions
    this->reduceGazeboExtensionSensorTransformReduction(blobIt,
      reductionTransform);
    this->reduceGazeboExtensionProjectorTransformReduction(blobIt,
      reductionTransform);
  }
}


void URDF2Gazebo::listGazeboExtensions()
{
  gzdbg << "================================================================\n";
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       sdfIt = this->sdfExtensions.begin();
       sdfIt != this->sdfExtensions.end(); ++sdfIt)
  {
    int extCount = 0;
    for (std::vector<GazeboExtension*>::iterator ge = sdfIt->second.begin();
         ge != sdfIt->second.end(); ++ge)
    {
      if ((*ge)->blobs.size() > 0)
      {
        gzdbg <<  "  PRINTING [" << staticCast<int>((*ge)->blobs.size())
              << "] BLOBS for extension [" << ++extCount
              << "] referencing [" << sdfIt->first << "]\n";
        for (std::vector<TiXmlElement*>::iterator
          blobIt = (*ge)->blobs.begin();
          blobIt != (*ge)->blobs.end(); ++blobIt)
        {
          std::ostringstream streamIn;
          streamIn << *(*blobIt);
          gzdbg << "    BLOB: [" << streamIn.str() << "]\n";
        }
      }
    }
  }
  gzdbg << "================================================================\n";
}

void URDF2Gazebo::listGazeboExtensions(std::string reference)
{
  gzdbg << "================================================================\n";
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       sdfIt = this->sdfExtensions.begin();
       sdfIt != this->sdfExtensions.end(); ++sdfIt)
  {
    if (sdfIt->first == reference)
    {
        gzdbg <<  "  PRINTING [" << staticCast<int>(sdfIt->second.size())
              << "] extensions referencing [" << reference << "]\n";
      for (std::vector<GazeboExtension*>::iterator
           ge = sdfIt->second.begin(); ge != sdfIt->second.end(); ++ge)
      {
        for (std::vector<TiXmlElement*>::iterator
          blobIt = (*ge)->blobs.begin();
          blobIt != (*ge)->blobs.end(); ++blobIt)
        {
          std::ostringstream streamIn;
          streamIn << *(*blobIt);
          gzdbg << "    BLOB: [" << streamIn.str() << "]\n";
        }
      }
    }
  }
  gzdbg << "================================================================\n";
}

void URDF2Gazebo::createSDF(TiXmlElement *root,
  boost::sharedPtr<const urdf::Link> link,
  const Pose &transform)
{
    Pose currentTransform = transform;

    // must have an <inertial> block and cannot have zero mass.
    //  allow det(I) == zero, in the case of point mass geoms.
    // @todo:  keyword "world" should be a constant defined somewhere else
    if (link->name != "world" &&
      ((!link->inertial) || equal(link->inertial->mass, 0.0)))
    {
      if (!link->childLinks.empty())
        sdfwarn << "urdf2sdf: link[" << link->name
               << "] has no inertia, ["
               << staticCast<int>(link->childLinks.size())
               << "] children links ignored\n.";

      if (!link->childJoints.empty())
        sdfwarn << "urdf2sdf: link[" << link->name
               << "] has no inertia, ["
               << staticCast<int>(link->childLinks.size())
               << "] children joints ignored\n.";

      if (link->parentJoint)
        sdfwarn << "urdf2sdf: link[" << link->name
               << "] has no inertia, parent joint [" << link->parentJoint->name
               << "] ignored\n.";

        sdfwarn << "urdf2sdf: link[" << link->name
               << "] has no inertia, not modeled in sdf\n";
      return;
    }

    /* create <body:...> block for non fixed joint attached bodies */
    if ((link->getParent() && link->getParent()->name == "world") ||
        !this->reduceFixedJoints ||
        (!link->parentJoint || link->parentJoint->type != urdf::Joint::FIXED))
      createLink(root, link, currentTransform);

    // recurse into children
    for (unsigned int i = 0 ; i < link->childLinks.size() ; ++i)
        createSDF(root, link->childLinks[i], currentTransform);
}

Pose  URDF2Gazebo::copyPose(urdf::Pose pose)
{
  Pose p;
  p.pos.x = pose.position.x;
  p.pos.y = pose.position.y;
  p.pos.z = pose.position.z;
  p.rot.x = pose.rotation.x;
  p.rot.y = pose.rotation.y;
  p.rot.z = pose.rotation.z;
  p.rot.w = pose.rotation.w;
  return p;
}
urdf::Pose  URDF2Gazebo::copyPose(Pose pose)
{
  urdf::Pose p;
  p.position.x = pose.pos.x;
  p.position.y = pose.pos.y;
  p.position.z = pose.pos.z;
  p.rotation.x = pose.rot.x;
  p.rotation.y = pose.rot.y;
  p.rotation.z = pose.rot.z;
  p.rotation.w = pose.rot.w;
  return p;
}

void URDF2Gazebo::createLink(TiXmlElement *root,
  boost::sharedPtr<const urdf::Link> link,
  Pose &currentTransform)
{
  /* create new body */
  TiXmlElement *elem     = new TiXmlElement("link");

  /* set body name */
  elem->SetAttribute("name", link->name);

  /* compute global transform */
  Pose localTransform;
  // this is the transform from parent link to current link
  // this transform does not exist for the root link
  if (link->parentJoint)
  {
    localTransform = URDF2Gazebo::copyPose(
      link->parentJoint->parentToJointOriginTransform);
    currentTransform = localTransform * currentTransform;
  }
  else
    gzdbg << "[" << link->name << "] has no parent joint\n";

  // create origin tag for this element
  addTransform(elem, currentTransform);

  /* create new inerial block */
  createInertial(elem, link);

  /* create new collision block */
  createCollisions(elem, link);

  /* create new visual block */
  createVisuals(elem, link);

  /* copy sdf extensions data */
  insertGazeboExtensionLink(elem, link->name);

  /* add body to document */
  root->LinkEndChild(elem);

  /* make a <joint:...> block */
  createJoint(root, link, currentTransform);
}

void URDF2Gazebo::createCollisions(TiXmlElement* elem,
  boost::sharedPtr<const urdf::Link> link)
{
  // loop through all collisions. make additional geoms using the lumped stuff
  for (std::map<std::string,
    boost::sharedPtr<std::vector<CollisionPtr> > >::constIterator
    collisionsIt = link->collisionGroups.begin();
    collisionsIt != link->collisionGroups.end(); ++collisionsIt)
  {
    boost::sharedPtr<urdf::Collision> collision =
      *(collisionsIt->second->begin());

    if (collisionsIt->first == "default")
    {
      // gzdbg << "creating default collision for link [" << link->name
      //       << "]";

      /* make a <collision> block */
      createCollision(elem, link, collision, link->name);
    }
    else if (collisionsIt->first.find(std::string("lump::")) == 0)
    {
      // if collision name starts with "lump::", pass through
      //   original parent link name
      // gzdbg << "creating lump collision [" << collisionsIt->first
      //       << "] for link [" << link->name << "].\n";
      /// oldLinkName is the original name before lumping
      std::string oldLinkName = collisionsIt->first.substr(6);
      createCollision(elem, link, collision, oldLinkName);
    }
  }
}

void URDF2Gazebo::createVisuals(TiXmlElement* elem,
  boost::sharedPtr<const urdf::Link> link)
{
  // loop through all visuals. make additional collisions using the
  //   lumped stuff
  for (std::map<std::string,
    boost::sharedPtr<std::vector<VisualPtr> > >::constIterator
    visualsIt = link->visualGroups.begin();
    visualsIt != link->visualGroups.end(); ++visualsIt)
  {
    boost::sharedPtr<urdf::Visual> visual = *(visualsIt->second->begin());

    if (visualsIt->first == "default")
    {
      /* make a visual block */
      createVisual(elem, link, visual, link->name);
    }
    else if (visualsIt->first.find(std::string("lump::")) == 0)
    {
      std::string oldLinkName = visualsIt->first.substr(6);
      /* make a visual block */
      createVisual(elem, link, visual, oldLinkName);
    }
  }
}


void URDF2Gazebo::createInertial(TiXmlElement *elem,
  boost::sharedPtr<const urdf::Link> link)
{
  TiXmlElement *inertial = new TiXmlElement("inertial");

  /* set mass properties */
  // check and print a warning message
  double roll, pitch, yaw;
  link->inertial->origin.rotation.getRPY(roll, pitch, yaw);
  if (!equal(roll, 0.0) ||
    !equal(pitch, 0.0) || !equal(yaw, 0.0))
      sdferr << "rotation of inertial frame in link ["
            << link->name << "] is not supported\n";

  /// add pose
  Pose pose = URDF2Gazebo::copyPose(link->inertial->origin);
  addTransform(inertial, pose);

  // add mass
  addKeyValue(inertial, "mass", values2str(1, &link->inertial->mass));

  // add inertia (ixx, ixy, ixz, iyy, iyz, izz)
  TiXmlElement *inertia = new TiXmlElement("inertia");
  addKeyValue(inertia, "ixx", values2str(1, &link->inertial->ixx));
  addKeyValue(inertia, "ixy", values2str(1, &link->inertial->ixy));
  addKeyValue(inertia, "ixz", values2str(1, &link->inertial->ixz));
  addKeyValue(inertia, "iyy", values2str(1, &link->inertial->iyy));
  addKeyValue(inertia, "iyz", values2str(1, &link->inertial->iyz));
  addKeyValue(inertia, "izz", values2str(1, &link->inertial->izz));
  inertial->LinkEndChild(inertia);

  elem->LinkEndChild(inertial);
}


void URDF2Gazebo::createJoint(TiXmlElement *root,
  boost::sharedPtr<const urdf::Link> link,
  Pose &currentTransform)
{
    /* compute the joint tag */
    std::string jtype;
    jtype.clear();
    if (link->parentJoint != NULL)
    {
      switch (link->parentJoint->type)
      {
        case urdf::Joint::CONTINUOUS:
        case urdf::Joint::REVOLUTE:
            jtype = "revolute";
            break;
        case urdf::Joint::PRISMATIC:
            jtype = "prismatic";
            break;
        case urdf::Joint::FLOATING:
        case urdf::Joint::PLANAR:
            break;
        case urdf::Joint::FIXED:
            jtype = "fixed";
            break;
        default:
            sdfwarn << "Unknown joint type: [" << link->parentJoint->type
                   << "] in link [" << link->name << "]\n";
            break;
      }
    }

    // skip if joint type is fixed and we are not faking it with a hinge,
    //   skip/return with the exception of root link being world,
    //   because there's no lumping there
    if (link->getParent() && link->getParent()->name != "world"
        && jtype == "fixed" && this->reduceFixedJoints) return;

    if (!jtype.empty())
    {
        TiXmlElement *joint = new TiXmlElement("joint");
        if (jtype == "fixed")
          joint->SetAttribute("type", "revolute");
        else
          joint->SetAttribute("type", jtype);
        joint->SetAttribute("name", link->parentJoint->name);
        addKeyValue(joint, "child", link->name);
        addKeyValue(joint, "parent", link->getParent()->name);

        TiXmlElement *jointAxis = new TiXmlElement("axis");
        TiXmlElement *jointAxisLimit = new TiXmlElement("limit");
        TiXmlElement *jointAxisDynamics = new TiXmlElement("dynamics");
        if (jtype == "fixed")
        {
          addKeyValue(jointAxisLimit, "lower", "0");
          addKeyValue(jointAxisLimit, "upper", "0");
          addKeyValue(jointAxisDynamics, "damping", "0");
        }
        else
        {
          Vector3 rotatedJointAxis =
            currentTransform.rot.RotateVector(
            Vector3(link->parentJoint->axis.x,
                                  link->parentJoint->axis.y,
                                  link->parentJoint->axis.z));
          double rotatedJointAxisArray[3] =
            { rotatedJointAxis.x, rotatedJointAxis.y, rotatedJointAxis.z };
          addKeyValue(jointAxis, "xyz", values2str(3, rotatedJointAxisArray));
          if (link->parentJoint->dynamics)
            addKeyValue(jointAxisDynamics, "damping",
              values2str(1, &link->parentJoint->dynamics->damping));

          if (this->enforceLimits && link->parentJoint->limits)
          {
            if (jtype == "slider")
            {
              addKeyValue(jointAxisLimit, "lower",
                values2str(1, &link->parentJoint->limits->lower));
              addKeyValue(jointAxisLimit, "upper",
                values2str(1, &link->parentJoint->limits->upper));
            }
            else if (link->parentJoint->type != urdf::Joint::CONTINUOUS)
            {
              double *lowstop  = &link->parentJoint->limits->lower;
              double *highstop = &link->parentJoint->limits->upper;
              // enforce ode bounds, this will need to be fixed
              if (*lowstop > *highstop)
              {
                sdfwarn << "urdf2sdf: revolute joint ["
                       << link->parentJoint->name
                       << "] with limits: lowStop[" << *lowstop
                       << "] > highStop[" << highstop
                       << "], switching the two.\n";
                double tmp = *lowstop;
                *lowstop = *highstop;
                *highstop = tmp;
              }
              addKeyValue(jointAxisLimit, "lower",
                values2str(1, &link->parentJoint->limits->lower));
              addKeyValue(jointAxisLimit, "upper",
                values2str(1, &link->parentJoint->limits->upper));
            }
          }
        }
        jointAxis->LinkEndChild(jointAxisLimit);
        jointAxis->LinkEndChild(jointAxisDynamics);
        joint->LinkEndChild(jointAxis);

        /* copy sdf extensions data */
        insertGazeboExtensionJoint(joint, link->parentJoint->name);

        /* add joint to document */
        root->LinkEndChild(joint);
    }
}


void URDF2Gazebo::createCollision(TiXmlElement* elem,
  boost::sharedPtr<const urdf::Link> link,
  boost::sharedPtr<urdf::Collision> collision,
  std::string oldLinkName)
{
    /* begin create geometry node, skip if no collision specified */
    TiXmlElement *sdfCollision = new TiXmlElement("collision");

    /* set its name, if lumped, add original link name */
    if (oldLinkName == link->name)
      sdfCollision->SetAttribute("name",
        link->name + std::string("Collision"));
    else
      sdfCollision->SetAttribute("name",
        link->name + std::string("Collision_")+oldLinkName);

    /* set transform */
    double pose[6];
    pose[0] = collision->origin.position.x;
    pose[1] = collision->origin.position.y;
    pose[2] = collision->origin.position.z;
    collision->origin.rotation.getRPY(pose[3], pose[4], pose[5]);
    addKeyValue(sdfCollision, "pose", values2str(6, pose));


    /* add geometry block */
    if (!collision || !collision->geometry)
    {
      // gzdbg << "urdf2sdf: collision of link [" << link->name
      //       << "] has no <geometry>.\n";
    }
    else
    {
      createGeometry(sdfCollision, collision->geometry);
    }

    /* set additional data from extensions */
    insertGazeboExtensionCollision(sdfCollision, oldLinkName);

    /* add geometry to body */
    elem->LinkEndChild(sdfCollision);
}

void URDF2Gazebo::createVisual(TiXmlElement *elem,
  boost::sharedPtr<const urdf::Link> link,
  boost::sharedPtr<urdf::Visual> visual, std::string oldLinkName)
{
    /* begin create sdf visual node */
    TiXmlElement *sdfVisual = new TiXmlElement("visual");

    /* set its name */
    // gzdbg << "original link name [" << oldLinkName
    //       << "] new link name [" << link->name << "]\n";
    if (oldLinkName == link->name)
      sdfVisual->SetAttribute("name", link->name + std::string("Vis"));
    else
      sdfVisual->SetAttribute("name", link->name + std::string("Vis_")
        + oldLinkName);

    /* add the visualisation transfrom */
    double pose[6];
    pose[0] = visual->origin.position.x;
    pose[1] = visual->origin.position.y;
    pose[2] = visual->origin.position.z;
    visual->origin.rotation.getRPY(pose[3], pose[4], pose[5]);
    addKeyValue(sdfVisual, "pose", values2str(6, pose));

    /* insert geometry */
    if (!visual || !visual->geometry)
    {
      // gzdbg << "urdf2sdf: visual of link [" << link->name
      //       << "] has no <geometry>\n.";
    }
    else
      createGeometry(sdfVisual, visual->geometry);

    /* set additional data from extensions */
    insertGazeboExtensionVisual(sdfVisual, oldLinkName);

    /* end create visual node */
    elem->LinkEndChild(sdfVisual);
}

TiXmlDocument URDF2Gazebo::initModelString(std::string urdfStr,
  bool EnforceLimits)
{
    this->enforceLimits = EnforceLimits;
    return this->initModelString(urdfStr);
}

TiXmlDocument URDF2Gazebo::initModelString(std::string urdfStr)
{
    /* Create a RobotModel from string */
    boost::sharedPtr<urdf::ModelInterface> robotModel =
      urdf::parseURDF(urdfStr.cStr());

    // an xml object to hold the xml result
    TiXmlDocument sdfXmlOut;

    if (!robotModel)
    {
        sdferr << "Unable to call parseURDF on robot model\n";
        return sdfXmlOut;
    }

    /* create root element and define needed namespaces */
    TiXmlElement *robot = new TiXmlElement("model");

    // set model name to urdf robot name if not specified
    robot->SetAttribute("name", robotModel->getName());

    /* initialize transform for the model, urdf is recursive,
       while sdf defines all links relative to model frame */
    Pose transform;

    /* parse sdf extension */
    TiXmlDocument urdfXml;
    urdfXml.Parse(urdfStr.cStr());
    parseGazeboExtension(urdfXml);

    boost::sharedPtr<const urdf::Link> rootLink = robotModel->getRoot();

    /* Fixed Joint Reduction */
    /* if link connects to parent via fixed joint, lump down and remove link */
    /* set reduceFixedJoints to false will replace fixed joints with
       zero limit revolute joints, otherwise, we reduce it down to its
       parent link recursively */
    if (this->reduceFixedJoints)
      reduceFixedJoints(robot,
        (boost::constPointerCast< urdf::Link >(rootLink)));

    if (rootLink->name == "world")
    {
      /* convert all children link */
      for (std::vector<boost::sharedPtr<urdf::Link> >::constIterator
        child = rootLink->childLinks.begin();
        child != rootLink->childLinks.end(); ++child)
          createSDF(robot, (*child), transform);
    }
    else
    {
      /* convert, starting from root link */
      createSDF(robot, rootLink, transform);
    }

    /* insert the extensions without reference into <robot> root level */
    insertGazeboExtensionRobot(robot);

    // add robot to sdfXmlOut
    TiXmlElement *sdfSdf = new TiXmlElement("sdf");
    sdfSdf->SetAttribute("version", "1.2");
    sdfSdf->LinkEndChild(robot);
    sdfXmlOut.LinkEndChild(sdfSdf);

    // debug
    // sdfXmlOut.Print();

    return sdfXmlOut;
}

TiXmlDocument URDF2Gazebo::initModelDoc(TiXmlDocument* XmlDoc)
{
    std::ostringstream stream;
    stream << *XmlDoc;
    std::string urdfStr = stream.str();
    return initModelString(urdfStr);
}

TiXmlDocument URDF2Gazebo::initModelFile(std::string filename)
{
  TiXmlDocument xmlDoc;
  if (xmlDoc.LoadFile(filename))
  {
    return initModelDoc(&xmlDoc);
  }
  else
    sdferr << "Unable to load file[" << filename << "].\n";

    return xmlDoc;
}

void URDF2Gazebo::reduceInertialToParent(boost::sharedPtr<urdf::Link> link)
{
    // gzdbg << "TREE:   mass lumping from [" << link->name
    //      << "] to [" << link->getParent()->name << "]\n.";
    /* now lump all contents of this link to parent */
    if (link->inertial)
    {
      // get parent mass (in parent link frame)
      dMass parentMass;
      if (!link->getParent()->inertial)
        link->getParent()->inertial.reset(new urdf::Inertial);
      dMassSetParameters(&parentMass, link->getParent()->inertial->mass,
        link->getParent()->inertial->origin.position.x,
        link->getParent()->inertial->origin.position.y,
        link->getParent()->inertial->origin.position.z,
        link->getParent()->inertial->ixx, link->getParent()->inertial->iyy,
        link->getParent()->inertial->izz, link->getParent()->inertial->ixy,
         link->getParent()->inertial->ixz, link->getParent()->inertial->iyz);
      // printMass(link->getParent()->name, parentMass);
      // printMass(link->getParent());
      // set link mass (in link frame)
      dMass linkMass;
      dMassSetParameters(&linkMass, link->inertial->mass,
        link->inertial->origin.position.x,
        link->inertial->origin.position.y,
        link->inertial->origin.position.z,
        link->inertial->ixx, link->inertial->iyy, link->inertial->izz,
        link->inertial->ixy, link->inertial->ixz, link->inertial->iyz);
      // printMass(link->name, linkMass);
      // printMass(link);
      // un-rotate link mass into parent link frame
      dMatrix3 R;
      double phi, theta, psi;
      link->parentJoint->parentToJointOriginTransform.rotation.getRPY(
        phi, theta, psi);
      dRFromEulerAngles(R, phi, theta, psi);
      dMassRotate(&linkMass, R);
      // printMass(link->name, linkMass);
      // un-translate link mass into parent link frame
      dMassTranslate(&linkMass,
        link->parentJoint->parentToJointOriginTransform.position.x,
        link->parentJoint->parentToJointOriginTransform.position.y,
        link->parentJoint->parentToJointOriginTransform.position.z);
      // printMass(link->name, linkMass);
      // now linkMass is in the parent frame, add linkMass to parentMass
      dMassAdd(&parentMass, &linkMass);
      // printMass(link->getParent()->name, parentMass);
      // update parent mass
      link->getParent()->inertial->mass = parentMass.mass;
      link->getParent()->inertial->ixx  = parentMass.I[0+4*0];
      link->getParent()->inertial->iyy  = parentMass.I[1+4*1];
      link->getParent()->inertial->izz  = parentMass.I[2+4*2];
      link->getParent()->inertial->ixy  = parentMass.I[0+4*1];
      link->getParent()->inertial->ixz  = parentMass.I[0+4*2];
      link->getParent()->inertial->iyz  = parentMass.I[1+4*2];
      link->getParent()->inertial->origin.position.x  = parentMass.c[0];
      link->getParent()->inertial->origin.position.y  = parentMass.c[1];
      link->getParent()->inertial->origin.position.z  = parentMass.c[2];
      // printMass(link->getParent());
    }
}

void URDF2Gazebo::reduceVisualsToParent(boost::sharedPtr<urdf::Link> link)
{
  // lump visual to parent
  // lump all visual to parent, assign group name
  // "lump::"+group name+"::'+link name
  // lump but keep the link name in(/as) the group name,
  // so we can correlate visuals to visuals somehow.
  for (std::map<std::string,
    boost::sharedPtr<std::vector<VisualPtr> > >::iterator
    visualsIt = link->visualGroups.begin();
    visualsIt != link->visualGroups.end(); ++visualsIt)
  {
    /// @todo: extend to different groups,
    /// only work with default meshes right now.
    if (visualsIt->first == "default")
    {
      std::string lumpGroupName = std::string("lump::")+link->name;
      // gzdbg << "adding modified lump group name [" << lumpGroupName
      //       << "] to link [" << link->getParent()->name << "]\n.";
      for (std::vector<boost::sharedPtr<urdf::Visual> >::iterator
        visualIt = visualsIt->second->begin();
        visualIt != visualsIt->second->end(); ++visualIt)
      {
        // transform visual origin from link frame to
        // parent link frame before adding to parent
        (*visualIt)->origin = transformToParentFrame((*visualIt)->origin,
          link->parentJoint->parentToJointOriginTransform);
        // add the modified visual to parent
        this->reduceVisualToParent(link->getParent(), lumpGroupName,
          *visualIt);
      }
    }
    else if (visualsIt->first.find(std::string("lump::")) == 0)
    {
      // it's a previously lumped mesh, re-lump under same groupName
      std::string lumpGroupName = visualsIt->first;
      // gzdbg << "re-lumping group name [" << lumpGroupName
      //       << "] to link [" << link->getParent()->name << "]\n";
      for (std::vector<boost::sharedPtr<urdf::Visual> >::iterator
           visualIt = visualsIt->second->begin();
           visualIt != visualsIt->second->end(); ++visualIt)
      {
        // transform visual origin from link frame to parent link
        // frame before adding to parent
        (*visualIt)->origin = transformToParentFrame((*visualIt)->origin,
          link->parentJoint->parentToJointOriginTransform);
        // add the modified visual to parent
        this->reduceVisualToParent(link->getParent(), lumpGroupName,
          *visualIt);
      }
    }
  }
}

void URDF2Gazebo::reduceCollisionsToParent(boost::sharedPtr<urdf::Link> link)
{
    // lump collision parent
    // lump all collision to parent, assign group name
    // "lump::"+group name+"::'+link name
    // lump but keep the link name in(/as) the group name,
    // so we can correlate visuals to collisions somehow.
    for (std::map<std::string,
      boost::sharedPtr<std::vector<CollisionPtr> > >::iterator
      collisionsIt = link->collisionGroups.begin();
      collisionsIt != link->collisionGroups.end(); ++collisionsIt)
    {
      if (collisionsIt->first == "default")
      {
        // if it's a "default" mesh, it will be added under "lump::"+link name
        std::string lumpGroupName = std::string("lump::")+link->name;
        // gzdbg << "lumping collision [" << collisionsIt->first
        //       << "] for link [" << link->name
        //       << "] to parent [" << link->getParent()->name
        //       << "] with group name [" << lumpGroupName << "]\n";
        for (std::vector<boost::sharedPtr<urdf::Collision> >::iterator
          collisionIt = collisionsIt->second->begin();
          collisionIt != collisionsIt->second->end(); ++collisionIt)
        {
          // transform collision origin from link frame to
          // parent link frame before adding to parent
          (*collisionIt)->origin = transformToParentFrame(
            (*collisionIt)->origin,
            link->parentJoint->parentToJointOriginTransform);

          // add the modified collision to parent
          this->reduceCollisionToParent(link->getParent(), lumpGroupName,
            *collisionIt);
        }
      }
      else if (collisionsIt->first.find(std::string("lump::")) == 0)
      {
        // if it's a previously lumped mesh, relump under same groupName
        std::string lumpGroupName = collisionsIt->first;
        // gzdbg << "re-lumping collision [" << collisionsIt->first
        //       << "] for link [" << link->name
        //       << "] to parent [" << link->getParent()->name
        //       << "] with group name [" << lumpGroupName << "]\n";
        for (std::vector<boost::sharedPtr<urdf::Collision> >::iterator
          collisionIt = collisionsIt->second->begin();
          collisionIt != collisionsIt->second->end(); ++collisionIt)
        {
          // transform collision origin from link frame to
          // parent link frame before adding to parent
          (*collisionIt)->origin = transformToParentFrame(
            (*collisionIt)->origin,
            link->parentJoint->parentToJointOriginTransform);
          // add the modified collision to parent
          this->reduceCollisionToParent(link->getParent(), lumpGroupName,
            *collisionIt);
        }
      }
    }
    // printCollisionGroups(link->getParent());
}

void URDF2Gazebo::reduceJointsToParent(boost::sharedPtr<urdf::Link> link)
{
    // set child link's parentJoint's parent link to
    // a parent link up stream that does not have a fixed parentJoint
    for (unsigned int i = 0 ; i < link->childLinks.size() ; ++i)
    {
      boost::sharedPtr<urdf::Joint> parentJoint =
        link->childLinks[i]->parentJoint;
      if (parentJoint->type != urdf::Joint::FIXED)
      {
        // go down the tree until we hit a parent joint that is not fixed
        boost::sharedPtr<urdf::Link> newParentLink = link;
        Pose jointAnchorTransform;
        while (newParentLink->parentJoint &&
              newParentLink->getParent()->name != "world" &&
              newParentLink->parentJoint->type == urdf::Joint::FIXED)
        {
          jointAnchorTransform = jointAnchorTransform *
            jointAnchorTransform;
          parentJoint->parentToJointOriginTransform =
            transformToParentFrame(
            parentJoint->parentToJointOriginTransform,
            newParentLink->parentJoint->parentToJointOriginTransform);
          newParentLink = newParentLink->getParent();
        }
        // now set the link->childLinks[i]->parentJoint's parent link to
        // the newParentLink
        link->childLinks[i]->setParent(newParentLink);
        parentJoint->parentLinkName = newParentLink->name;
        // and set the link->childLinks[i]->parentJoint's
        // parentToJointOroginTransform as the aggregated anchor transform?
      }
    }
}

void URDF2Gazebo::reduceGazeboExtensionSensorTransformReduction(
  std::vector<TiXmlElement*>::iterator blobIt,
  Pose reductionTransform)
{
    // overwrite <xyz> and <rpy> if they exist
    if ((*blobIt)->ValueStr() == "sensor")
    {
      /*
      // parse it and add/replace the reduction transform
      // find first instance of xyz and rpy, replace with reduction transform
      for (TiXmlNode* elIt = (*blobIt)->FirstChild();
           elIt; elIt = elIt->NextSibling())
      {
        std::ostringstream streamIn;
        streamIn << *elIt;
        gzdbg << "    " << streamIn << "\n";
      }
      */

      {
        TiXmlNode* oldPoseKey = (*blobIt)->FirstChild("pose");
        /// @todo: FIXME:  we should read xyz, rpy and aggregate it to
        /// reductionTransform instead of just throwing the info away.
        if (oldPoseKey)
          (*blobIt)->RemoveChild(oldPoseKey);
      }

      // convert reductionTransform to values
      urdf::Vector3 reductionXyz(reductionTransform.pos.x,
        reductionTransform.pos.y,
        reductionTransform.pos.z);
      urdf::Rotation reductionQ(reductionTransform.rot.x,
        reductionTransform.rot.y,
        reductionTransform.rot.z, reductionTransform.rot.w);

      urdf::Vector3 reductionRpy;
      reductionQ.getRPY(reductionRpy.x, reductionRpy.y, reductionRpy.z);

      // output updated pose to text
      std::ostringstream poseStream;
      poseStream << reductionXyz.x << " " << reductionXyz.y
                  << " " << reductionXyz.z << " " << reductionRpy.x
                  << " " << reductionRpy.y << " " << reductionRpy.z;
      TiXmlText* poseTxt = new TiXmlText(poseStream.str());

      TiXmlElement* poseKey = new TiXmlElement("pose");
      poseKey->LinkEndChild(poseTxt);

      (*blobIt)->LinkEndChild(poseKey);
    }
}

void URDF2Gazebo::reduceGazeboExtensionProjectorTransformReduction(
  std::vector<TiXmlElement*>::iterator blobIt,
  Pose reductionTransform)
{
    // overwrite <pose> (xyz/rpy) if it exists
    if ((*blobIt)->ValueStr() == "projector")
    {
      /*
      // parse it and add/replace the reduction transform
      // find first instance of xyz and rpy, replace with reduction transform
      for (TiXmlNode* elIt = (*blobIt)->FirstChild();
        elIt; elIt = elIt->NextSibling())
      {
        std::ostringstream streamIn;
        streamIn << *elIt;
        gzdbg << "    " << streamIn << "\n";
      }
      */

      /* should read <pose>...</pose> and agregate reductionTransform */
      TiXmlNode* poseKey = (*blobIt)->FirstChild("pose");
      // read pose and save it

      // remove the tag for now
      if (poseKey) (*blobIt)->RemoveChild(poseKey);

      // convert reductionTransform to values
      urdf::Vector3 reductionXyz(reductionTransform.pos.x,
        reductionTransform.pos.y,
        reductionTransform.pos.z);
      urdf::Rotation reductionQ(reductionTransform.rot.x,
        reductionTransform.rot.y,
        reductionTransform.rot.z,
        reductionTransform.rot.w);

      urdf::Vector3 reductionRpy;
      reductionQ.getRPY(reductionRpy.x, reductionRpy.y, reductionRpy.z);

      // output updated pose to text
      std::ostringstream poseStream;
      poseStream << reductionXyz.x << " " << reductionXyz.y
                  << " " << reductionXyz.z << " " << reductionRpy.x
                  << " " << reductionRpy.y << " " << reductionRpy.z;
      TiXmlText* poseTxt = new TiXmlText(poseStream.str());

      poseKey = new TiXmlElement("pose");
      poseKey->LinkEndChild(poseTxt);

      (*blobIt)->LinkEndChild(poseKey);
    }
}

void URDF2Gazebo::reduceGazeboExtensionContactSensorFrameReplace(
  std::vector<TiXmlElement*>::iterator blobIt,
  boost::sharedPtr<urdf::Link> link)
{
  std::string linkName = link->name;
  std::string newLinkName = link->getParent()->name;
  if ((*blobIt)->ValueStr() == "sensor")
  {
    // parse it and add/replace the reduction transform
    // find first instance of xyz and rpy, replace with reduction transform
    TiXmlNode* contact = (*blobIt)->FirstChild("contact");
    if (contact)
    {
      TiXmlNode* collision = contact->FirstChild("collision");
      if (collision)
      {
        if (getKeyValueAsString(collision->ToElement()) ==
          linkName + std::string("Collision"))
        {
          contact->RemoveChild(collision);
          TiXmlElement* collisionNameKey = new TiXmlElement("collision");
          std::ostringstream collisionNameStream;
          collisionNameStream << newLinkName << "Collision_" << linkName;
          TiXmlText* collisionNameTxt = new TiXmlText(
            collisionNameStream.str());
          collisionNameKey->LinkEndChild(collisionNameTxt);
          contact->LinkEndChild(collisionNameKey);
        }
        // @todo: FIXME: chagning contact sensor's contact collision
        //   should trigger a update in sensor offset as well.
        //   But first we need to implement offsets in contact sensors
      }
    }
  }
}

void URDF2Gazebo::reduceGazeboExtensionPluginFrameReplace(
  std::vector<TiXmlElement*>::iterator blobIt,
  boost::sharedPtr<urdf::Link> link,
  std::string pluginName, std::string elementName,
  Pose reductionTransform)
{
  std::string linkName = link->name;
  std::string newLinkName = link->getParent()->name;
  if ((*blobIt)->ValueStr() == pluginName)
  {
    // replace element containing link names to parent link names
    // find first instance of xyz and rpy, replace with reduction transform
    TiXmlNode* elementNode = (*blobIt)->FirstChild(elementName);
    if (elementNode)
    {
      if (getKeyValueAsString(elementNode->ToElement()) == linkName)
      {
        (*blobIt)->RemoveChild(elementNode);
        TiXmlElement* bodyNameKey = new TiXmlElement(elementName);
        std::ostringstream bodyNameStream;
        bodyNameStream << newLinkName;
        TiXmlText* bodyNameTxt = new TiXmlText(bodyNameStream.str());
        bodyNameKey->LinkEndChild(bodyNameTxt);
        (*blobIt)->LinkEndChild(bodyNameKey);
        /// @todo update transforms for this sdf plugin too

        // look for offset transforms, add reduction transform
        TiXmlNode* xyzKey = (*blobIt)->FirstChild("xyzOffset");
        if (xyzKey)
        {
          urdf::Vector3 v1 = parseVector3(xyzKey);
          reductionTransform.pos = Vector3(v1.x, v1.y, v1.z);
          // remove xyzOffset and rpyOffset
          (*blobIt)->RemoveChild(xyzKey);
        }
        TiXmlNode* rpyKey = (*blobIt)->FirstChild("rpyOffset");
        if (rpyKey)
        {
          urdf::Vector3 rpy = parseVector3(rpyKey, M_PI/180.0);
          reductionTransform.rot =
            Quaternion::EulerToQuaternion(rpy.x, rpy.y, rpy.z);
          // remove xyzOffset and rpyOffset
          (*blobIt)->RemoveChild(rpyKey);
        }

        // pass through the parent transform from fixed joint reduction
        reductionTransform = inverseTransformToParentFrame(reductionTransform,
          link->parentJoint->parentToJointOriginTransform);

        // create new offset xml blocks
        xyzKey = new TiXmlElement("xyzOffset");
        rpyKey = new TiXmlElement("rpyOffset");

        // create new offset xml blocks
        urdf::Vector3 reductionXyz(reductionTransform.pos.x,
          reductionTransform.pos.y,
          reductionTransform.pos.z);
        urdf::Rotation reductionQ(reductionTransform.rot.x,
          reductionTransform.rot.y, reductionTransform.rot.z,
          reductionTransform.rot.w);

        std::ostringstream xyzStream, rpyStream;
        xyzStream << reductionXyz.x << " " << reductionXyz.y << " "
                   << reductionXyz.z;
        urdf::Vector3 reductionRpy;
        reductionQ.getRPY(reductionRpy.x, reductionRpy.y, reductionRpy.z);
        rpyStream << reductionRpy.x << " " << reductionRpy.y << " "
                   << reductionRpy.z;

        TiXmlText* xyzTxt = new TiXmlText(xyzStream.str());
        TiXmlText* rpyTxt = new TiXmlText(rpyStream.str());

        xyzKey->LinkEndChild(xyzTxt);
        rpyKey->LinkEndChild(rpyTxt);

        (*blobIt)->LinkEndChild(xyzKey);
        (*blobIt)->LinkEndChild(rpyKey);
      }
    }
  }
}

void URDF2Gazebo::reduceGazeboExtensionProjectorFrameReplace(
  std::vector<TiXmlElement*>::iterator blobIt,
  boost::sharedPtr<urdf::Link> link)
{
  std::string linkName = link->name;
  std::string newLinkName = link->getParent()->name;

  // updates link reference for <projector> inside of
  // projector plugins
  // update from <projector>MyLinkName/MyProjectorName</projector>
  // to <projector>NewLinkName/MyProjectorName</projector>
  TiXmlNode* projectorElem = (*blobIt)->FirstChild("projector");
  {
    if (projectorElem)
    {
      std::string projectorName =  getKeyValueAsString(
        projectorElem->ToElement());
      // extract projector link name and projector name
      sizeT pos = projectorName.find("/");
      if (pos == std::string::npos)
        sdferr << "no slash in projector reference tag [" << projectorName
              << "], expecting linkName/projectorName.\n";
      std::string projectorLinkName = projectorName.substr(0, pos);

      if (projectorLinkName == linkName)
      {
        // do the replacement
        projectorName = newLinkName + "/" +
          projectorName.substr(pos+1, projectorName.size());

        (*blobIt)->RemoveChild(projectorElem);
        TiXmlElement* bodyNameKey = new TiXmlElement("projector");
        std::ostringstream bodyNameStream;
        bodyNameStream << projectorName;
        TiXmlText* bodyNameTxt = new TiXmlText(bodyNameStream.str());
        bodyNameKey->LinkEndChild(bodyNameTxt);
        (*blobIt)->LinkEndChild(bodyNameKey);
      }
    }
  }
}

void URDF2Gazebo::reduceGazeboExtensionGripperFrameReplace(
  std::vector<TiXmlElement*>::iterator blobIt,
  boost::sharedPtr<urdf::Link> link)
{
  std::string linkName = link->name;
  std::string newLinkName = link->getParent()->name;

  if ((*blobIt)->ValueStr() == "gripper")
  {
    TiXmlNode* gripperLink = (*blobIt)->FirstChild("gripperLink");
    if (gripperLink)
    {
      if (getKeyValueAsString(gripperLink->ToElement()) == linkName)
      {
        (*blobIt)->RemoveChild(gripperLink);
        TiXmlElement* bodyNameKey = new TiXmlElement("gripperLink");
        std::ostringstream bodyNameStream;
        bodyNameStream << newLinkName;
        TiXmlText* bodyNameTxt = new TiXmlText(bodyNameStream.str());
        bodyNameKey->LinkEndChild(bodyNameTxt);
        (*blobIt)->LinkEndChild(bodyNameKey);
      }
    }
    TiXmlNode* palmLink = (*blobIt)->FirstChild("palmLink");
    if (palmLink)
    {
      if (getKeyValueAsString(palmLink->ToElement()) == linkName)
      {
        (*blobIt)->RemoveChild(palmLink);
        TiXmlElement* bodyNameKey = new TiXmlElement("palmLink");
        std::ostringstream bodyNameStream;
        bodyNameStream << newLinkName;
        TiXmlText* bodyNameTxt = new TiXmlText(bodyNameStream.str());
        bodyNameKey->LinkEndChild(bodyNameTxt);
        (*blobIt)->LinkEndChild(bodyNameKey);
      }
    }
  }
}

void URDF2Gazebo::reduceGazeboExtensionJointFrameReplace(
  std::vector<TiXmlElement*>::iterator blobIt,
  boost::sharedPtr<urdf::Link> link)
{
  std::string linkName = link->name;
  std::string newLinkName = link->getParent()->name;

  if ((*blobIt)->ValueStr() == "joint")
  {
    // parse it and add/replace the reduction transform
    // find first instance of xyz and rpy, replace with reduction transform
    TiXmlNode* parent = (*blobIt)->FirstChild("parent");
    if (parent)
    {
      if (getKeyValueAsString(parent->ToElement()) == linkName)
      {
        (*blobIt)->RemoveChild(parent);
        TiXmlElement* parentNameKey = new TiXmlElement("parent");
        std::ostringstream parentNameStream;
        parentNameStream << newLinkName;
        TiXmlText* parentNameTxt = new TiXmlText(parentNameStream.str());
        parentNameKey->LinkEndChild(parentNameTxt);
        (*blobIt)->LinkEndChild(parentNameKey);
      }
    }
    TiXmlNode* child = (*blobIt)->FirstChild("child");
    if (child)
    {
      if (getKeyValueAsString(child->ToElement()) == linkName)
      {
        (*blobIt)->RemoveChild(child);
        TiXmlElement* childNameKey = new TiXmlElement("child");
        std::ostringstream childNameStream;
        childNameStream << newLinkName;
        TiXmlText* childNameTxt = new TiXmlText(childNameStream.str());
        childNameKey->LinkEndChild(childNameTxt);
        (*blobIt)->LinkEndChild(childNameKey);
      }
    }
    /// @todo add anchor offsets if parent link changes location!
  }
}
}
