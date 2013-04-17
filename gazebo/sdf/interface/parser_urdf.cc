/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <urdf_parser/urdf_parser.h>

#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>

#include "gazebo/sdf/interface/parser_urdf.hh"
#include "gazebo/sdf/sdf.hh"
#include "gazebo/common/SystemPaths.hh"

namespace urdf2gazebo
{
////////////////////////////////////////////////////////////////////////////////
std::string lowerStr(std::string str)
{
  std::string out = str;
  std::transform(out.begin(), out.end(), out.begin(), ::tolower);
  return out;
}

////////////////////////////////////////////////////////////////////////////////
URDF2Gazebo::URDF2Gazebo()
{
    // default options
    this->enforceLimits = true;
    this->reduceFixedJoints = true;
}

////////////////////////////////////////////////////////////////////////////////
URDF2Gazebo::~URDF2Gazebo()
{
}

////////////////////////////////////////////////////////////////////////////////
urdf::Vector3 URDF2Gazebo::ParseVector3(TiXmlNode* _key, double _scale)
{
  if (_key != NULL)
  {
    std::string str = _key->Value();
    std::vector<std::string> pieces;
    std::vector<double> vals;

    boost::split(pieces, str, boost::is_any_of(" "));
    for (unsigned int i = 0; i < pieces.size(); ++i)
    {
      if (pieces[i] != "")
      {
        try
        {
          vals.push_back(_scale
                         * boost::lexical_cast<double>(pieces[i].c_str()));
        }
        catch(boost::bad_lexical_cast &e)
        {
          gzerr << "xml key [" << str
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

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceVisualToParent(UrdfLinkPtr _link,
       const std::string &_groupName, UrdfVisualPtr _visual)
{
  boost::shared_ptr<std::vector<UrdfVisualPtr> > viss
    = _link->getVisuals(_groupName);
  if (!viss)
  {
    // group does not exist, create one and add to map
    viss.reset(new std::vector<UrdfVisualPtr>);
    // new group name, create vector, add vector to map and
    //   add Visual to the vector
    _link->visual_groups.insert(make_pair(_groupName, viss));
    // gzdbg << "successfully added a new visual group name ["
    //       << _groupName << "]\n";
  }

  // group exists, add Visual to the vector in the map if it's not there
  std::vector<UrdfVisualPtr>::iterator visIt
    = find(viss->begin(), viss->end(), _visual);
  if (visIt != viss->end())
    gzwarn << "attempted to add visual to link ["
           << _link->name
           << "], but it already exists under group ["
           << _groupName << "]\n";
  else
    viss->push_back(_visual);
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceCollisionToParent(UrdfLinkPtr _link,
      const std::string &_groupName, UrdfCollisionPtr _collision)
{
  boost::shared_ptr<std::vector<UrdfCollisionPtr> >
    cols = _link->getCollisions(_groupName);
  if (!cols)
  {
    // group does not exist, create one and add to map
    cols.reset(new std::vector<UrdfCollisionPtr>);
    // new group name, create add vector to map and add Collision to the vector
    _link->collision_groups.insert(make_pair(_groupName, cols));
  }

  // group exists, add Collision to the vector in the map
  std::vector<UrdfCollisionPtr>::iterator colIt =
    find(cols->begin(), cols->end(), _collision);
  if (colIt != cols->end())
    gzwarn << "attempted to add collision to link ["
           << _link->name
           << "], but it already exists under group ["
           << _groupName << "]\n";
  else
    cols->push_back(_collision);
}

////////////////////////////////////////////////////////////////////////////////
std::string URDF2Gazebo::Vector32Str(const urdf::Vector3 _vector)
{
  std::stringstream ss;
  ss << _vector.x;
  ss << " ";
  ss << _vector.y;
  ss << " ";
  ss << _vector.z;
  return ss.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string URDF2Gazebo::Values2str(unsigned int _count, const double *_values)
{
  std::stringstream ss;
  for (unsigned int i = 0 ; i < _count ; ++i)
  {
      if (i > 0)
          ss << " ";
      ss << _values[i];
  }
  return ss.str();
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::AddKeyValue(TiXmlElement *_elem, const std::string &_key,
  const std::string &_value)
{
  TiXmlElement* childElem = _elem->FirstChildElement(_key);
  if (childElem)
  {
    std::string oldValue = this->GetKeyValueAsString(childElem);
    if (oldValue != _value)
      gzwarn << "multiple inconsistent <" << _key
             << "> exists due to fixed joint reduction"
             << " overwriting previous value [" << oldValue
             << "] with [" << _value << "].\n";
    // else
    //   gzdbg << "multiple consistent <" << _key
    //          << "> exists with [" << _value
    //          << "] due to fixed joint reduction.\n";
    _elem->RemoveChild(childElem);  // remove old _elem
  }

  TiXmlElement *ekey      = new TiXmlElement(_key);
  TiXmlText    *textEkey = new TiXmlText(_value);
  ekey->LinkEndChild(textEkey);
  _elem->LinkEndChild(ekey);
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::AddTransform(TiXmlElement *_elem,
  const::gazebo::math::Pose& _transform)
{
  gazebo::math::Vector3 e = _transform.rot.GetAsEuler();
  double cpose[6] = { _transform.pos.x, _transform.pos.y,
                      _transform.pos.z, e.x, e.y, e.z };

  /* set geometry transform */
  this->AddKeyValue(_elem, "pose", this->Values2str(6, cpose));
}

////////////////////////////////////////////////////////////////////////////////
std::string URDF2Gazebo::GetKeyValueAsString(TiXmlElement* _elem)
{
  std::string valueStr;
  if (_elem->Attribute("value"))
  {
    valueStr = _elem->Attribute("value");
  }
  else if (_elem->FirstChild())
  /// @todo: FIXME: comment out check for now, different tinyxml
  /// versions fails to compile:
  //  && _elem->FirstChild()->Type() == TiXmlNode::TINYXML_TEXT)
  {
    valueStr = _elem->FirstChild()->ValueStr();
  }
  return valueStr;
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ParseGazeboExtension(TiXmlDocument &_urdfXml)
{
  TiXmlElement* robotXml = _urdfXml.FirstChildElement("robot");

  // Get all Gazebo extension elements, put everything in
  //   this->extensions map, containing a key string
  //   (link/joint name) and values
  for (TiXmlElement* gazeboXml = robotXml->FirstChildElement("gazebo");
       gazeboXml; gazeboXml = gazeboXml->NextSiblingElement("gazebo"))
  {
    const char* ref = gazeboXml->Attribute("reference");
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

    if (this->extensions.find(refStr) ==
        this->extensions.end())
    {
        // create extension map for reference
        std::vector<GazeboExtension*> ge;
        this->extensions.insert(std::make_pair(refStr, ge));
    }

    // create and insert a new GazeboExtension into the map
    GazeboExtension* gazebo = new GazeboExtension();

    // begin parsing xml node
    for (TiXmlElement *childElem = gazeboXml->FirstChildElement();
         childElem; childElem = childElem->NextSiblingElement())
    {
      gazebo->oldLinkName = refStr;

      // go through all elements of the extension,
      //   extract what we know, and save the rest in blobs
      // @todo:  somehow use sdf definitions here instead of hard coded
      //         objects

      // material
      if (childElem->ValueStr() == "material")
      {
          gazebo->material = this->GetKeyValueAsString(childElem);
      }
      else if (childElem->ValueStr() == "static")
      {
        std::string valueStr = this->GetKeyValueAsString(childElem);

        // default of setting static flag is false
        if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
            valueStr == "1")
          gazebo->setStaticFlag = true;
        else
          gazebo->setStaticFlag = false;
      }
      else if (childElem->ValueStr() == "gravity")
      {
        std::string valueStr = this->GetKeyValueAsString(childElem);

        // default of gravity is true
        if (lowerStr(valueStr) == "false" || lowerStr(valueStr) == "no" ||
            valueStr == "0")
          gazebo->gravity = false;
        else
          gazebo->gravity = true;
      }
      else if (childElem->ValueStr() == "dampingFactor")
      {
          gazebo->isDampingFactor = true;
          gazebo->dampingFactor = boost::lexical_cast<double>(
              this->GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "maxVel")
      {
          gazebo->isMaxVel = true;
          gazebo->maxVel = boost::lexical_cast<double>(
              this->GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "minDepth")
      {
          gazebo->isMinDepth = true;
          gazebo->minDepth = boost::lexical_cast<double>(
            this->GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "mu1")
      {
          gazebo->isMu1 = true;
          gazebo->mu1 = boost::lexical_cast<double>(
            this->GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "mu2")
      {
          gazebo->isMu2 = true;
          gazebo->mu2 = boost::lexical_cast<double>(
            this->GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "fdir1")
      {
          gazebo->fdir1 = this->GetKeyValueAsString(childElem);
      }
      else if (childElem->ValueStr() == "kp")
      {
          gazebo->isKp = true;
          gazebo->kp = boost::lexical_cast<double>(
            this->GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "kd")
      {
          gazebo->isKd = true;
          gazebo->kd = boost::lexical_cast<double>(
            this->GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "selfCollide")
      {
        std::string valueStr = this->GetKeyValueAsString(childElem);

        // default of selfCollide is false
        if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
            valueStr == "1")
          gazebo->selfCollide = true;
        else
          gazebo->selfCollide = false;
      }
      else if (childElem->ValueStr() == "laserRetro")
      {
          gazebo->isLaserRetro = true;
          gazebo->laserRetro = boost::lexical_cast<double>(
            this->GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "stopCfm")
      {
          gazebo->isStopCfm = true;
          gazebo->stopCfm = boost::lexical_cast<double>(
            this->GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "stopErp")
      {
          gazebo->isStopErp = true;
          gazebo->stopErp = boost::lexical_cast<double>(
            this->GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "initialJointPosition")
      {
          gazebo->isInitialJointPosition = true;
          gazebo->initialJointPosition = boost::lexical_cast<double>(
            this->GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "fudgeFactor")
      {
          gazebo->isFudgeFactor = true;
          gazebo->fudgeFactor = boost::lexical_cast<double>(
            this->GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "provideFeedback")
      {
          std::string valueStr = this->GetKeyValueAsString(childElem);

          if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
              valueStr == "1")
            gazebo->provideFeedback = true;
          else
            gazebo->provideFeedback = false;
      }
      else if (childElem->ValueStr() == "cfmDamping")
      {
          std::string valueStr = this->GetKeyValueAsString(childElem);

          if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
              valueStr == "1")
            gazebo->cfmDamping = true;
          else
            gazebo->cfmDamping = false;
      }
      else
      {
          std::ostringstream stream;
          stream << *childElem;
          // save all unknown stuff in a vector of blobs
          TiXmlElement *blob = new TiXmlElement(*childElem);
          gazebo->blobs.push_back(blob);
      }
    }

    // insert into my map
    (this->extensions.find(refStr))->second.push_back(gazebo);
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::InsertGazeboExtensionCollision(TiXmlElement *_elem,
  const std::string &_linkName)
{
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazeboIt = this->extensions.begin();
       gazeboIt != this->extensions.end(); ++gazeboIt)
  {
    for (std::vector<GazeboExtension*>::iterator ge = gazeboIt->second.begin();
         ge != gazeboIt->second.end(); ++ge)
    {
      if ((*ge)->oldLinkName == _linkName)
      {
        TiXmlElement *surface = new TiXmlElement("surface");
        TiXmlElement *friction = new TiXmlElement("friction");
        TiXmlElement *frictionOde = new TiXmlElement("ode");
        TiXmlElement *contact = new TiXmlElement("contact");
        TiXmlElement *contactOde = new TiXmlElement("ode");

        // insert mu1, mu2, kp, kd for collision
        if ((*ge)->isMu1)
          this->AddKeyValue(frictionOde, "mu",
                            this->Values2str(1, &(*ge)->mu1));
        if ((*ge)->isMu2)
          this->AddKeyValue(frictionOde, "mu2",
                            this->Values2str(1, &(*ge)->mu2));
        if (!(*ge)->fdir1.empty())
          this->AddKeyValue(frictionOde, "fdir1", (*ge)->fdir1);
        if ((*ge)->isKp)
          this->AddKeyValue(contactOde, "kp", this->Values2str(1, &(*ge)->kp));
        if ((*ge)->isKd)
          this->AddKeyValue(contactOde, "kd", this->Values2str(1, &(*ge)->kd));
        // max contact interpenetration correction velocity
        if ((*ge)->isMaxVel)
          this->AddKeyValue(contactOde, "max_vel",
                      this->Values2str(1, &(*ge)->maxVel));
        // contact interpenetration margin tolerance
        if ((*ge)->isMinDepth)
          this->AddKeyValue(contactOde, "min_depth",
                      this->Values2str(1, &(*ge)->minDepth));
        if ((*ge)->isLaserRetro)
          this->AddKeyValue(_elem, "laser_retro",
                      this->Values2str(1, &(*ge)->laserRetro));

        contact->LinkEndChild(contactOde);
        surface->LinkEndChild(contact);
        friction->LinkEndChild(frictionOde);
        surface->LinkEndChild(friction);
        _elem->LinkEndChild(surface);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::InsertGazeboExtensionVisual(TiXmlElement *_elem,
  const std::string &_linkName)
{
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazeboIt = this->extensions.begin();
       gazeboIt != this->extensions.end(); ++gazeboIt)
  {
    for (std::vector<GazeboExtension*>::iterator ge = gazeboIt->second.begin();
         ge != gazeboIt->second.end(); ++ge)
    {
      if ((*ge)->oldLinkName == _linkName)
      {
        // insert material block
        if (!(*ge)->material.empty())
            this->AddKeyValue(_elem, "material", (*ge)->material);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::InsertGazeboExtensionLink(TiXmlElement *_elem,
  const std::string &_linkName)
{
    for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
         gazeboIt = this->extensions.begin();
         gazeboIt != this->extensions.end(); ++gazeboIt)
    {
      if (gazeboIt->first == _linkName)
      {
        // gzdbg << "inserting extension with reference ["
        //       << _linkName << "] into link.\n";
        for (std::vector<GazeboExtension*>::iterator ge =
             gazeboIt->second.begin(); ge != gazeboIt->second.end(); ++ge)
        {
          // insert gravity
          if ((*ge)->gravity)
              this->AddKeyValue(_elem, "gravity", "true");
          else
              this->AddKeyValue(_elem, "gravity", "false");

          // damping factor
          TiXmlElement *velocityDecay = new TiXmlElement("velocity_decay");
          if ((*ge)->isDampingFactor)
          {
            /// @todo separate linear and angular velocity decay
            this->AddKeyValue(velocityDecay, "linear",
                        this->Values2str(1, &(*ge)->dampingFactor));
            this->AddKeyValue(velocityDecay, "angular",
                        this->Values2str(1, &(*ge)->dampingFactor));
          }
          _elem->LinkEndChild(velocityDecay);
          // selfCollide tag
          if ((*ge)->selfCollide)
              this->AddKeyValue(_elem, "self_collide", "true");
          else
              this->AddKeyValue(_elem, "self_collide", "false");
          // insert blobs into body
          for (std::vector<TiXmlElement*>::iterator
               blobIt = (*ge)->blobs.begin();
               blobIt != (*ge)->blobs.end(); ++blobIt)
          {
              _elem->LinkEndChild((*blobIt)->Clone());
          }
        }
      }
    }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::InsertGazeboExtensionJoint(TiXmlElement *_elem,
  const std::string &_jointName)
{
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazeboIt = this->extensions.begin();
       gazeboIt != this->extensions.end(); ++gazeboIt)
  {
    if (gazeboIt->first == _jointName)
    {
      for (std::vector<GazeboExtension*>::iterator
           ge = gazeboIt->second.begin();
           ge != gazeboIt->second.end(); ++ge)
      {
        TiXmlElement *physics     = new TiXmlElement("physics");
        TiXmlElement *physicsOde = new TiXmlElement("ode");
        TiXmlElement *limit       = new TiXmlElement("limit");
        // insert stopCfm, stopErp, fudgeFactor
        if ((*ge)->isStopCfm)
        {
          this->AddKeyValue(limit, "erp", this->Values2str(1, &(*ge)->stopCfm));
        }
        if ((*ge)->isStopErp)
        {
          this->AddKeyValue(limit, "cfm", this->Values2str(1, &(*ge)->stopErp));
        }
        /* gone
        if ((*ge)->isInitialJointPosition)
            this->AddKeyValue(_elem, "initialJointPosition",
              this->Values2str(1, &(*ge)->initialJointPosition));
        */

        // insert provideFeedback
        if ((*ge)->provideFeedback)
            this->AddKeyValue(physicsOde, "provide_feedback", "true");
        else
            this->AddKeyValue(physicsOde, "provide_feedback", "false");

        // insert cfmDamping
        if ((*ge)->cfmDamping)
            this->AddKeyValue(physicsOde, "cfm_damping", "true");
        else
            this->AddKeyValue(physicsOde, "cfm_damping", "false");

        // insert fudgeFactor
        if ((*ge)->isFudgeFactor)
          this->AddKeyValue(physicsOde, "fudge_factor",
                      this->Values2str(1, &(*ge)->fudgeFactor));

        physics->LinkEndChild(physicsOde);
        physicsOde->LinkEndChild(limit);
        _elem->LinkEndChild(physics);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::InsertGazeboExtensionRobot(TiXmlElement *_elem)
{
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazeboIt = this->extensions.begin();
       gazeboIt != this->extensions.end(); ++gazeboIt)
  {
    if (gazeboIt->first.empty())
    {
      // no reference specified
      for (std::vector<GazeboExtension*>::iterator
        ge = gazeboIt->second.begin(); ge != gazeboIt->second.end(); ++ge)
      {
        // insert static flag
        if ((*ge)->setStaticFlag)
            this->AddKeyValue(_elem, "static", "true");
        else
            this->AddKeyValue(_elem, "static", "false");

        // copy extension containing blobs and without reference
        for (std::vector<TiXmlElement*>::iterator
             blobIt = (*ge)->blobs.begin();
             blobIt != (*ge)->blobs.end(); ++blobIt)
        {
            std::ostringstream streamIn;
            streamIn << *(*blobIt);
            _elem->LinkEndChild((*blobIt)->Clone());
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::CreateGeometry(TiXmlElement* _elem,
  boost::shared_ptr<urdf::Geometry> _geom)
{
  int sizeCount;
  double sizeVals[3];

  TiXmlElement *gazeboGeometry = new TiXmlElement("geometry");

  std::string type;
  TiXmlElement *geometryType = NULL;

  switch (_geom->type)
  {
  case urdf::Geometry::BOX:
    type = "box";
    sizeCount = 3;
    {
      boost::shared_ptr<const urdf::Box> box;
      box = boost::dynamic_pointer_cast< const urdf::Box >(_geom);
      sizeVals[0] = box->dim.x;
      sizeVals[1] = box->dim.y;
      sizeVals[2] = box->dim.z;
      geometryType = new TiXmlElement(type);
      this->AddKeyValue(geometryType, "size",
                        this->Values2str(sizeCount, sizeVals));
    }
    break;
  case urdf::Geometry::CYLINDER:
    type = "cylinder";
    sizeCount = 2;
    {
      boost::shared_ptr<const urdf::Cylinder> cylinder;
      cylinder = boost::dynamic_pointer_cast<const urdf::Cylinder >(_geom);
      geometryType = new TiXmlElement(type);
      this->AddKeyValue(geometryType, "length",
                  this->Values2str(1, &cylinder->length));
      this->AddKeyValue(geometryType, "radius",
                  this->Values2str(1, &cylinder->radius));
    }
    break;
  case urdf::Geometry::SPHERE:
    type = "sphere";
    sizeCount = 1;
    {
      boost::shared_ptr<const urdf::Sphere> sphere;
      sphere = boost::dynamic_pointer_cast<const urdf::Sphere >(_geom);
      geometryType = new TiXmlElement(type);
      this->AddKeyValue(geometryType, "radius",
                  this->Values2str(1, &sphere->radius));
    }
    break;
  case urdf::Geometry::MESH:
    type = "mesh";
    sizeCount = 3;
    {
      boost::shared_ptr<const urdf::Mesh> mesh;
      mesh = boost::dynamic_pointer_cast<const urdf::Mesh >(_geom);
      sizeVals[0] = mesh->scale.x;
      sizeVals[1] = mesh->scale.y;
      sizeVals[2] = mesh->scale.z;
      geometryType = new TiXmlElement(type);
      this->AddKeyValue(geometryType, "scale", this->Vector32Str(mesh->scale));
      // do something more to meshes
      {
        /* set mesh file */
        if (mesh->filename.empty())
        {
            gzerr << "urdf2gazebo: mesh geometry with no filename given.\n";
        }

        // give some warning if file does not exist.
        // disabled while switching to uri
        // @todo: re-enable check
        // std::ifstream fin;
        // fin.open(mesh->filename.c_str(), std::ios::in);
        // fin.close();
        // if (fin.fail())
        //   gzwarn << "filename referred by mesh ["
        //          << mesh->filename << "] does not appear to exist.\n";

        // Convert package:// to model://,
        // in ROS, this will work if
        // the model package is in ROS_PACKAGE_PATH and has a manifest.xml
        // as a typical ros package does.
        std::string modelFilename = mesh->filename;
        std::string packagePrefix("package://");
        std::string modelPrefix("model://");
        size_t pos1 = modelFilename.find(packagePrefix, 0);
        if (pos1 != std::string::npos)
        {
          size_t repLen = packagePrefix.size();
          modelFilename.replace(pos1, repLen, modelPrefix);
          // gzwarn << "ros style uri [package://] is"
          //   << "automatically converted: [" << modelFilename
          //   << "], make sure your ros package is in GAZEBO_MODEL_PATH"
          //   << " and switch your manifest to conform to gazebo's"
          //   << " model database format.  See ["
          //   << "http://gazebosim.org/wiki/Model_database#Model_Manifest_XML"
          //   << "] for more info.\n";
        }

        // add mesh filename
        this->AddKeyValue(geometryType, "uri", modelFilename);
      }
    }
    break;
  default:
    sizeCount = 0;
    gzwarn << "Unknown body type: [" << _geom->type
           << "] skipped in geometry\n";
    break;
  }

  if (geometryType)
  {
    gazeboGeometry->LinkEndChild(geometryType);
    _elem->LinkEndChild(gazeboGeometry);
  }
}

////////////////////////////////////////////////////////////////////////////////
std::string URDF2Gazebo::GetGeometryBoundingBox(
  boost::shared_ptr<urdf::Geometry> _geom, double *_sizeVals)
{
  std::string type;

  switch (_geom->type)
  {
  case urdf::Geometry::BOX:
      type = "box";
      {
        boost::shared_ptr<const urdf::Box> box;
        box = boost::dynamic_pointer_cast<const urdf::Box >(_geom);
        _sizeVals[0] = box->dim.x;
        _sizeVals[1] = box->dim.y;
        _sizeVals[2] = box->dim.z;
      }
      break;
  case urdf::Geometry::CYLINDER:
      type = "cylinder";
      {
        boost::shared_ptr<const urdf::Cylinder> cylinder;
        cylinder = boost::dynamic_pointer_cast<const urdf::Cylinder >(_geom);
        _sizeVals[0] = cylinder->radius * 2;
        _sizeVals[1] = cylinder->radius * 2;
        _sizeVals[2] = cylinder->length;
      }
      break;
  case urdf::Geometry::SPHERE:
      type = "sphere";
      {
        boost::shared_ptr<const urdf::Sphere> sphere;
        sphere = boost::dynamic_pointer_cast<const urdf::Sphere >(_geom);
        _sizeVals[0] = _sizeVals[1] = _sizeVals[2] = sphere->radius * 2;
      }
      break;
  case urdf::Geometry::MESH:
      type = "trimesh";
      {
        boost::shared_ptr<const urdf::Mesh> mesh;
        mesh = boost::dynamic_pointer_cast<const urdf::Mesh >(_geom);
        _sizeVals[0] = mesh->scale.x;
        _sizeVals[1] = mesh->scale.y;
        _sizeVals[2] = mesh->scale.z;
      }
      break;
  default:
      _sizeVals[0] = _sizeVals[1] = _sizeVals[2] = 0;
      gzwarn << "Unknown body type: [" << _geom->type
             << "] skipped in geometry\n";
      break;
  }

  return type;
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::PrintMass(const std::string &_linkName, dMass _mass)
{
  gzdbg << "LINK NAME: [" << _linkName << "] from dMass\n";
  gzdbg << "     MASS: [" << _mass.mass << "]\n";
  gzdbg << "       CG: [" << _mass.c[0] << ", " << _mass.c[1] << ", "
        << _mass.c[2] << "]\n";
  gzdbg << "        I: [" << _mass.I[0] << ", " << _mass.I[1] << ", "
        << _mass.I[2] << "]\n";
  gzdbg << "           [" << _mass.I[4] << ", " << _mass.I[5] << ", "
        << _mass.I[6] << "]\n";
  gzdbg << "           [" << _mass.I[8] << ", " << _mass.I[9] << ", "
        << _mass.I[10] << "]\n";
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::PrintMass(UrdfLinkPtr _link)
{
  gzdbg << "LINK NAME: [" << _link->name << "] from dMass\n";
  gzdbg << "     MASS: [" << _link->inertial->mass << "]\n";
  gzdbg << "       CG: [" << _link->inertial->origin.position.x << ", "
                          << _link->inertial->origin.position.y << ", "
                          << _link->inertial->origin.position.z << "]\n";
  gzdbg << "        I: [" << _link->inertial->ixx << ", "
                          << _link->inertial->ixy << ", "
                          << _link->inertial->ixz << "]\n";
  gzdbg << "           [" << _link->inertial->ixy << ", "
                          << _link->inertial->iyy << ", "
                          << _link->inertial->iyz << "]\n";
  gzdbg << "           [" << _link->inertial->ixz << ", "
                          << _link->inertial->iyz << ", "
                          << _link->inertial->izz << "]\n";
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceFixedJoints(TiXmlElement *_root, UrdfLinkPtr _link)
{
  // if child is attached to self by fixed _link first go up the tree,
  //   check it's children recursively
  for (unsigned int i = 0 ; i < _link->child_links.size() ; ++i)
    if (_link->child_links[i]->parent_joint->type == urdf::Joint::FIXED)
      this->ReduceFixedJoints(_root, _link->child_links[i]);

  // reduce this _link's stuff up the tree to parent but skip first joint
  //   if it's the world
  if (_link->getParent() && _link->getParent()->name != "world" &&
      _link->parent_joint && _link->parent_joint->type == urdf::Joint::FIXED)
  {
    // gzdbg << "Fixed Joint Reduction: extension lumping from ["
    //       << _link->name << "] to [" << _link->getParent()->name << "]\n";

    // lump gazebo extensions to parent, (give them new reference _link names)
    ReduceGazeboExtensionToParent(_link);

    // reduce _link elements to parent
    this->ReduceInertialToParent(_link);
    this->ReduceVisualsToParent(_link);
    this->ReduceCollisionsToParent(_link);
    this->ReduceJointsToParent(_link);
  }

  // continue down the tree for non-fixed joints
  for (unsigned int i = 0 ; i < _link->child_links.size() ; ++i)
    if (_link->child_links[i]->parent_joint->type != urdf::Joint::FIXED)
      this->ReduceFixedJoints(_root, _link->child_links[i]);
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::PrintCollisionGroups(UrdfLinkPtr _link)
{
  gzdbg << "COLLISION LUMPING: link: [" << _link->name << "] contains ["
        << static_cast<int>(_link->collision_groups.size())
        << "] collisions.\n";
  for (std::map<std::string,
    boost::shared_ptr<std::vector<UrdfCollisionPtr > > >::iterator
    colsIt = _link->collision_groups.begin();
    colsIt != _link->collision_groups.end(); ++colsIt)
  {
    gzdbg << "    collision_groups: [" << colsIt->first << "] has ["
          << static_cast<int>(colsIt->second->size())
          << "] Collision objects\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
urdf::Pose  URDF2Gazebo::TransformToParentFrame(
  urdf::Pose _transformInLinkFrame, urdf::Pose _parentToLinkTransform)
{
  // transform to gazebo::math::Pose then call TransformToParentFrame
  gazebo::math::Pose p1 = URDF2Gazebo::CopyPose(_transformInLinkFrame);
  gazebo::math::Pose p2 = URDF2Gazebo::CopyPose(_parentToLinkTransform);
  return URDF2Gazebo::CopyPose(this->TransformToParentFrame(p1, p2));
}

////////////////////////////////////////////////////////////////////////////////
gazebo::math::Pose  URDF2Gazebo::TransformToParentFrame(
  gazebo::math::Pose _transformInLinkFrame,
  urdf::Pose _parentToLinkTransform)
{
  // transform to gazebo::math::Pose then call TransformToParentFrame
  gazebo::math::Pose p2 = URDF2Gazebo::CopyPose(_parentToLinkTransform);
  return this->TransformToParentFrame(_transformInLinkFrame, p2);
}

////////////////////////////////////////////////////////////////////////////////
gazebo::math::Pose  URDF2Gazebo::TransformToParentFrame(
  gazebo::math::Pose _transformInLinkFrame,
  gazebo::math::Pose _parentToLinkTransform)
{
  gazebo::math::Pose transformInParentLinkFrame;
  // rotate link pose to parentLink frame
  transformInParentLinkFrame.pos =
    _parentToLinkTransform.rot * _transformInLinkFrame.pos;
  transformInParentLinkFrame.rot =
    _parentToLinkTransform.rot * _transformInLinkFrame.rot;
  // translate link to parentLink frame
  transformInParentLinkFrame.pos =
    _parentToLinkTransform.pos + transformInParentLinkFrame.pos;

  return transformInParentLinkFrame;
}

////////////////////////////////////////////////////////////////////////////////
gazebo::math::Pose  URDF2Gazebo::inverseTransformToParentFrame(
  gazebo::math::Pose _transformInLinkFrame,
  urdf::Pose _parentToLinkTransform)
{
  gazebo::math::Pose transformInParentLinkFrame;
  //   rotate link pose to parentLink frame
  urdf::Rotation ri = _parentToLinkTransform.rotation.GetInverse();
  gazebo::math::Quaternion q1(ri.w, ri.x, ri.y, ri.z);
  transformInParentLinkFrame.pos = q1 * _transformInLinkFrame.pos;
  urdf::Rotation r2 = _parentToLinkTransform.rotation.GetInverse();
  gazebo::math::Quaternion q3(r2.w, r2.x, r2.y, r2.z);
  transformInParentLinkFrame.rot = q3 * _transformInLinkFrame.rot;
  //   translate link to parentLink frame
  transformInParentLinkFrame.pos.x = transformInParentLinkFrame.pos.x
    - _parentToLinkTransform.position.x;
  transformInParentLinkFrame.pos.y = transformInParentLinkFrame.pos.y
    - _parentToLinkTransform.position.y;
  transformInParentLinkFrame.pos.z = transformInParentLinkFrame.pos.z
    - _parentToLinkTransform.position.z;

  return transformInParentLinkFrame;
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceGazeboExtensionToParent(UrdfLinkPtr _link)
{
  /// @todo: this is a very complicated module that updates the plugins
  /// based on fixed joint reduction really wish this could be a lot cleaner

  std::string linkName = _link->name;

  // update extension map with references to linkName
  // this->ListGazeboExtensions();
  std::map<std::string, std::vector<GazeboExtension*> >::iterator ext =
    this->extensions.find(linkName);
  if (ext != this->extensions.end())
  {
    // gzdbg << "  REDUCE EXTENSION: moving reference from ["
    //       << linkName << "] to [" << _link->getParent()->name << "]\n";

    // update reduction transform (for rays, cameras for now).
    //   FIXME: contact frames too?
    for (std::vector<GazeboExtension*>::iterator ge = ext->second.begin();
         ge != ext->second.end(); ++ge)
    {
      (*ge)->reductionTransform = this->TransformToParentFrame(
        (*ge)->reductionTransform,
        _link->parent_joint->parent_to_joint_origin_transform);
      // for sensor and projector blocks only
      ReduceGazeboExtensionsTransform((*ge));
    }

    // find pointer to the existing extension with the new _link reference
    std::string newLinkName = _link->getParent()->name;
    std::map<std::string, std::vector<GazeboExtension*> >::iterator
      newExt = this->extensions.find(newLinkName);

    // if none exist, create new extension with newLinkName
    if (newExt == this->extensions.end())
    {
      std::vector<GazeboExtension*> ge;
      this->extensions.insert(std::make_pair(
        newLinkName, ge));
      newExt = this->extensions.find(newLinkName);
    }

    // move gazebo extensions from _link into the parent _link's extensions
    for (std::vector<GazeboExtension*>::iterator ge = ext->second.begin();
         ge != ext->second.end(); ++ge)
      newExt->second.push_back(*ge);
    ext->second.clear();
  }

  // for extensions with empty reference, search and replace
  // _link name patterns within the plugin with new _link name
  // and assign the proper reduction transform for the _link name pattern
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazeboIt = this->extensions.begin();
       gazeboIt != this->extensions.end(); ++gazeboIt)
    {
      // update reduction transform (for contacts, rays, cameras for now).
      for (std::vector<GazeboExtension*>::iterator
        ge = gazeboIt->second.begin(); ge != gazeboIt->second.end(); ++ge)
        ReduceGazeboExtensionFrameReplace(*ge, _link);
    }

  // this->ListGazeboExtensions();
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceGazeboExtensionFrameReplace(GazeboExtension* _ge,
                                                    UrdfLinkPtr _link)
{
  // std::string linkName = _link->name;
  // std::string newLinkName = _link->getParent()->name;

  // HACK: need to do this more generally, but we also need to replace
  //       all instances of _link name with new link name
  //       e.g. contact sensor refers to
  //         <collision>base_link_collision</collision>
  //         and it needs to be reparented to
  //         <collision>base_footprint_collision</collision>
  // gzdbg << "  STRING REPLACE: instances of _link name ["
  //       << linkName << "] with [" << newLinkName << "]\n";
  for (std::vector<TiXmlElement*>::iterator blobIt = _ge->blobs.begin();
       blobIt != _ge->blobs.end(); ++blobIt)
  {
    std::ostringstream debugStreamIn;
    debugStreamIn << *(*blobIt);
    // std::string debugBlob = debugStreamIn.str();
    // gzdbg << "        INITIAL STRING link ["
    //       << linkName << "]-->[" << newLinkName << "]: ["
    //       << debugBlob << "]\n";

    this->ReduceGazeboExtensionContactSensorFrameReplace(blobIt, _link);
    this->ReduceGazeboExtensionPluginFrameReplace(blobIt, _link,
      "plugin", "bodyName", _ge->reductionTransform);
    this->ReduceGazeboExtensionPluginFrameReplace(blobIt, _link,
      "plugin", "frameName", _ge->reductionTransform);
    this->ReduceGazeboExtensionProjectorFrameReplace(blobIt, _link);
    this->ReduceGazeboExtensionGripperFrameReplace(blobIt, _link);
    this->ReduceGazeboExtensionJointFrameReplace(blobIt, _link);

    std::ostringstream debugStreamOut;
    debugStreamOut << *(*blobIt);
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceGazeboExtensionsTransform(GazeboExtension* _ge)
{
  for (std::vector<TiXmlElement*>::iterator blobIt = _ge->blobs.begin();
       blobIt != _ge->blobs.end(); ++blobIt)
  {
    /// @todo make sure we are not missing any additional transform reductions
    this->ReduceGazeboExtensionSensorTransformReduction(blobIt,
      _ge->reductionTransform);
    this->ReduceGazeboExtensionProjectorTransformReduction(blobIt,
      _ge->reductionTransform);
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ListGazeboExtensions()
{
  gzdbg << "================================================================\n";
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazeboIt = this->extensions.begin();
       gazeboIt != this->extensions.end(); ++gazeboIt)
  {
    int extCount = 0;
    for (std::vector<GazeboExtension*>::iterator ge = gazeboIt->second.begin();
         ge != gazeboIt->second.end(); ++ge)
    {
      if ((*ge)->blobs.size() > 0)
      {
        gzdbg <<  "  PRINTING [" << static_cast<int>((*ge)->blobs.size())
              << "] BLOBS for extension [" << ++extCount
              << "] referencing [" << gazeboIt->first << "]\n";
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

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ListGazeboExtensions(const std::string &_reference)
{
  gzdbg << "================================================================\n";
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazeboIt = this->extensions.begin();
       gazeboIt != this->extensions.end(); ++gazeboIt)
  {
    if (gazeboIt->first == _reference)
    {
        gzdbg <<  "  PRINTING [" << static_cast<int>(gazeboIt->second.size())
              << "] extensions referencing [" << _reference << "]\n";
      for (std::vector<GazeboExtension*>::iterator
           ge = gazeboIt->second.begin(); ge != gazeboIt->second.end(); ++ge)
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

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::CreateSDF(TiXmlElement *_root,
  ConstUrdfLinkPtr _link,
  const gazebo::math::Pose &_transform)
{
    gazebo::math::Pose _currentTransform = _transform;

    // must have an <inertial> block and cannot have zero mass.
    //  allow det(I) == zero, in the case of point mass geoms.
    // @todo:  keyword "world" should be a constant defined somewhere else
    if (_link->name != "world" &&
      ((!_link->inertial) || gazebo::math::equal(_link->inertial->mass, 0.0)))
    {
      if (!_link->child_links.empty())
        gzwarn << "urdf2gazebo: link[" << _link->name
               << "] has no inertia, ["
               << static_cast<int>(_link->child_links.size())
               << "] children links ignored\n.";

      if (!_link->child_joints.empty())
        gzwarn << "urdf2gazebo: link[" << _link->name
               << "] has no inertia, ["
               << static_cast<int>(_link->child_links.size())
               << "] children joints ignored\n.";

      if (_link->parent_joint)
        gzwarn << "urdf2gazebo: link[" << _link->name
               << "] has no inertia, "
               << "parent joint [" << _link->parent_joint->name
               << "] ignored\n.";

        gzwarn << "urdf2gazebo: link[" << _link->name
               << "] has no inertia, not modeled in gazebo\n";
      return;
    }

    /* create <body:...> block for non fixed joint attached bodies */
    if ((_link->getParent() && _link->getParent()->name == "world") ||
        !this->reduceFixedJoints ||
        (!_link->parent_joint ||
         _link->parent_joint->type != urdf::Joint::FIXED))
      CreateLink(_root, _link, _currentTransform);

    // recurse into children
    for (unsigned int i = 0 ; i < _link->child_links.size() ; ++i)
        CreateSDF(_root, _link->child_links[i], _currentTransform);
}

////////////////////////////////////////////////////////////////////////////////
gazebo::math::Pose  URDF2Gazebo::CopyPose(urdf::Pose _pose)
{
  gazebo::math::Pose p;
  p.pos.x = _pose.position.x;
  p.pos.y = _pose.position.y;
  p.pos.z = _pose.position.z;
  p.rot.x = _pose.rotation.x;
  p.rot.y = _pose.rotation.y;
  p.rot.z = _pose.rotation.z;
  p.rot.w = _pose.rotation.w;
  return p;
}

////////////////////////////////////////////////////////////////////////////////
urdf::Pose  URDF2Gazebo::CopyPose(gazebo::math::Pose _pose)
{
  urdf::Pose p;
  p.position.x = _pose.pos.x;
  p.position.y = _pose.pos.y;
  p.position.z = _pose.pos.z;
  p.rotation.x = _pose.rot.x;
  p.rotation.y = _pose.rot.y;
  p.rotation.z = _pose.rot.z;
  p.rotation.w = _pose.rot.w;
  return p;
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::CreateLink(TiXmlElement *_root,
  ConstUrdfLinkPtr _link,
  gazebo::math::Pose &_currentTransform)
{
  /* create new body */
  TiXmlElement *elem     = new TiXmlElement("link");

  /* set body name */
  elem->SetAttribute("name", _link->name);

  /* compute global transform */
  gazebo::math::Pose localTransform;
  // this is the transform from parent link to current _link
  // this transform does not exist for the root link
  if (_link->parent_joint)
  {
    localTransform = URDF2Gazebo::CopyPose(
      _link->parent_joint->parent_to_joint_origin_transform);
    _currentTransform = localTransform * _currentTransform;
  }
  else
    gzdbg << "[" << _link->name << "] has no parent joint\n";

  // create origin tag for this element
  this->AddTransform(elem, _currentTransform);

  /* create new inerial block */
  this->CreateInertial(elem, _link);

  /* create new collision block */
  this->CreateCollisions(elem, _link);

  /* create new visual block */
  this->CreateVisuals(elem, _link);

  /* copy gazebo extensions data */
  this->InsertGazeboExtensionLink(elem, _link->name);

  /* add body to document */
  _root->LinkEndChild(elem);

  /* make a <joint:...> block */
  this->CreateJoint(_root, _link, _currentTransform);
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::CreateCollisions(TiXmlElement* _elem,
  ConstUrdfLinkPtr _link)
{
  // loop through all collision groups. as well as additional collision from
  //   lumped meshes (fixed joint reduction)
  for (std::map<std::string,
    boost::shared_ptr<std::vector<UrdfCollisionPtr> > >::const_iterator
    collisionsIt = _link->collision_groups.begin();
    collisionsIt != _link->collision_groups.end(); ++collisionsIt)
  {
    unsigned int defaultMeshCount = 0;
    unsigned int groupMeshCount = 0;
    unsigned int lumpMeshCount = 0;
    // loop through collisions in each group
    for (std::vector<UrdfCollisionPtr>::iterator
         collision = collisionsIt->second->begin();
         collision != collisionsIt->second->end();
         ++collision)
    {
      if (collisionsIt->first == "default")
      {
        // gzdbg << "creating default collision for link [" << _link->name
        //       << "]";

        std::string collisionPrefix = _link->name;

        if (defaultMeshCount > 0)
        {
          // append _[meshCount] to link name for additional collisions
          std::ostringstream collisionNameStream;
          collisionNameStream << collisionPrefix << "_" << defaultMeshCount;
          collisionPrefix = collisionNameStream.str();
        }

        /* make a <collision> block */
        this->CreateCollision(_elem, _link, *collision, collisionPrefix);

        // only 1 default mesh
        ++defaultMeshCount;
      }
      else if (collisionsIt->first.find(std::string("lump::")) == 0)
      {
        // if collision name starts with "lump::", pass through
        //   original parent link name
        // gzdbg << "creating lump collision [" << collisionsIt->first
        //       << "] for link [" << _link->name << "].\n";
        /// collisionPrefix is the original name before lumping
        std::string collisionPrefix = collisionsIt->first.substr(6);

        if (lumpMeshCount > 0)
        {
          // append _[meshCount] to link name for additional collisions
          std::ostringstream collisionNameStream;
          collisionNameStream << collisionPrefix << "_" << lumpMeshCount;
          collisionPrefix = collisionNameStream.str();
        }

        this->CreateCollision(_elem, _link, *collision, collisionPrefix);
        ++lumpMeshCount;
      }
      else
      {
        // gzdbg << "adding collisions from collision group ["
        //      << collisionsIt->first << "]\n";

        std::string collisionPrefix = _link->name + std::string("_") +
                                      collisionsIt->first;

        if (groupMeshCount > 0)
        {
          // append _[meshCount] to _link name for additional collisions
          std::ostringstream collisionNameStream;
          collisionNameStream << collisionPrefix << "_" << groupMeshCount;
          collisionPrefix = collisionNameStream.str();
        }

        this->CreateCollision(_elem, _link, *collision, collisionPrefix);
        ++groupMeshCount;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::CreateVisuals(TiXmlElement* _elem,
  ConstUrdfLinkPtr _link)
{
  // loop through all visual groups. as well as additional visuals from
  //   lumped meshes (fixed joint reduction)
  for (std::map<std::string,
    boost::shared_ptr<std::vector<UrdfVisualPtr> > >::const_iterator
    visualsIt = _link->visual_groups.begin();
    visualsIt != _link->visual_groups.end(); ++visualsIt)
  {
    unsigned int defaultMeshCount = 0;
    unsigned int groupMeshCount = 0;
    unsigned int lumpMeshCount = 0;
    // loop through all visuals in this group
    for (std::vector<UrdfVisualPtr>::iterator
         visual = visualsIt->second->begin();
         visual != visualsIt->second->end();
         ++visual)
    {
      if (visualsIt->first == "default")
      {
        // gzdbg << "creating default visual for link [" << _link->name
        //       << "]";

        std::string visualPrefix = _link->name;

        if (defaultMeshCount > 0)
        {
          // append _[meshCount] to _link name for additional visuals
          std::ostringstream visualNameStream;
          visualNameStream << visualPrefix << "_" << defaultMeshCount;
          visualPrefix = visualNameStream.str();
        }

        // create a <visual> block
        this->CreateVisual(_elem, _link, *visual, visualPrefix);

        // only 1 default mesh
        ++defaultMeshCount;
      }
      else if (visualsIt->first.find(std::string("lump::")) == 0)
      {
        // if visual name starts with "lump::", pass through
        //   original parent link name
        // gzdbg << "creating lump visual [" << visualsIt->first
        //       << "] for link [" << _link->name << "].\n";
        /// visualPrefix is the original name before lumping
        std::string visualPrefix = visualsIt->first.substr(6);

        if (lumpMeshCount > 0)
        {
          // append _[meshCount] to _link name for additional visuals
          std::ostringstream visualNameStream;
          visualNameStream << visualPrefix << "_" << lumpMeshCount;
          visualPrefix = visualNameStream.str();
        }

        this->CreateVisual(_elem, _link, *visual, visualPrefix);
        ++lumpMeshCount;
      }
      else
      {
        // gzdbg << "adding visuals from visual group ["
        //      << visualsIt->first << "]\n";

        std::string visualPrefix = _link->name + std::string("_") +
                                      visualsIt->first;

        if (groupMeshCount > 0)
        {
          // append _[meshCount] to _link name for additional visuals
          std::ostringstream visualNameStream;
          visualNameStream << visualPrefix << "_" << groupMeshCount;
          visualPrefix = visualNameStream.str();
        }

        this->CreateVisual(_elem, _link, *visual, visualPrefix);
        ++groupMeshCount;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::CreateInertial(TiXmlElement *_elem,
  ConstUrdfLinkPtr _link)
{
  TiXmlElement *inertial = new TiXmlElement("inertial");

  /* set mass properties */
  // check and print a warning message
  double roll, pitch, yaw;
  _link->inertial->origin.rotation.getRPY(roll, pitch, yaw);
  if (!gazebo::math::equal(roll, 0.0) ||
    !gazebo::math::equal(pitch, 0.0) || !gazebo::math::equal(yaw, 0.0))
      gzerr << "rotation of inertial frame in link ["
            << _link->name << "] is not supported\n";

  /// add pose
  gazebo::math::Pose pose = URDF2Gazebo::CopyPose(_link->inertial->origin);
  this->AddTransform(inertial, pose);

  // add mass
  this->AddKeyValue(inertial, "mass",
                  this->Values2str(1, &_link->inertial->mass));

  // add inertia (ixx, ixy, ixz, iyy, iyz, izz)
  TiXmlElement *inertia = new TiXmlElement("inertia");
  this->AddKeyValue(inertia, "ixx",
                  this->Values2str(1, &_link->inertial->ixx));
  this->AddKeyValue(inertia, "ixy",
                  this->Values2str(1, &_link->inertial->ixy));
  this->AddKeyValue(inertia, "ixz",
                  this->Values2str(1, &_link->inertial->ixz));
  this->AddKeyValue(inertia, "iyy",
                  this->Values2str(1, &_link->inertial->iyy));
  this->AddKeyValue(inertia, "iyz",
                  this->Values2str(1, &_link->inertial->iyz));
  this->AddKeyValue(inertia, "izz",
                  this->Values2str(1, &_link->inertial->izz));
  inertial->LinkEndChild(inertia);

  _elem->LinkEndChild(inertial);
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::CreateJoint(TiXmlElement *_root,
  ConstUrdfLinkPtr _link,
  gazebo::math::Pose &_currentTransform)
{
    /* compute the joint tag */
    std::string jtype;
    jtype.clear();
    if (_link->parent_joint != NULL)
    {
      switch (_link->parent_joint->type)
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
            gzwarn << "Unknown joint type: [" << _link->parent_joint->type
                   << "] in link [" << _link->name << "]\n";
            break;
      }
    }

    // skip if joint type is fixed and we are not faking it with a hinge,
    //   skip/return with the exception of root link being world,
    //   because there's no lumping there
    if (_link->getParent() && _link->getParent()->name != "world"
        && jtype == "fixed" && this->reduceFixedJoints) return;

    if (!jtype.empty())
    {
        TiXmlElement *joint = new TiXmlElement("joint");
        if (jtype == "fixed")
          joint->SetAttribute("type", "revolute");
        else
          joint->SetAttribute("type", jtype);
        joint->SetAttribute("name", _link->parent_joint->name);
        this->AddKeyValue(joint, "child", _link->name);
        this->AddKeyValue(joint, "parent", _link->getParent()->name);

        TiXmlElement *jointAxis = new TiXmlElement("axis");
        TiXmlElement *jointAxisLimit = new TiXmlElement("limit");
        TiXmlElement *jointAxisDynamics = new TiXmlElement("dynamics");
        if (jtype == "fixed")
        {
          this->AddKeyValue(jointAxisLimit, "lower", "0");
          this->AddKeyValue(jointAxisLimit, "upper", "0");
          this->AddKeyValue(jointAxisDynamics, "damping", "0");
        }
        else
        {
          gazebo::math::Vector3 rotatedJointAxis =
            _currentTransform.rot.RotateVector(
            gazebo::math::Vector3(_link->parent_joint->axis.x,
                                  _link->parent_joint->axis.y,
                                  _link->parent_joint->axis.z));
          double rotatedJointAxisArray[3] =
            { rotatedJointAxis.x, rotatedJointAxis.y, rotatedJointAxis.z };
          this->AddKeyValue(jointAxis, "xyz",
                  this->Values2str(3, rotatedJointAxisArray));
          if (_link->parent_joint->dynamics)
            this->AddKeyValue(jointAxisDynamics, "damping",
              this->Values2str(1, &_link->parent_joint->dynamics->damping));

          if (this->enforceLimits && _link->parent_joint->limits)
          {
            if (jtype == "slider")
            {
              this->AddKeyValue(jointAxisLimit, "lower",
                this->Values2str(1, &_link->parent_joint->limits->lower));
              this->AddKeyValue(jointAxisLimit, "upper",
                this->Values2str(1, &_link->parent_joint->limits->upper));
              this->AddKeyValue(jointAxisLimit, "effort",
                this->Values2str(1, &_link->parent_joint->limits->effort));
              this->AddKeyValue(jointAxisLimit, "velocity",
                this->Values2str(1, &_link->parent_joint->limits->velocity));
            }
            else if (_link->parent_joint->type != urdf::Joint::CONTINUOUS)
            {
              double *lowstop  = &_link->parent_joint->limits->lower;
              double *highstop = &_link->parent_joint->limits->upper;
              // enforce ode bounds, this will need to be fixed
              if (*lowstop > *highstop)
              {
                gzwarn << "urdf2gazebo: revolute joint ["
                       << _link->parent_joint->name
                       << "] with limits: lowStop[" << *lowstop
                       << "] > highStop[" << highstop
                       << "], switching the two.\n";
                double tmp = *lowstop;
                *lowstop = *highstop;
                *highstop = tmp;
              }
              this->AddKeyValue(jointAxisLimit, "lower",
                this->Values2str(1, &_link->parent_joint->limits->lower));
              this->AddKeyValue(jointAxisLimit, "upper",
                this->Values2str(1, &_link->parent_joint->limits->upper));
              this->AddKeyValue(jointAxisLimit, "effort",
                this->Values2str(1, &_link->parent_joint->limits->effort));
              this->AddKeyValue(jointAxisLimit, "velocity",
                this->Values2str(1, &_link->parent_joint->limits->velocity));
            }
          }
        }
        jointAxis->LinkEndChild(jointAxisLimit);
        jointAxis->LinkEndChild(jointAxisDynamics);
        joint->LinkEndChild(jointAxis);

        /* copy gazebo extensions data */
        this->InsertGazeboExtensionJoint(joint, _link->parent_joint->name);

        /* add joint to document */
        _root->LinkEndChild(joint);
    }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::CreateCollision(TiXmlElement* _elem, ConstUrdfLinkPtr _link,
  UrdfCollisionPtr _collision, const std::string &_oldLinkName)
{
    /* begin create geometry node, skip if no collision specified */
    TiXmlElement *gazeboCollision = new TiXmlElement("collision");

    /* set its name, if lumped, add original link name */
    if (_oldLinkName == _link->name)
      gazeboCollision->SetAttribute("name",
        _link->name + std::string("_collision"));
    else
      gazeboCollision->SetAttribute("name",
        _link->name + std::string("_collision_") + _oldLinkName);

    /* set transform */
    double pose[6];
    pose[0] = _collision->origin.position.x;
    pose[1] = _collision->origin.position.y;
    pose[2] = _collision->origin.position.z;
    _collision->origin.rotation.getRPY(pose[3], pose[4], pose[5]);
    this->AddKeyValue(gazeboCollision, "pose", this->Values2str(6, pose));


    /* add geometry block */
    if (!_collision || !_collision->geometry)
    {
      // gzdbg << "urdf2gazebo: collision of link [" << _link->name
      //       << "] has no <geometry>.\n";
    }
    else
    {
      CreateGeometry(gazeboCollision, _collision->geometry);
    }

    /* set additional data from extensions */
    this->InsertGazeboExtensionCollision(gazeboCollision, _oldLinkName);

    /* add geometry to body */
    _elem->LinkEndChild(gazeboCollision);
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::CreateVisual(TiXmlElement *_elem, ConstUrdfLinkPtr _link,
  UrdfVisualPtr _visual, const std::string &_oldLinkName)
{
    /* begin create gazebo visual node */
    TiXmlElement *gazeboVisual = new TiXmlElement("visual");

    /* set its name */
    // gzdbg << "original link name [" << _oldLinkName
    //       << "] new link name [" << _link->name << "]\n";
    if (_oldLinkName == _link->name)
      gazeboVisual->SetAttribute("name", _link->name + std::string("_vis"));
    else
      gazeboVisual->SetAttribute("name", _link->name + std::string("_vis_")
        + _oldLinkName);

    /* add the visualisation transfrom */
    double pose[6];
    pose[0] = _visual->origin.position.x;
    pose[1] = _visual->origin.position.y;
    pose[2] = _visual->origin.position.z;
    _visual->origin.rotation.getRPY(pose[3], pose[4], pose[5]);
    this->AddKeyValue(gazeboVisual, "pose", this->Values2str(6, pose));

    /* insert geometry */
    if (!_visual || !_visual->geometry)
    {
      // gzdbg << "urdf2gazebo: visual of link [" << _link->name
      //       << "] has no <geometry>\n.";
    }
    else
      CreateGeometry(gazeboVisual, _visual->geometry);

    /* set additional data from extensions */
    this->InsertGazeboExtensionVisual(gazeboVisual, _oldLinkName);

    /* end create _visual node */
    _elem->LinkEndChild(gazeboVisual);
}

////////////////////////////////////////////////////////////////////////////////
TiXmlDocument URDF2Gazebo::InitModelString(const std::string &_urdfStr,
  bool _enforceLimits)
{
    this->enforceLimits = _enforceLimits;

    /* Create a RobotModel from string */
    boost::shared_ptr<urdf::ModelInterface> robotModel =
      urdf::parseURDF(_urdfStr.c_str());

    // an xml object to hold the xml result
    TiXmlDocument gazeboXmlOut;

    if (!robotModel)
    {
        gzerr << "Unable to call parseURDF on robot model\n";
        return gazeboXmlOut;
    }

    /* create root element and define needed namespaces */
    TiXmlElement *robot = new TiXmlElement("model");

    // set model name to urdf robot name if not specified
    robot->SetAttribute("name", robotModel->getName());

    /* initialize transform for the model, urdf is recursive,
       while sdf defines all links relative to model frame */
    gazebo::math::Pose transform;

    /* parse gazebo extension */
    TiXmlDocument urdfXml;
    urdfXml.Parse(_urdfStr.c_str());
    this->ParseGazeboExtension(urdfXml);

    ConstUrdfLinkPtr rootLink = robotModel->getRoot();

    /* Fixed Joint Reduction */
    /* if link connects to parent via fixed joint, lump down and remove link */
    /* set reduceFixedJoints to false will replace fixed joints with
       zero limit revolute joints, otherwise, we reduce it down to its
       parent link recursively */
    if (this->reduceFixedJoints)
      this->ReduceFixedJoints(robot,
        (boost::const_pointer_cast< urdf::Link >(rootLink)));

    if (rootLink->name == "world")
    {
      /* convert all children link */
      for (std::vector<UrdfLinkPtr>::const_iterator
        child = rootLink->child_links.begin();
        child != rootLink->child_links.end(); ++child)
          CreateSDF(robot, (*child), transform);
    }
    else
    {
      /* convert, starting from root link */
      CreateSDF(robot, rootLink, transform);
    }

    /* insert the extensions without reference into <robot> root level */
    this->InsertGazeboExtensionRobot(robot);

    // add robot to gazeboXmlOut
    TiXmlElement *gazeboSdf = new TiXmlElement("sdf");
    gazeboSdf->SetAttribute("version", SDF_VERSION);
    gazeboSdf->LinkEndChild(robot);
    gazeboXmlOut.LinkEndChild(gazeboSdf);

    // debug
    // gazeboXmlOut.Print();

    return gazeboXmlOut;
}

////////////////////////////////////////////////////////////////////////////////
TiXmlDocument URDF2Gazebo::InitModelDoc(TiXmlDocument* _xmlDoc)
{
    std::ostringstream stream;
    stream << *_xmlDoc;
    std::string urdfStr = stream.str();
    return InitModelString(urdfStr);
}

////////////////////////////////////////////////////////////////////////////////
TiXmlDocument URDF2Gazebo::InitModelFile(const std::string &_filename)
{
  TiXmlDocument xmlDoc;
  if (xmlDoc.LoadFile(_filename))
  {
    return this->InitModelDoc(&xmlDoc);
  }
  else
    gzerr << "Unable to load file[" << _filename << "].\n";

    return xmlDoc;
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceInertialToParent(UrdfLinkPtr _link)
{
    // gzdbg << "TREE:   mass lumping from [" << link->name
    //      << "] to [" << _link->getParent()->name << "]\n.";
    /* now lump all contents of this _link to parent */
    if (_link->inertial)
    {
      // get parent mass (in parent link frame)
      dMass parentMass;
      if (!_link->getParent()->inertial)
        _link->getParent()->inertial.reset(new urdf::Inertial);
      dMassSetParameters(&parentMass, _link->getParent()->inertial->mass,
        _link->getParent()->inertial->origin.position.x,
        _link->getParent()->inertial->origin.position.y,
        _link->getParent()->inertial->origin.position.z,
        _link->getParent()->inertial->ixx, _link->getParent()->inertial->iyy,
        _link->getParent()->inertial->izz, _link->getParent()->inertial->ixy,
         _link->getParent()->inertial->ixz, _link->getParent()->inertial->iyz);
      // PrintMass(_link->getParent()->name, parentMass);
      // PrintMass(_link->getParent());
      // set _link mass (in _link frame)
      dMass linkMass;
      dMassSetParameters(&linkMass, _link->inertial->mass,
        _link->inertial->origin.position.x,
        _link->inertial->origin.position.y,
        _link->inertial->origin.position.z,
        _link->inertial->ixx, _link->inertial->iyy, _link->inertial->izz,
        _link->inertial->ixy, _link->inertial->ixz, _link->inertial->iyz);
      // PrintMass(_link->name, linkMass);
      // PrintMass(_link);
      // un-rotate _link mass into parent link frame
      dMatrix3 R;
      double phi, theta, psi;
      _link->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(
        phi, theta, psi);
      dRFromEulerAngles(R, phi, theta, psi);
      dMassRotate(&linkMass, R);
      // PrintMass(_link->name, linkMass);
      // un-translate _link mass into parent link frame
      dMassTranslate(&linkMass,
        _link->parent_joint->parent_to_joint_origin_transform.position.x,
        _link->parent_joint->parent_to_joint_origin_transform.position.y,
        _link->parent_joint->parent_to_joint_origin_transform.position.z);
      // PrintMass(_link->name, linkMass);
      // now linkMass is in the parent frame, add linkMass to parentMass
      dMassAdd(&parentMass, &linkMass);
      // PrintMass(_link->getParent()->name, parentMass);
      // update parent mass
      _link->getParent()->inertial->mass = parentMass.mass;
      _link->getParent()->inertial->ixx  = parentMass.I[0+4*0];
      _link->getParent()->inertial->iyy  = parentMass.I[1+4*1];
      _link->getParent()->inertial->izz  = parentMass.I[2+4*2];
      _link->getParent()->inertial->ixy  = parentMass.I[0+4*1];
      _link->getParent()->inertial->ixz  = parentMass.I[0+4*2];
      _link->getParent()->inertial->iyz  = parentMass.I[1+4*2];
      _link->getParent()->inertial->origin.position.x  = parentMass.c[0];
      _link->getParent()->inertial->origin.position.y  = parentMass.c[1];
      _link->getParent()->inertial->origin.position.z  = parentMass.c[2];
      // PrintMass(_link->getParent());
    }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceVisualsToParent(UrdfLinkPtr _link)
{
  // lump visual to parent
  // lump all visual to parent, assign group name
  // "lump::"+group name+"::'+_link name
  // lump but keep the _link name in(/as) the group name,
  // so we can correlate visuals to visuals somehow.
  for (std::map<std::string,
    boost::shared_ptr<std::vector<UrdfVisualPtr> > >::iterator
    visualsIt = _link->visual_groups.begin();
    visualsIt != _link->visual_groups.end(); ++visualsIt)
  {
    if (visualsIt->first.find(std::string("lump::")) == 0)
    {
      // it's a previously lumped mesh, re-lump under same _groupName
      std::string lumpGroupName = visualsIt->first;
      // gzdbg << "re-lumping group name [" << lumpGroupName
      //       << "] to link [" << _link->getParent()->name << "]\n";
      for (std::vector<UrdfVisualPtr>::iterator
           visualIt = visualsIt->second->begin();
           visualIt != visualsIt->second->end(); ++visualIt)
      {
        // transform visual origin from _link frame to parent link
        // frame before adding to parent
        (*visualIt)->origin = this->TransformToParentFrame((*visualIt)->origin,
          _link->parent_joint->parent_to_joint_origin_transform);
        // add the modified visual to parent
        this->ReduceVisualToParent(_link->getParent(), lumpGroupName,
          *visualIt);
      }
    }
    else
    {
      // default and any other groups meshes
      std::string lumpGroupName = std::string("lump::")+_link->name;
      // gzdbg << "adding modified lump group name [" << lumpGroupName
      //       << "] to link [" << _link->getParent()->name << "]\n.";
      for (std::vector<UrdfVisualPtr>::iterator
        visualIt = visualsIt->second->begin();
        visualIt != visualsIt->second->end(); ++visualIt)
      {
        // transform visual origin from _link frame to
        // parent link frame before adding to parent
        (*visualIt)->origin = this->TransformToParentFrame((*visualIt)->origin,
          _link->parent_joint->parent_to_joint_origin_transform);
        // add the modified visual to parent
        this->ReduceVisualToParent(_link->getParent(), lumpGroupName,
          *visualIt);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceCollisionsToParent(UrdfLinkPtr _link)
{
    // lump collision parent
    // lump all collision to parent, assign group name
    // "lump::"+group name+"::'+_link name
    // lump but keep the _link name in(/as) the group name,
    // so we can correlate visuals to collisions somehow.
    for (std::map<std::string,
      boost::shared_ptr<std::vector<UrdfCollisionPtr> > >::iterator
      collisionsIt = _link->collision_groups.begin();
      collisionsIt != _link->collision_groups.end(); ++collisionsIt)
    {
      if (collisionsIt->first.find(std::string("lump::")) == 0)
      {
        // if it's a previously lumped mesh, relump under same _groupName
        std::string lumpGroupName = collisionsIt->first;
        // gzdbg << "re-lumping collision [" << collisionsIt->first
        //       << "] for link [" << _link->name
        //       << "] to parent [" << _link->getParent()->name
        //       << "] with group name [" << lumpGroupName << "]\n";
        for (std::vector<UrdfCollisionPtr>::iterator
          collisionIt = collisionsIt->second->begin();
          collisionIt != collisionsIt->second->end(); ++collisionIt)
        {
          // transform collision origin from _link frame to
          // parent link frame before adding to parent
          (*collisionIt)->origin = this->TransformToParentFrame(
            (*collisionIt)->origin,
            _link->parent_joint->parent_to_joint_origin_transform);
          // add the modified collision to parent
          this->ReduceCollisionToParent(_link->getParent(), lumpGroupName,
            *collisionIt);
        }
      }
      else
      {
        // default and any other group meshes
        std::string lumpGroupName = std::string("lump::")+_link->name;
        // gzdbg << "lumping collision [" << collisionsIt->first
        //       << "] for link [" << _link->name
        //       << "] to parent [" << _link->getParent()->name
        //       << "] with group name [" << lumpGroupName << "]\n";
        for (std::vector<UrdfCollisionPtr>::iterator
          collisionIt = collisionsIt->second->begin();
          collisionIt != collisionsIt->second->end(); ++collisionIt)
        {
          // transform collision origin from _link frame to
          // parent link frame before adding to parent
          (*collisionIt)->origin = this->TransformToParentFrame(
            (*collisionIt)->origin,
            _link->parent_joint->parent_to_joint_origin_transform);

          // add the modified collision to parent
          this->ReduceCollisionToParent(_link->getParent(), lumpGroupName,
            *collisionIt);
        }
      }
    }
    // this->PrintCollisionGroups(_link->getParent());
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceJointsToParent(UrdfLinkPtr _link)
{
    // set child link's parentJoint's parent link to
    // a parent link up stream that does not have a fixed parentJoint
    for (unsigned int i = 0 ; i < _link->child_links.size() ; ++i)
    {
      boost::shared_ptr<urdf::Joint> parentJoint =
        _link->child_links[i]->parent_joint;
      if (parentJoint->type != urdf::Joint::FIXED)
      {
        // go down the tree until we hit a parent joint that is not fixed
        UrdfLinkPtr newParentLink = _link;
        gazebo::math::Pose jointAnchorTransform;
        while (newParentLink->parent_joint &&
              newParentLink->getParent()->name != "world" &&
              newParentLink->parent_joint->type == urdf::Joint::FIXED)
        {
          jointAnchorTransform = jointAnchorTransform * jointAnchorTransform;
          parentJoint->parent_to_joint_origin_transform =
            this->TransformToParentFrame(
            parentJoint->parent_to_joint_origin_transform,
            newParentLink->parent_joint->parent_to_joint_origin_transform);
          newParentLink = newParentLink->getParent();
        }
        // now set the _link->child_links[i]->parent_joint's parent link to
        // the newParentLink
        _link->child_links[i]->setParent(newParentLink);
        parentJoint->parent_link_name = newParentLink->name;
        // and set the _link->child_links[i]->parent_joint's
        // parent_to_joint_origin_transform as the aggregated anchor transform?
      }
    }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceGazeboExtensionSensorTransformReduction(
  std::vector<TiXmlElement*>::iterator _blobIt,
  gazebo::math::Pose _reductionTransform)
{
    // overwrite <xyz> and <rpy> if they exist
    if ((*_blobIt)->ValueStr() == "sensor")
    {
      // parse it and add/replace the reduction transform
      // find first instance of xyz and rpy, replace with reduction transform

      // debug print
      // for (TiXmlNode* elIt = (*_blobIt)->FirstChild();
      //      elIt; elIt = elIt->NextSibling())
      // {
      //   std::ostringstream streamIn;
      //   streamIn << *elIt;
      //   gzdbg << "    " << streamIn << "\n";
      // }

      {
        TiXmlNode* oldPoseKey = (*_blobIt)->FirstChild("pose");
        /// @todo: FIXME:  we should read xyz, rpy and aggregate it to
        /// reductionTransform instead of just throwing the info away.
        if (oldPoseKey)
          (*_blobIt)->RemoveChild(oldPoseKey);
      }

      // convert reductionTransform to values
      urdf::Vector3 reductionXyz(_reductionTransform.pos.x,
                                  _reductionTransform.pos.y,
                                  _reductionTransform.pos.z);
      urdf::Rotation reductionQ(_reductionTransform.rot.x,
                                 _reductionTransform.rot.y,
                                 _reductionTransform.rot.z,
                                 _reductionTransform.rot.w);

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

      (*_blobIt)->LinkEndChild(poseKey);
    }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceGazeboExtensionProjectorTransformReduction(
  std::vector<TiXmlElement*>::iterator _blobIt,
  gazebo::math::Pose _reductionTransform)
{
    // overwrite <pose> (xyz/rpy) if it exists
    if ((*_blobIt)->ValueStr() == "projector")
    {
      /*
      // parse it and add/replace the reduction transform
      // find first instance of xyz and rpy, replace with reduction transform
      for (TiXmlNode* elIt = (*_blobIt)->FirstChild();
        elIt; elIt = elIt->NextSibling())
      {
        std::ostringstream streamIn;
        streamIn << *elIt;
        gzdbg << "    " << streamIn << "\n";
      }
      */

      /* should read <pose>...</pose> and agregate reductionTransform */
      TiXmlNode* poseKey = (*_blobIt)->FirstChild("pose");
      // read pose and save it

      // remove the tag for now
      if (poseKey) (*_blobIt)->RemoveChild(poseKey);

      // convert reductionTransform to values
      urdf::Vector3 reductionXyz(_reductionTransform.pos.x,
                                  _reductionTransform.pos.y,
                                  _reductionTransform.pos.z);
      urdf::Rotation reductionQ(_reductionTransform.rot.x,
                                 _reductionTransform.rot.y,
                                 _reductionTransform.rot.z,
                                 _reductionTransform.rot.w);

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

      (*_blobIt)->LinkEndChild(poseKey);
    }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceGazeboExtensionContactSensorFrameReplace(
  std::vector<TiXmlElement*>::iterator _blobIt, UrdfLinkPtr _link)
{
  std::string linkName = _link->name;
  std::string newLinkName = _link->getParent()->name;
  if ((*_blobIt)->ValueStr() == "sensor")
  {
    // parse it and add/replace the reduction transform
    // find first instance of xyz and rpy, replace with reduction transform
    TiXmlNode* contact = (*_blobIt)->FirstChild("contact");
    if (contact)
    {
      TiXmlNode* collision = contact->FirstChild("collision");
      if (collision)
      {
        if (this->GetKeyValueAsString(collision->ToElement()) ==
          linkName + std::string("_collision"))
        {
          contact->RemoveChild(collision);
          TiXmlElement* collisionNameKey = new TiXmlElement("collision");
          std::ostringstream collisionNameStream;
          collisionNameStream << newLinkName << "_collision_" << linkName;
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

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceGazeboExtensionPluginFrameReplace(
  std::vector<TiXmlElement*>::iterator _blobIt, UrdfLinkPtr _link,
  const std::string &_pluginName, const std::string &_elementName,
  gazebo::math::Pose _reductionTransform)
{
  std::string linkName = _link->name;
  std::string newLinkName = _link->getParent()->name;
  if ((*_blobIt)->ValueStr() == _pluginName)
  {
    // replace element containing _link names to parent link names
    // find first instance of xyz and rpy, replace with reduction transform
    TiXmlNode* elementNode = (*_blobIt)->FirstChild(_elementName);
    if (elementNode)
    {
      if (this->GetKeyValueAsString(elementNode->ToElement()) == linkName)
      {
        (*_blobIt)->RemoveChild(elementNode);
        TiXmlElement* bodyNameKey = new TiXmlElement(_elementName);
        std::ostringstream bodyNameStream;
        bodyNameStream << newLinkName;
        TiXmlText* bodyNameTxt = new TiXmlText(bodyNameStream.str());
        bodyNameKey->LinkEndChild(bodyNameTxt);
        (*_blobIt)->LinkEndChild(bodyNameKey);
        /// @todo update transforms for this gazebo plugin too

        // look for offset transforms, add reduction transform
        TiXmlNode* xyzKey = (*_blobIt)->FirstChild("xyzOffset");
        if (xyzKey)
        {
          urdf::Vector3 v1 = this->ParseVector3(xyzKey);
          _reductionTransform.pos = gazebo::math::Vector3(v1.x, v1.y, v1.z);
          // remove xyzOffset and rpyOffset
          (*_blobIt)->RemoveChild(xyzKey);
        }
        TiXmlNode* rpyKey = (*_blobIt)->FirstChild("rpyOffset");
        if (rpyKey)
        {
          urdf::Vector3 rpy = this->ParseVector3(rpyKey, M_PI/180.0);
          _reductionTransform.rot =
            gazebo::math::Quaternion::EulerToQuaternion(rpy.x, rpy.y, rpy.z);
          // remove xyzOffset and rpyOffset
          (*_blobIt)->RemoveChild(rpyKey);
        }

        // pass through the parent transform from fixed joint reduction
        _reductionTransform = inverseTransformToParentFrame(_reductionTransform,
          _link->parent_joint->parent_to_joint_origin_transform);

        // create new offset xml blocks
        xyzKey = new TiXmlElement("xyzOffset");
        rpyKey = new TiXmlElement("rpyOffset");

        // create new offset xml blocks
        urdf::Vector3 reductionXyz(_reductionTransform.pos.x,
          _reductionTransform.pos.y,
          _reductionTransform.pos.z);
        urdf::Rotation reductionQ(_reductionTransform.rot.x,
          _reductionTransform.rot.y, _reductionTransform.rot.z,
          _reductionTransform.rot.w);

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

        (*_blobIt)->LinkEndChild(xyzKey);
        (*_blobIt)->LinkEndChild(rpyKey);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceGazeboExtensionProjectorFrameReplace(
  std::vector<TiXmlElement*>::iterator _blobIt, UrdfLinkPtr _link)
{
  std::string linkName = _link->name;
  std::string newLinkName = _link->getParent()->name;

  // updates _link reference for <projector> inside of
  // projector plugins
  // update from <projector>MyLinkName/MyProjectorName</projector>
  // to <projector>NewLinkName/MyProjectorName</projector>
  TiXmlNode* projectorElem = (*_blobIt)->FirstChild("projector");
  {
    if (projectorElem)
    {
      std::string projectorName =  this->GetKeyValueAsString(
        projectorElem->ToElement());
      // extract projector _link name and projector name
      size_t pos = projectorName.find("/");
      if (pos == std::string::npos)
        gzerr << "no slash in projector reference tag [" << projectorName
              << "], expecting linkName/projector_name.\n";
      std::string projectorLinkName = projectorName.substr(0, pos);

      if (projectorLinkName == linkName)
      {
        // do the replacement
        projectorName = newLinkName + "/" +
          projectorName.substr(pos+1, projectorName.size());

        (*_blobIt)->RemoveChild(projectorElem);
        TiXmlElement* bodyNameKey = new TiXmlElement("projector");
        std::ostringstream bodyNameStream;
        bodyNameStream << projectorName;
        TiXmlText* bodyNameTxt = new TiXmlText(bodyNameStream.str());
        bodyNameKey->LinkEndChild(bodyNameTxt);
        (*_blobIt)->LinkEndChild(bodyNameKey);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceGazeboExtensionGripperFrameReplace(
  std::vector<TiXmlElement*>::iterator _blobIt, UrdfLinkPtr _link)
{
  std::string linkName = _link->name;
  std::string newLinkName = _link->getParent()->name;

  if ((*_blobIt)->ValueStr() == "gripper")
  {
    TiXmlNode* gripperLink = (*_blobIt)->FirstChild("gripper_link");
    if (gripperLink)
    {
      if (this->GetKeyValueAsString(gripperLink->ToElement()) == linkName)
      {
        (*_blobIt)->RemoveChild(gripperLink);
        TiXmlElement* bodyNameKey = new TiXmlElement("gripper_link");
        std::ostringstream bodyNameStream;
        bodyNameStream << newLinkName;
        TiXmlText* bodyNameTxt = new TiXmlText(bodyNameStream.str());
        bodyNameKey->LinkEndChild(bodyNameTxt);
        (*_blobIt)->LinkEndChild(bodyNameKey);
      }
    }
    TiXmlNode* palmLink = (*_blobIt)->FirstChild("palm_link");
    if (palmLink)
    {
      if (this->GetKeyValueAsString(palmLink->ToElement()) == linkName)
      {
        (*_blobIt)->RemoveChild(palmLink);
        TiXmlElement* bodyNameKey = new TiXmlElement("palm_link");
        std::ostringstream bodyNameStream;
        bodyNameStream << newLinkName;
        TiXmlText* bodyNameTxt = new TiXmlText(bodyNameStream.str());
        bodyNameKey->LinkEndChild(bodyNameTxt);
        (*_blobIt)->LinkEndChild(bodyNameKey);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2Gazebo::ReduceGazeboExtensionJointFrameReplace(
  std::vector<TiXmlElement*>::iterator _blobIt, UrdfLinkPtr _link)
{
  std::string linkName = _link->name;
  std::string newLinkName = _link->getParent()->name;

  if ((*_blobIt)->ValueStr() == "joint")
  {
    // parse it and add/replace the reduction transform
    // find first instance of xyz and rpy, replace with reduction transform
    TiXmlNode* parent = (*_blobIt)->FirstChild("parent");
    if (parent)
    {
      if (this->GetKeyValueAsString(parent->ToElement()) == linkName)
      {
        (*_blobIt)->RemoveChild(parent);
        TiXmlElement* parentNameKey = new TiXmlElement("parent");
        std::ostringstream parentNameStream;
        parentNameStream << newLinkName;
        TiXmlText* parentNameTxt = new TiXmlText(parentNameStream.str());
        parentNameKey->LinkEndChild(parentNameTxt);
        (*_blobIt)->LinkEndChild(parentNameKey);
      }
    }
    TiXmlNode* child = (*_blobIt)->FirstChild("child");
    if (child)
    {
      if (this->GetKeyValueAsString(child->ToElement()) == linkName)
      {
        (*_blobIt)->RemoveChild(child);
        TiXmlElement* childNameKey = new TiXmlElement("child");
        std::ostringstream childNameStream;
        childNameStream << newLinkName;
        TiXmlText* childNameTxt = new TiXmlText(childNameStream.str());
        childNameKey->LinkEndChild(childNameTxt);
        (*_blobIt)->LinkEndChild(childNameKey);
      }
    }
    /// @todo add anchor offsets if parent link changes location!
  }
}
}
