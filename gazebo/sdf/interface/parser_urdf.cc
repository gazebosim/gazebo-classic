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
#include <sdf/interface/parser_urdf.hh>
#include <sdf/sdf.hh>

#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>

#include "gazebo/common/SystemPaths.hh"

namespace urdf2gazebo
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

void URDF2Gazebo::ReduceVisualToParent(LinkPtr _link,
       std::string _groupName, VisualPtr _visual)
{
  boost::shared_ptr<std::vector<VisualPtr> > viss
    = _link->getVisuals(_groupName);
  if (!viss)
  {
    // group does not exist, create one and add to map
    viss.reset(new std::vector<VisualPtr>);
    // new group name, create vector, add vector to map and
    //   add Visual to the vector
    _link->visual_groups.insert(make_pair(_groupName, viss));
    // gzdbg << "successfully added a new visual group name ["
    //       << _groupName << "]\n";
  }

  // group exists, add Visual to the vector in the map if it's not there
  std::vector<VisualPtr>::iterator vis_it
    = find(viss->begin(), viss->end(), _visual);
  if (vis_it != viss->end())
    gzwarn << "attempted to add visual to link ["
           << _link->name
           << "], but it already exists under group ["
           << _groupName << "]\n";
  else
    viss->push_back(_visual);
}

void URDF2Gazebo::ReduceCollisionToParent(LinkPtr _link,
      std::string _groupName, CollisionPtr _collision)
{
  boost::shared_ptr<std::vector<CollisionPtr> >
    cols = _link->getCollisions(_groupName);
  if (!cols)
  {
    // group does not exist, create one and add to map
    cols.reset(new std::vector<CollisionPtr>);
    // new group name, create add vector to map and add Collision to the vector
    _link->collision_groups.insert(make_pair(_groupName, cols));
  }

  // group exists, add Collision to the vector in the map
  std::vector<CollisionPtr>::iterator col_it =
    find(cols->begin(), cols->end(), _collision);
  if (col_it != cols->end())
    gzwarn << "attempted to add collision to link ["
           << _link->name
           << "], but it already exists under group ["
           << _groupName << "]\n";
  else
    cols->push_back(_collision);
}

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

void URDF2Gazebo::AddKeyValue(TiXmlElement *_elem, const std::string& _key,
  const std::string &_value)
{
  TiXmlElement* child_elem = _elem->FirstChildElement(_key);
  if (child_elem)
  {
    std::string old_value = GetKeyValueAsString(child_elem);
    if (old_value != _value)
      gzwarn << "multiple inconsistent <" << _key
             << "> exists due to fixed joint reduction"
             << " overwriting previous value [" << old_value
             << "] with [" << _value << "].\n";
    // else
    //   gzdbg << "multiple consistent <" << _key
    //          << "> exists with [" << _value
    //          << "] due to fixed joint reduction.\n";
    _elem->RemoveChild(child_elem);  // remove old _elem
  }

  TiXmlElement *ekey      = new TiXmlElement(_key);
  TiXmlText    *text_ekey = new TiXmlText(_value);
  ekey->LinkEndChild(text_ekey);
  _elem->LinkEndChild(ekey);
}

void URDF2Gazebo::AddTransform(TiXmlElement *_elem,
  const::gazebo::math::Pose& _transform)
{
  gazebo::math::Vector3 e = _transform.rot.GetAsEuler();
  double cpose[6] = { _transform.pos.x, _transform.pos.y,
                      _transform.pos.z, e.x, e.y, e.z };

  /* set geometry transform */
  AddKeyValue(_elem, "pose", Values2str(6, cpose));
}

std::string URDF2Gazebo::GetKeyValueAsString(TiXmlElement* _elem)
{
  std::string value_str;
  if (_elem->Attribute("value"))
  {
    value_str = _elem->Attribute("value");
  }
  else if (_elem->FirstChild())
  /// @todo: FIXME: comment out check for now, different tinyxml
  /// versions fails to compile:
  //  && _elem->FirstChild()->Type() == TiXmlNode::TINYXML_TEXT)
  {
    value_str = _elem->FirstChild()->ValueStr();
  }
  return value_str;
}

void URDF2Gazebo::ParseGazeboExtension(TiXmlDocument &_urdfXml)
{
  TiXmlElement* robot_xml = _urdfXml.FirstChildElement("robot");

  // Get all Gazebo extension elements, put everything in
  //   this->gazeboEntensions map, containing a key string
  //   (link/joint name) and values
  for (TiXmlElement* gazebo_xml = robot_xml->FirstChildElement("gazebo");
       gazebo_xml; gazebo_xml = gazebo_xml->NextSiblingElement("gazebo"))
  {
    const char* ref = gazebo_xml->Attribute("reference");
    std::string ref_str;
    if (!ref)
    {
      // copy extensions for robot (outside of link/joint)
      ref_str.clear();
    }
    else
    {
      // copy extensions for link/joint
      ref_str = std::string(ref);
    }

    if (this->gazeboEntensions.find(ref_str) ==
        this->gazeboEntensions.end())
    {
        // create extension map for reference
        std::vector<GazeboExtension*> extensions;
        this->gazeboEntensions.insert(std::make_pair(ref_str, extensions));
    }

    // create and insert a new GazeboExtension into the map
    GazeboExtension* gazebo = new GazeboExtension();

    // begin parsing xml node
    for (TiXmlElement *child_elem = gazebo_xml->FirstChildElement();
         child_elem; child_elem = child_elem->NextSiblingElement())
    {
      gazebo->oldLinkName = ref_str;

      // go through all elements of the extension,
      //   extract what we know, and save the rest in blobs
      // @todo:  somehow use sdf definitions here instead of hard coded
      //         objects

      // material
      if (child_elem->ValueStr() == "material")
      {
          gazebo->material = GetKeyValueAsString(child_elem);
      }
      else if (child_elem->ValueStr() == "static")
      {
        std::string value_str = GetKeyValueAsString(child_elem);

        // default of setting static flag is false
        if (lowerStr(value_str) == "true" || lowerStr(value_str) == "yes" ||
            value_str == "1")
          gazebo->setStaticFlag = true;
        else
          gazebo->setStaticFlag = false;
      }
      else if (child_elem->ValueStr() == "gravity")
      {
        std::string value_str = GetKeyValueAsString(child_elem);

        // default of gravity is true
        if (lowerStr(value_str) == "false" || lowerStr(value_str) == "no" ||
            value_str == "0")
          gazebo->gravity = false;
        else
          gazebo->gravity = true;
      }
      else if (child_elem->ValueStr() == "dampingFactor")
      {
          gazebo->isDampingFactor = true;
          gazebo->dampingFactor = boost::lexical_cast<double>(
              GetKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "maxVel")
      {
          gazebo->isMaxVel = true;
          gazebo->maxVel = boost::lexical_cast<double>(
              GetKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "minDepth")
      {
          gazebo->isMinDepth = true;
          gazebo->minDepth = boost::lexical_cast<double>(
            GetKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "mu1")
      {
          gazebo->isMu1 = true;
          gazebo->mu1 = boost::lexical_cast<double>(
            GetKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "mu2")
      {
          gazebo->isMu2 = true;
          gazebo->mu2 = boost::lexical_cast<double>(
            GetKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "fdir1")
      {
          gazebo->fdir1 = GetKeyValueAsString(child_elem);
      }
      else if (child_elem->ValueStr() == "kp")
      {
          gazebo->isKp = true;
          gazebo->kp = boost::lexical_cast<double>(
            GetKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "kd")
      {
          gazebo->isKd = true;
          gazebo->kd = boost::lexical_cast<double>(
            GetKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "selfCollide")
      {
        std::string value_str = GetKeyValueAsString(child_elem);

        // default of selfCollide is false
        if (lowerStr(value_str) == "true" || lowerStr(value_str) == "yes" ||
            value_str == "1")
          gazebo->selfCollide = true;
        else
          gazebo->selfCollide = false;
      }
      else if (child_elem->ValueStr() == "laserRetro")
      {
          gazebo->isLaserRetro = true;
          gazebo->laserRetro = boost::lexical_cast<double>(
            GetKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "stopCfm")
      {
          gazebo->isStopCfm = true;
          gazebo->stopCfm = boost::lexical_cast<double>(
            GetKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "stopErp")
      {
          gazebo->isStopErp = true;
          gazebo->stopErp = boost::lexical_cast<double>(
            GetKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "initialJointPosition")
      {
          gazebo->isInitialJointPosition = true;
          gazebo->initialJointPosition = boost::lexical_cast<double>(
            GetKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "fudgeFactor")
      {
          gazebo->isFudgeFactor = true;
          gazebo->fudgeFactor = boost::lexical_cast<double>(
            GetKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "provideFeedback")
      {
          std::string value_str = GetKeyValueAsString(child_elem);

          if (lowerStr(value_str) == "true" || lowerStr(value_str) == "yes" ||
              value_str == "1")
            gazebo->provideFeedback = true;
          else
            gazebo->provideFeedback = false;
      }
      else
      {
          std::ostringstream stream;
          stream << *child_elem;
          // save all unknown stuff in a vector of blobs
          TiXmlElement *blob = new TiXmlElement(*child_elem);
          gazebo->blobs.push_back(blob);
      }
    }

    // insert into my map
    (this->gazeboEntensions.find(ref_str))->second.push_back(gazebo);
  }
}

void URDF2Gazebo::InsertGazeboExtensionCollision(TiXmlElement *_elem,
  std::string _linkName)
{
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazebo_it = this->gazeboEntensions.begin();
       gazebo_it != this->gazeboEntensions.end(); ++gazebo_it)
  {
    for (std::vector<GazeboExtension*>::iterator ge = gazebo_it->second.begin();
         ge != gazebo_it->second.end(); ++ge)
    {
      if ((*ge)->oldLinkName == _linkName)
      {
        TiXmlElement *surface = new TiXmlElement("surface");
        TiXmlElement *friction = new TiXmlElement("friction");
        TiXmlElement *friction_ode = new TiXmlElement("ode");
        TiXmlElement *contact = new TiXmlElement("contact");
        TiXmlElement *contact_ode = new TiXmlElement("ode");

        // insert mu1, mu2, kp, kd for collision
        if ((*ge)->isMu1)
          AddKeyValue(friction_ode, "mu", Values2str(1, &(*ge)->mu1));
        if ((*ge)->isMu2)
          AddKeyValue(friction_ode, "mu2", Values2str(1, &(*ge)->mu2));
        if (!(*ge)->fdir1.empty())
          AddKeyValue(friction_ode, "fdir1", (*ge)->fdir1);
        if ((*ge)->isKp)
          AddKeyValue(contact_ode, "kp", Values2str(1, &(*ge)->kp));
        if ((*ge)->isKd)
          AddKeyValue(contact_ode, "kd", Values2str(1, &(*ge)->kd));
        // max contact interpenetration correction velocity
        if ((*ge)->isMaxVel)
          AddKeyValue(contact_ode, "max_vel", Values2str(1, &(*ge)->maxVel));
        // contact interpenetration margin tolerance
        if ((*ge)->isMinDepth)
          AddKeyValue(contact_ode, "min_depth",
                      Values2str(1, &(*ge)->minDepth));
        if ((*ge)->isLaserRetro)
          AddKeyValue(_elem, "laser_retro", Values2str(1, &(*ge)->laserRetro));

        contact->LinkEndChild(contact_ode);
        surface->LinkEndChild(contact);
        friction->LinkEndChild(friction_ode);
        surface->LinkEndChild(friction);
        _elem->LinkEndChild(surface);
      }
    }
  }
}

void URDF2Gazebo::InsertGazeboExtensionVisual(TiXmlElement *_elem,
  std::string _linkName)
{
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazebo_it = this->gazeboEntensions.begin();
       gazebo_it != this->gazeboEntensions.end(); ++gazebo_it)
  {
    for (std::vector<GazeboExtension*>::iterator ge = gazebo_it->second.begin();
         ge != gazebo_it->second.end(); ++ge)
    {
      if ((*ge)->oldLinkName == _linkName)
      {
        // insert material block
        if (!(*ge)->material.empty())
            AddKeyValue(_elem, "material", (*ge)->material);
      }
    }
  }
}

void URDF2Gazebo::InsertGazeboExtensionLink(TiXmlElement *_elem,
  std::string _linkName)
{
    for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
         gazebo_it = this->gazeboEntensions.begin();
         gazebo_it != this->gazeboEntensions.end(); ++gazebo_it)
    {
      if (gazebo_it->first == _linkName)
      {
        // gzdbg << "inserting extension with reference ["
        //       << _linkName << "] into link.\n";
        for (std::vector<GazeboExtension*>::iterator ge =
             gazebo_it->second.begin(); ge != gazebo_it->second.end(); ++ge)
        {
          // insert gravity
          if ((*ge)->gravity)
              AddKeyValue(_elem, "gravity", "true");
          else
              AddKeyValue(_elem, "gravity", "false");

          // damping factor
          TiXmlElement *velocity_decay = new TiXmlElement("velocity_decay");
          if ((*ge)->isDampingFactor)
          {
            /// @todo separate linear and angular velocity decay
            AddKeyValue(_elem, "linear", Values2str(1, &(*ge)->dampingFactor));
            AddKeyValue(_elem, "angular", Values2str(1, &(*ge)->dampingFactor));
          }
          _elem->LinkEndChild(velocity_decay);
          // selfCollide tag
          if ((*ge)->selfCollide)
              AddKeyValue(_elem, "self_collide", "true");
          else
              AddKeyValue(_elem, "self_collide", "false");
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

void URDF2Gazebo::InsertGazeboExtensionJoint(TiXmlElement *_elem,
  std::string _jointName)
{
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazebo_it = this->gazeboEntensions.begin();
       gazebo_it != this->gazeboEntensions.end(); ++gazebo_it)
  {
    if (gazebo_it->first == _jointName)
    {
      for (std::vector<GazeboExtension*>::iterator
           ge = gazebo_it->second.begin();
           ge != gazebo_it->second.end(); ++ge)
      {
        TiXmlElement *physics     = new TiXmlElement("physics");
        TiXmlElement *physicsOde = new TiXmlElement("ode");
        TiXmlElement *limit       = new TiXmlElement("limit");
        // insert stopCfm, stopErp, fudgeFactor
        if ((*ge)->isStopCfm)
        {
          AddKeyValue(limit, "erp", Values2str(1, &(*ge)->stopCfm));
        }
        if ((*ge)->isStopErp)
        {
          AddKeyValue(limit, "cfm", Values2str(1, &(*ge)->stopErp));
        }
        /* gone
        if ((*ge)->isInitialJointPosition)
            AddKeyValue(_elem, "initialJointPosition",
              Values2str(1, &(*ge)->initialJointPosition));
        */

        // insert provideFeedback
        if ((*ge)->provideFeedback)
            AddKeyValue(physicsOde, "provide_feedback", "true");
        else
            AddKeyValue(physicsOde, "provide_feedback", "false");

        // insert fudgeFactor
        if ((*ge)->isFudgeFactor)
          AddKeyValue(physicsOde, "fudge_factor",
                      Values2str(1, &(*ge)->fudgeFactor));

        physics->LinkEndChild(physicsOde);
        physicsOde->LinkEndChild(limit);
        _elem->LinkEndChild(physics);
      }
    }
  }
}

void URDF2Gazebo::InsertGazeboExtensionRobot(TiXmlElement *_elem)
{
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazebo_it = this->gazeboEntensions.begin();
       gazebo_it != this->gazeboEntensions.end(); ++gazebo_it)
  {
    if (gazebo_it->first.empty())
    {
      // no reference specified
      for (std::vector<GazeboExtension*>::iterator
        ge = gazebo_it->second.begin(); ge != gazebo_it->second.end(); ++ge)
      {
        // insert static flag
        if ((*ge)->setStaticFlag)
            AddKeyValue(_elem, "static", "true");
        else
            AddKeyValue(_elem, "static", "false");

        // copy extension containing blobs and without reference
        for (std::vector<TiXmlElement*>::iterator
             blobIt = (*ge)->blobs.begin();
             blobIt != (*ge)->blobs.end(); ++blobIt)
        {
            std::ostringstream stream_in;
            stream_in << *(*blobIt);
            _elem->LinkEndChild((*blobIt)->Clone());
        }
      }
    }
  }
}

void URDF2Gazebo::CreateGeometry(TiXmlElement* _elem,
  boost::shared_ptr<urdf::Geometry> _geometry)
{
  int sizeCount;
  double sizeVals[3];

  TiXmlElement *gazebo_geometry = new TiXmlElement("geometry");

  std::string type;
  TiXmlElement *geometry_type = NULL;

  switch (_geometry->type)
  {
  case urdf::Geometry::BOX:
    type = "box";
    sizeCount = 3;
    {
      boost::shared_ptr<const urdf::Box> box;
      box = boost::dynamic_pointer_cast< const urdf::Box >(_geometry);
      sizeVals[0] = box->dim.x;
      sizeVals[1] = box->dim.y;
      sizeVals[2] = box->dim.z;
      geometry_type = new TiXmlElement(type);
      AddKeyValue(geometry_type, "size", Values2str(sizeCount, sizeVals));
    }
    break;
  case urdf::Geometry::CYLINDER:
    type = "cylinder";
    sizeCount = 2;
    {
      boost::shared_ptr<const urdf::Cylinder> cylinder;
      cylinder = boost::dynamic_pointer_cast<const urdf::Cylinder >(_geometry);
      geometry_type = new TiXmlElement(type);
      AddKeyValue(geometry_type, "length", Values2str(1, &cylinder->length));
      AddKeyValue(geometry_type, "radius", Values2str(1, &cylinder->radius));
    }
    break;
  case urdf::Geometry::SPHERE:
    type = "sphere";
    sizeCount = 1;
    {
      boost::shared_ptr<const urdf::Sphere> sphere;
      sphere = boost::dynamic_pointer_cast<const urdf::Sphere >(_geometry);
      geometry_type = new TiXmlElement(type);
      AddKeyValue(geometry_type, "radius", Values2str(1, &sphere->radius));
    }
    break;
  case urdf::Geometry::MESH:
    type = "mesh";
    sizeCount = 3;
    {
      boost::shared_ptr<const urdf::Mesh> mesh;
      mesh = boost::dynamic_pointer_cast<const urdf::Mesh >(_geometry);
      sizeVals[0] = mesh->scale.x;
      sizeVals[1] = mesh->scale.y;
      sizeVals[2] = mesh->scale.z;
      geometry_type = new TiXmlElement(type);
      AddKeyValue(geometry_type, "scale", Vector32Str(mesh->scale));
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
        std::string model_filename = mesh->filename;
        std::string package_prefix("package://");
        std::string model_prefix("model://");
        size_t pos1 = model_filename.find(package_prefix, 0);
        if (pos1 != std::string::npos)
        {
          size_t rep_len = package_prefix.size();
          model_filename.replace(pos1, rep_len, model_prefix);
          // gzwarn << "ros style uri [package://] is"
          //   << "automatically converted: [" << model_filename
          //   << "], make sure your ros package is in GAZEBO_MODEL_PATH"
          //   << " and switch your manifest to conform to gazebo's"
          //   << " model database format.  See ["
          //   << "http://gazebosim.org/wiki/Model_database#Model_Manifest_XML"
          //   << "] for more info.\n";
        }

        // add mesh filename
        AddKeyValue(geometry_type, "uri", model_filename);
      }
    }
    break;
  default:
    sizeCount = 0;
    gzwarn << "Unknown body type: [" << _geometry->type
           << "] skipped in geometry\n";
    break;
  }

  if (geometry_type)
  {
    gazebo_geometry->LinkEndChild(geometry_type);
    _elem->LinkEndChild(gazebo_geometry);
  }
}


std::string URDF2Gazebo::GetGeometryBoundingBox(
  boost::shared_ptr<urdf::Geometry> _geometry, double *_sizeVals)
{
  std::string type;

  switch (_geometry->type)
  {
  case urdf::Geometry::BOX:
      type = "box";
      {
        boost::shared_ptr<const urdf::Box> box;
        box = boost::dynamic_pointer_cast<const urdf::Box >(_geometry);
        _sizeVals[0] = box->dim.x;
        _sizeVals[1] = box->dim.y;
        _sizeVals[2] = box->dim.z;
      }
      break;
  case urdf::Geometry::CYLINDER:
      type = "cylinder";
      {
        boost::shared_ptr<const urdf::Cylinder> cylinder;
        cylinder = boost::dynamic_pointer_cast<const urdf::Cylinder >(_geometry);
        _sizeVals[0] = cylinder->radius * 2;
        _sizeVals[1] = cylinder->radius * 2;
        _sizeVals[2] = cylinder->length;
      }
      break;
  case urdf::Geometry::SPHERE:
      type = "sphere";
      {
        boost::shared_ptr<const urdf::Sphere> sphere;
        sphere = boost::dynamic_pointer_cast<const urdf::Sphere >(_geometry);
        _sizeVals[0] = _sizeVals[1] = _sizeVals[2] = sphere->radius * 2;
      }
      break;
  case urdf::Geometry::MESH:
      type = "trimesh";
      {
        boost::shared_ptr<const urdf::Mesh> mesh;
        mesh = boost::dynamic_pointer_cast<const urdf::Mesh >(_geometry);
        _sizeVals[0] = mesh->scale.x;
        _sizeVals[1] = mesh->scale.y;
        _sizeVals[2] = mesh->scale.z;
      }
      break;
  default:
      _sizeVals[0] = _sizeVals[1] = _sizeVals[2] = 0;
      gzwarn << "Unknown body type: [" << _geometry->type
             << "] skipped in geometry\n";
      break;
  }

  return type;
}

void URDF2Gazebo::PrintMass(std::string _linkName, dMass _mass)
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

void URDF2Gazebo::PrintMass(LinkPtr _link)
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

void URDF2Gazebo::ReduceFixedJoints(TiXmlElement *_root, LinkPtr _link)
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

void URDF2Gazebo::PrintCollisionGroups(LinkPtr _link)
{
  gzdbg << "COLLISION LUMPING: link: [" << _link->name << "] contains ["
        << static_cast<int>(_link->collision_groups.size())
        << "] collisions.\n";
  for (std::map<std::string,
    boost::shared_ptr<std::vector<CollisionPtr > > >::iterator
    cols_it = _link->collision_groups.begin();
    cols_it != _link->collision_groups.end(); ++cols_it)
  {
    gzdbg << "    collision_groups: [" << cols_it->first << "] has ["
          << static_cast<int>(cols_it->second->size())
          << "] Collision objects\n";
  }
}

/// \brief returns the same transform in parent frame
urdf::Pose  URDF2Gazebo::TransformToParentFrame(
  urdf::Pose _transformInLinkFrame, urdf::Pose _parentToLinkTransform)
{
  // transform to gazebo::math::Pose then call TransformToParentFrame
  gazebo::math::Pose p1 = URDF2Gazebo::CopyPose(_transformInLinkFrame);
  gazebo::math::Pose p2 = URDF2Gazebo::CopyPose(_parentToLinkTransform);
  return URDF2Gazebo::CopyPose(TransformToParentFrame(p1, p2));
}
gazebo::math::Pose  URDF2Gazebo::TransformToParentFrame(
  gazebo::math::Pose _transformInLinkFrame,
  urdf::Pose _parentToLinkTransform)
{
  // transform to gazebo::math::Pose then call TransformToParentFrame
  gazebo::math::Pose p2 = URDF2Gazebo::CopyPose(_parentToLinkTransform);
  return TransformToParentFrame(_transformInLinkFrame, p2);
}

gazebo::math::Pose  URDF2Gazebo::TransformToParentFrame(
  gazebo::math::Pose _transformInLinkFrame,
  gazebo::math::Pose _parentToLinkTransform)
{
  gazebo::math::Pose transform_in_parent_link_frame;
  // rotate link pose to parent_link frame
  transform_in_parent_link_frame.pos =
    _parentToLinkTransform.rot * _transformInLinkFrame.pos;
  transform_in_parent_link_frame.rot =
    _parentToLinkTransform.rot * _transformInLinkFrame.rot;
  // translate link to parent_link frame
  transform_in_parent_link_frame.pos =
    _parentToLinkTransform.pos + transform_in_parent_link_frame.pos;

  return transform_in_parent_link_frame;
}

gazebo::math::Pose  URDF2Gazebo::inverseTransformToParentFrame(
  gazebo::math::Pose _transformInLinkFrame,
  urdf::Pose _parentToLinkTransform)
{
  gazebo::math::Pose transform_in_parent_link_frame;
  //   rotate link pose to parent_link frame
  urdf::Rotation ri = _parentToLinkTransform.rotation.GetInverse();
  gazebo::math::Quaternion q1(ri.w, ri.x, ri.y, ri.z);
  transform_in_parent_link_frame.pos = q1 * _transformInLinkFrame.pos;
  urdf::Rotation r2 = _parentToLinkTransform.rotation.GetInverse();
  gazebo::math::Quaternion q3(r2.w, r2.x, r2.y, r2.z);
  transform_in_parent_link_frame.rot = q3 * _transformInLinkFrame.rot;
  //   translate link to parent_link frame
  transform_in_parent_link_frame.pos.x = transform_in_parent_link_frame.pos.x
    - _parentToLinkTransform.position.x;
  transform_in_parent_link_frame.pos.y = transform_in_parent_link_frame.pos.y
    - _parentToLinkTransform.position.y;
  transform_in_parent_link_frame.pos.z = transform_in_parent_link_frame.pos.z
    - _parentToLinkTransform.position.z;

  return transform_in_parent_link_frame;
}

/// Take the link's existing list of gazebo extensions, transfer them
/// into parent link.  Along the way, update local transforms by adding
/// the additional transform to parent.  Also, look through all
/// referenced link names with plugins and update references to current
/// link to the parent link. (ReduceGazeboExtensionFrameReplace())
void URDF2Gazebo::ReduceGazeboExtensionToParent(LinkPtr _link)
{
  /// @todo: this is a very complicated module that updates the plugins
  /// based on fixed joint reduction really wish this could be a lot cleaner

  std::string linkName = _link->name;

  // update extension map with references to linkName
  // this->ListGazeboExtensions();
  std::map<std::string, std::vector<GazeboExtension*> >::iterator ext =
    this->gazeboEntensions.find(linkName);
  if (ext != this->gazeboEntensions.end())
  {
    // gzdbg << "  REDUCE EXTENSION: moving reference from ["
    //       << linkName << "] to [" << _link->getParent()->name << "]\n";

    // update reduction transform (for rays, cameras for now).
    //   FIXME: contact frames too?
    for (std::vector<GazeboExtension*>::iterator ge = ext->second.begin();
         ge != ext->second.end(); ++ge)
    {
      (*ge)->reductionTransform = TransformToParentFrame(
        (*ge)->reductionTransform,
        _link->parent_joint->parent_to_joint_origin_transform);
      // for sensor and projector blocks only
      ReduceGazeboExtensionsTransformReduction((*ge));
    }

    // find pointer to the existing extension with the new _link reference
    std::string newLinkName = _link->getParent()->name;
    std::map<std::string, std::vector<GazeboExtension*> >::iterator
      new_ext = this->gazeboEntensions.find(newLinkName);

    // if none exist, create new_extension with newLinkName
    if (new_ext == this->gazeboEntensions.end())
    {
      std::vector<GazeboExtension*> extensions;
      this->gazeboEntensions.insert(std::make_pair(
        newLinkName, extensions));
      new_ext = this->gazeboEntensions.find(newLinkName);
    }

    // move gazebo extensions from _link into the parent _link's extensions
    for (std::vector<GazeboExtension*>::iterator ge = ext->second.begin();
         ge != ext->second.end(); ++ge)
      new_ext->second.push_back(*ge);
    ext->second.clear();
  }

  // for extensions with empty reference, search and replace
  // _link name patterns within the plugin with new _link name
  // and assign the proper reduction transform for the _link name pattern
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazebo_it = this->gazeboEntensions.begin();
       gazebo_it != this->gazeboEntensions.end(); ++gazebo_it)
    {
      // update reduction transform (for contacts, rays, cameras for now).
      for (std::vector<GazeboExtension*>::iterator
        ge = gazebo_it->second.begin(); ge != gazebo_it->second.end(); ++ge)
        ReduceGazeboExtensionFrameReplace(*ge, _link);
    }

  // this->ListGazeboExtensions();
}

void URDF2Gazebo::ReduceGazeboExtensionFrameReplace(GazeboExtension* _ge,
                                                    LinkPtr _link)
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
    std::ostringstream debug_stream_in;
    debug_stream_in << *(*blobIt);
    // std::string debug_blob = debug_stream_in.str();
    // gzdbg << "        INITIAL STRING link ["
    //       << linkName << "]-->[" << newLinkName << "]: ["
    //       << debug_blob << "]\n";

    this->ReduceGazeboExtensionContactSensorFrameReplace(blobIt, _link);
    this->ReduceGazeboExtensionPluginFrameReplace(blobIt, _link,
      "plugin", "bodyName", _ge->reductionTransform);
    this->ReduceGazeboExtensionPluginFrameReplace(blobIt, _link,
      "plugin", "frameName", _ge->reductionTransform);
    this->ReduceGazeboExtensionProjectorFrameReplace(blobIt, _link);
    this->ReduceGazeboExtensionGripperFrameReplace(blobIt, _link);
    this->ReduceGazeboExtensionJointFrameReplace(blobIt, _link);

    std::ostringstream debug_stream_out;
    debug_stream_out << *(*blobIt);
  }
}

void URDF2Gazebo::ReduceGazeboExtensionsTransformReduction(GazeboExtension* _ge)
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


void URDF2Gazebo::ListGazeboExtensions()
{
  gzdbg << "================================================================\n";
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazebo_it = this->gazeboEntensions.begin();
       gazebo_it != this->gazeboEntensions.end(); ++gazebo_it)
  {
    int ext_count = 0;
    for (std::vector<GazeboExtension*>::iterator ge = gazebo_it->second.begin();
         ge != gazebo_it->second.end(); ++ge)
    {
      if ((*ge)->blobs.size() > 0)
      {
        gzdbg <<  "  PRINTING [" << static_cast<int>((*ge)->blobs.size())
              << "] BLOBS for extension [" << ++ext_count
              << "] referencing [" << gazebo_it->first << "]\n";
        for (std::vector<TiXmlElement*>::iterator
          blobIt = (*ge)->blobs.begin();
          blobIt != (*ge)->blobs.end(); ++blobIt)
        {
          std::ostringstream stream_in;
          stream_in << *(*blobIt);
          gzdbg << "    BLOB: [" << stream_in.str() << "]\n";
        }
      }
    }
  }
  gzdbg << "================================================================\n";
}

void URDF2Gazebo::ListGazeboExtensions(std::string _reference)
{
  gzdbg << "================================================================\n";
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazebo_it = this->gazeboEntensions.begin();
       gazebo_it != this->gazeboEntensions.end(); ++gazebo_it)
  {
    if (gazebo_it->first == _reference)
    {
        gzdbg <<  "  PRINTING [" << static_cast<int>(gazebo_it->second.size())
              << "] extensions referencing [" << _reference << "]\n";
      for (std::vector<GazeboExtension*>::iterator
           ge = gazebo_it->second.begin(); ge != gazebo_it->second.end(); ++ge)
      {
        for (std::vector<TiXmlElement*>::iterator
          blobIt = (*ge)->blobs.begin();
          blobIt != (*ge)->blobs.end(); ++blobIt)
        {
          std::ostringstream stream_in;
          stream_in << *(*blobIt);
          gzdbg << "    BLOB: [" << stream_in.str() << "]\n";
        }
      }
    }
  }
  gzdbg << "================================================================\n";
}

void URDF2Gazebo::CreateSDF(TiXmlElement *_root,
  ConstLinkPtr _link,
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

void URDF2Gazebo::CreateLink(TiXmlElement *_root,
  ConstLinkPtr _link,
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
  AddTransform(elem, _currentTransform);

  /* create new inerial block */
  CreateInertial(elem, _link);

  /* create new collision block */
  CreateCollisions(elem, _link);

  /* create new visual block */
  CreateVisuals(elem, _link);

  /* copy gazebo extensions data */
  InsertGazeboExtensionLink(elem, _link->name);

  /* add body to document */
  _root->LinkEndChild(elem);

  /* make a <joint:...> block */
  CreateJoint(_root, _link, _currentTransform);
}

void URDF2Gazebo::CreateCollisions(TiXmlElement* _elem,
  ConstLinkPtr _link)
{
  // loop through all collisions. make additional geoms using the lumped stuff
  for (std::map<std::string,
    boost::shared_ptr<std::vector<CollisionPtr> > >::const_iterator
    collisions_it = _link->collision_groups.begin();
    collisions_it != _link->collision_groups.end(); ++collisions_it)
  {
    unsigned int defaultMeshCount = 0;
    unsigned int groupMeshCount = 0;
    unsigned int lumpMeshCount = 0;
    for (std::vector<CollisionPtr>::iterator
         collision = collisions_it->second->begin();
         collision != collisions_it->second->end();
         ++collision)
    {
      if (collisions_it->first == "default")
      {
        // gzdbg << "creating default collision for link [" << _link->name
        //       << "]";

        std::string collisionPrefix = _link->name;

        if (defaultMeshCount > 0)
        {
          // append _[meshCount] to link name for additional collisions
          std::ostringstream collision_name_stream;
          collision_name_stream << collisionPrefix << "_" << defaultMeshCount;
          collisionPrefix = collision_name_stream.str();
        }

        /* make a <collision> block */
        CreateCollision(_elem, _link, *collision, collisionPrefix);

        // only 1 default mesh
        ++defaultMeshCount;
      }
      else if (collisions_it->first.find(std::string("lump::")) == 0)
      {
        // if collision name starts with "lump::", pass through
        //   original parent link name
        // gzdbg << "creating lump collision [" << collisions_it->first
        //       << "] for link [" << _link->name << "].\n";
        /// collisionPrefix is the original name before lumping
        std::string collisionPrefix = collisions_it->first.substr(6);

        if (lumpMeshCount > 0)
        {
          // append _[meshCount] to link name for additional collisions
          std::ostringstream collision_name_stream;
          collision_name_stream << collisionPrefix << "_" << lumpMeshCount;
          collisionPrefix = collision_name_stream.str();
        }

        CreateCollision(_elem, _link, *collision, collisionPrefix);
        ++lumpMeshCount;
      }
      else
      {
        // gzdbg << "adding collisions from collision group ["
        //      << collisions_it->first << "]\n";

        std::string collisionPrefix = _link->name + std::string("_") +
                                      collisions_it->first;

        if (groupMeshCount > 0)
        {
          // append _[meshCount] to _link name for additional collisions
          std::ostringstream collision_name_stream;
          collision_name_stream << collisionPrefix << "_" << groupMeshCount;
          collisionPrefix = collision_name_stream.str();
        }

        CreateCollision(_elem, _link, *collision, collisionPrefix);
        ++groupMeshCount;
      }
    }
  }
}

void URDF2Gazebo::CreateVisuals(TiXmlElement* _elem,
  ConstLinkPtr _link)
{
  // loop through all visuals. make additional visuals using the
  //   lumped stuff
  for (std::map<std::string,
    boost::shared_ptr<std::vector<VisualPtr> > >::const_iterator
    visuals_it = _link->visual_groups.begin();
    visuals_it != _link->visual_groups.end(); ++visuals_it)
  {
    unsigned int defaultMeshCount = 0;
    unsigned int groupMeshCount = 0;
    unsigned int lumpMeshCount = 0;
    for (std::vector<VisualPtr>::iterator
         visual = visuals_it->second->begin();
         visual != visuals_it->second->end();
         ++visual)
    {
      if (visuals_it->first == "default")
      {
        // gzdbg << "creating default visual for link [" << _link->name
        //       << "]";

        std::string visualPrefix = _link->name;

        if (defaultMeshCount > 0)
        {
          // append _[meshCount] to _link name for additional visuals
          std::ostringstream visual_name_stream;
          visual_name_stream << visualPrefix << "_" << defaultMeshCount;
          visualPrefix = visual_name_stream.str();
        }

        /* make a <visual> block */
        CreateVisual(_elem, _link, *visual, visualPrefix);

        // only 1 default mesh
        ++defaultMeshCount;
      }
      else if (visuals_it->first.find(std::string("lump::")) == 0)
      {
        // if visual name starts with "lump::", pass through
        //   original parent link name
        // gzdbg << "creating lump visual [" << visuals_it->first
        //       << "] for link [" << _link->name << "].\n";
        /// visualPrefix is the original name before lumping
        std::string visualPrefix = visuals_it->first.substr(6);

        if (lumpMeshCount > 0)
        {
          // append _[meshCount] to _link name for additional visuals
          std::ostringstream visual_name_stream;
          visual_name_stream << visualPrefix << "_" << lumpMeshCount;
          visualPrefix = visual_name_stream.str();
        }

        CreateVisual(_elem, _link, *visual, visualPrefix);
        ++lumpMeshCount;
      }
      else
      {
        // gzdbg << "adding visuals from visual group ["
        //      << visuals_it->first << "]\n";

        std::string visualPrefix = _link->name + std::string("_") +
                                      visuals_it->first;

        if (groupMeshCount > 0)
        {
          // append _[meshCount] to _link name for additional visuals
          std::ostringstream visual_name_stream;
          visual_name_stream << visualPrefix << "_" << groupMeshCount;
          visualPrefix = visual_name_stream.str();
        }

        CreateVisual(_elem, _link, *visual, visualPrefix);
        ++groupMeshCount;
      }
    }
  }
}


void URDF2Gazebo::CreateInertial(TiXmlElement *_elem,
  ConstLinkPtr _link)
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
  AddTransform(inertial, pose);

  // add mass
  AddKeyValue(inertial, "mass", Values2str(1, &_link->inertial->mass));

  // add inertia (ixx, ixy, ixz, iyy, iyz, izz)
  TiXmlElement *inertia = new TiXmlElement("inertia");
  AddKeyValue(inertia, "ixx", Values2str(1, &_link->inertial->ixx));
  AddKeyValue(inertia, "ixy", Values2str(1, &_link->inertial->ixy));
  AddKeyValue(inertia, "ixz", Values2str(1, &_link->inertial->ixz));
  AddKeyValue(inertia, "iyy", Values2str(1, &_link->inertial->iyy));
  AddKeyValue(inertia, "iyz", Values2str(1, &_link->inertial->iyz));
  AddKeyValue(inertia, "izz", Values2str(1, &_link->inertial->izz));
  inertial->LinkEndChild(inertia);

  _elem->LinkEndChild(inertial);
}


void URDF2Gazebo::CreateJoint(TiXmlElement *_root,
  ConstLinkPtr _link,
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
        AddKeyValue(joint, "child", _link->name);
        AddKeyValue(joint, "parent", _link->getParent()->name);

        TiXmlElement *joint_axis = new TiXmlElement("axis");
        TiXmlElement *joint_axis_limit = new TiXmlElement("limit");
        TiXmlElement *joint_axis_dynamics = new TiXmlElement("dynamics");
        if (jtype == "fixed")
        {
          AddKeyValue(joint_axis_limit, "lower", "0");
          AddKeyValue(joint_axis_limit, "upper", "0");
          AddKeyValue(joint_axis_dynamics, "damping", "0");
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
          AddKeyValue(joint_axis, "xyz", Values2str(3, rotatedJointAxisArray));
          if (_link->parent_joint->dynamics)
            AddKeyValue(joint_axis_dynamics, "damping",
              Values2str(1, &_link->parent_joint->dynamics->damping));

          if (this->enforceLimits && _link->parent_joint->limits)
          {
            if (jtype == "slider")
            {
              AddKeyValue(joint_axis_limit, "lower",
                Values2str(1, &_link->parent_joint->limits->lower));
              AddKeyValue(joint_axis_limit, "upper",
                Values2str(1, &_link->parent_joint->limits->upper));
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
              AddKeyValue(joint_axis_limit, "lower",
                Values2str(1, &_link->parent_joint->limits->lower));
              AddKeyValue(joint_axis_limit, "upper",
                Values2str(1, &_link->parent_joint->limits->upper));
            }
          }
        }
        joint_axis->LinkEndChild(joint_axis_limit);
        joint_axis->LinkEndChild(joint_axis_dynamics);
        joint->LinkEndChild(joint_axis);

        /* copy gazebo extensions data */
        InsertGazeboExtensionJoint(joint, _link->parent_joint->name);

        /* add joint to document */
        _root->LinkEndChild(joint);
    }
}


void URDF2Gazebo::CreateCollision(TiXmlElement* _elem,
  ConstLinkPtr _link,
  CollisionPtr _collision,
  std::string _old_link_name)
{
    /* begin create geometry node, skip if no collision specified */
    TiXmlElement *gazebo_collision = new TiXmlElement("collision");

    /* set its name, if lumped, add original link name */
    if (_old_link_name == _link->name)
      gazebo_collision->SetAttribute("name",
        _link->name + std::string("_collision"));
    else
      gazebo_collision->SetAttribute("name",
        _link->name + std::string("_collision_") + _old_link_name);

    /* set transform */
    double pose[6];
    pose[0] = _collision->origin.position.x;
    pose[1] = _collision->origin.position.y;
    pose[2] = _collision->origin.position.z;
    _collision->origin.rotation.getRPY(pose[3], pose[4], pose[5]);
    AddKeyValue(gazebo_collision, "pose", Values2str(6, pose));


    /* add geometry block */
    if (!_collision || !_collision->geometry)
    {
      // gzdbg << "urdf2gazebo: collision of link [" << _link->name
      //       << "] has no <geometry>.\n";
    }
    else
    {
      CreateGeometry(gazebo_collision, _collision->geometry);
    }

    /* set additional data from extensions */
    InsertGazeboExtensionCollision(gazebo_collision, _old_link_name);

    /* add geometry to body */
    _elem->LinkEndChild(gazebo_collision);
}

void URDF2Gazebo::CreateVisual(TiXmlElement *_elem,
  ConstLinkPtr _link,
  VisualPtr _visual, std::string _old_link_name)
{
    /* begin create gazebo visual node */
    TiXmlElement *gazebo_visual = new TiXmlElement("visual");

    /* set its name */
    // gzdbg << "original link name [" << _old_link_name
    //       << "] new link name [" << _link->name << "]\n";
    if (_old_link_name == _link->name)
      gazebo_visual->SetAttribute("name", _link->name + std::string("_vis"));
    else
      gazebo_visual->SetAttribute("name", _link->name + std::string("_vis_")
        + _old_link_name);

    /* add the visualisation transfrom */
    double pose[6];
    pose[0] = _visual->origin.position.x;
    pose[1] = _visual->origin.position.y;
    pose[2] = _visual->origin.position.z;
    _visual->origin.rotation.getRPY(pose[3], pose[4], pose[5]);
    AddKeyValue(gazebo_visual, "pose", Values2str(6, pose));

    /* insert geometry */
    if (!_visual || !_visual->geometry)
    {
      // gzdbg << "urdf2gazebo: visual of link [" << _link->name
      //       << "] has no <geometry>\n.";
    }
    else
      CreateGeometry(gazebo_visual, _visual->geometry);

    /* set additional data from extensions */
    InsertGazeboExtensionVisual(gazebo_visual, _old_link_name);

    /* end create _visual node */
    _elem->LinkEndChild(gazebo_visual);
}

TiXmlDocument URDF2Gazebo::InitModelString(std::string _urdfStr,
  bool _enforceLimits)
{
    this->enforceLimits = _enforceLimits;
    return this->InitModelString(_urdfStr);
}

TiXmlDocument URDF2Gazebo::InitModelString(std::string _urdfStr)
{
    /* Create a RobotModel from string */
    boost::shared_ptr<urdf::ModelInterface> robot_model =
      urdf::parseURDF(_urdfStr.c_str());

    // an xml object to hold the xml result
    TiXmlDocument gazebo_xml_out;

    if (!robot_model)
    {
        gzerr << "Unable to call parseURDF on robot model\n";
        return gazebo_xml_out;
    }

    /* create root element and define needed namespaces */
    TiXmlElement *robot = new TiXmlElement("model");

    // set model name to urdf robot name if not specified
    robot->SetAttribute("name", robot_model->getName());

    /* initialize transform for the model, urdf is recursive,
       while sdf defines all links relative to model frame */
    gazebo::math::Pose transform;

    /* parse gazebo extension */
    TiXmlDocument urdfXml;
    urdfXml.Parse(_urdfStr.c_str());
    ParseGazeboExtension(urdfXml);

    ConstLinkPtr root_link = robot_model->getRoot();

    /* Fixed Joint Reduction */
    /* if link connects to parent via fixed joint, lump down and remove link */
    /* set reduceFixedJoints to false will replace fixed joints with
       zero limit revolute joints, otherwise, we reduce it down to its
       parent link recursively */
    if (this->reduceFixedJoints)
      this->ReduceFixedJoints(robot,
        (boost::const_pointer_cast< urdf::Link >(root_link)));

    if (root_link->name == "world")
    {
      /* convert all children link */
      for (std::vector<LinkPtr>::const_iterator
        child = root_link->child_links.begin();
        child != root_link->child_links.end(); ++child)
          CreateSDF(robot, (*child), transform);
    }
    else
    {
      /* convert, starting from root link */
      CreateSDF(robot, root_link, transform);
    }

    /* insert the extensions without reference into <robot> root level */
    InsertGazeboExtensionRobot(robot);

    // add robot to gazebo_xml_out
    TiXmlElement *gazebo_sdf = new TiXmlElement("sdf");
    gazebo_sdf->SetAttribute("version", SDF_VERSION);
    gazebo_sdf->LinkEndChild(robot);
    gazebo_xml_out.LinkEndChild(gazebo_sdf);

    // debug
    // gazebo_xml_out.Print();

    return gazebo_xml_out;
}

TiXmlDocument URDF2Gazebo::InitModelDoc(TiXmlDocument* _xmlDoc)
{
    std::ostringstream stream;
    stream << *_xmlDoc;
    std::string urdfStr = stream.str();
    return InitModelString(urdfStr);
}

TiXmlDocument URDF2Gazebo::InitModelFile(std::string _filename)
{
  TiXmlDocument xmlDoc;
  if (xmlDoc.LoadFile(_filename))
  {
    return InitModelDoc(&xmlDoc);
  }
  else
    gzerr << "Unable to load file[" << _filename << "].\n";

    return xmlDoc;
}

void URDF2Gazebo::ReduceInertialToParent(LinkPtr _link)
{
    // gzdbg << "TREE:   mass lumping from [" << link->name
    //      << "] to [" << _link->getParent()->name << "]\n.";
    /* now lump all contents of this _link to parent */
    if (_link->inertial)
    {
      // get parent mass (in parent link frame)
      dMass parent_mass;
      if (!_link->getParent()->inertial)
        _link->getParent()->inertial.reset(new urdf::Inertial);
      dMassSetParameters(&parent_mass, _link->getParent()->inertial->mass,
        _link->getParent()->inertial->origin.position.x,
        _link->getParent()->inertial->origin.position.y,
        _link->getParent()->inertial->origin.position.z,
        _link->getParent()->inertial->ixx, _link->getParent()->inertial->iyy,
        _link->getParent()->inertial->izz, _link->getParent()->inertial->ixy,
         _link->getParent()->inertial->ixz, _link->getParent()->inertial->iyz);
      // PrintMass(_link->getParent()->name, parent_mass);
      // PrintMass(_link->getParent());
      // set _link mass (in _link frame)
      dMass link_mass;
      dMassSetParameters(&link_mass, _link->inertial->mass,
        _link->inertial->origin.position.x,
        _link->inertial->origin.position.y,
        _link->inertial->origin.position.z,
        _link->inertial->ixx, _link->inertial->iyy, _link->inertial->izz,
        _link->inertial->ixy, _link->inertial->ixz, _link->inertial->iyz);
      // PrintMass(_link->name, link_mass);
      // PrintMass(_link);
      // un-rotate _link mass into parent link frame
      dMatrix3 R;
      double phi, theta, psi;
      _link->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(
        phi, theta, psi);
      dRFromEulerAngles(R, phi, theta, psi);
      dMassRotate(&link_mass, R);
      // PrintMass(_link->name, link_mass);
      // un-translate _link mass into parent link frame
      dMassTranslate(&link_mass,
        _link->parent_joint->parent_to_joint_origin_transform.position.x,
        _link->parent_joint->parent_to_joint_origin_transform.position.y,
        _link->parent_joint->parent_to_joint_origin_transform.position.z);
      // PrintMass(_link->name, link_mass);
      // now link_mass is in the parent frame, add link_mass to parent_mass
      dMassAdd(&parent_mass, &link_mass);
      // PrintMass(_link->getParent()->name, parent_mass);
      // update parent mass
      _link->getParent()->inertial->mass = parent_mass.mass;
      _link->getParent()->inertial->ixx  = parent_mass.I[0+4*0];
      _link->getParent()->inertial->iyy  = parent_mass.I[1+4*1];
      _link->getParent()->inertial->izz  = parent_mass.I[2+4*2];
      _link->getParent()->inertial->ixy  = parent_mass.I[0+4*1];
      _link->getParent()->inertial->ixz  = parent_mass.I[0+4*2];
      _link->getParent()->inertial->iyz  = parent_mass.I[1+4*2];
      _link->getParent()->inertial->origin.position.x  = parent_mass.c[0];
      _link->getParent()->inertial->origin.position.y  = parent_mass.c[1];
      _link->getParent()->inertial->origin.position.z  = parent_mass.c[2];
      // PrintMass(_link->getParent());
    }
}

void URDF2Gazebo::ReduceVisualsToParent(LinkPtr _link)
{
  // lump visual to parent
  // lump all visual to parent, assign group name
  // "lump::"+group name+"::'+_link name
  // lump but keep the _link name in(/as) the group name,
  // so we can correlate visuals to visuals somehow.
  for (std::map<std::string,
    boost::shared_ptr<std::vector<VisualPtr> > >::iterator
    visuals_it = _link->visual_groups.begin();
    visuals_it != _link->visual_groups.end(); ++visuals_it)
  {
    /// @todo: extend to different groups,
    /// only work with default meshes right now.
    if (visuals_it->first == "default")
    {
      std::string lump_group_name = std::string("lump::")+_link->name;
      // gzdbg << "adding modified lump group name [" << lump_group_name
      //       << "] to link [" << _link->getParent()->name << "]\n.";
      for (std::vector<VisualPtr>::iterator
        visual_it = visuals_it->second->begin();
        visual_it != visuals_it->second->end(); ++visual_it)
      {
        // transform visual origin from _link frame to
        // parent link frame before adding to parent
        (*visual_it)->origin = TransformToParentFrame((*visual_it)->origin,
          _link->parent_joint->parent_to_joint_origin_transform);
        // add the modified visual to parent
        this->ReduceVisualToParent(_link->getParent(), lump_group_name,
          *visual_it);
      }
    }
    else if (visuals_it->first.find(std::string("lump::")) == 0)
    {
      // it's a previously lumped mesh, re-lump under same _groupName
      std::string lump_group_name = visuals_it->first;
      // gzdbg << "re-lumping group name [" << lump_group_name
      //       << "] to link [" << _link->getParent()->name << "]\n";
      for (std::vector<VisualPtr>::iterator
           visual_it = visuals_it->second->begin();
           visual_it != visuals_it->second->end(); ++visual_it)
      {
        // transform visual origin from _link frame to parent link
        // frame before adding to parent
        (*visual_it)->origin = TransformToParentFrame((*visual_it)->origin,
          _link->parent_joint->parent_to_joint_origin_transform);
        // add the modified visual to parent
        this->ReduceVisualToParent(_link->getParent(), lump_group_name,
          *visual_it);
      }
    }
  }
}

void URDF2Gazebo::ReduceCollisionsToParent(LinkPtr _link)
{
    // lump collision parent
    // lump all collision to parent, assign group name
    // "lump::"+group name+"::'+_link name
    // lump but keep the _link name in(/as) the group name,
    // so we can correlate visuals to collisions somehow.
    for (std::map<std::string,
      boost::shared_ptr<std::vector<CollisionPtr> > >::iterator
      collisions_it = _link->collision_groups.begin();
      collisions_it != _link->collision_groups.end(); ++collisions_it)
    {
      if (collisions_it->first == "default")
      {
        // if it's a "default" mesh, it will be added under "lump::"+_link name
        std::string lump_group_name = std::string("lump::")+_link->name;
        // gzdbg << "lumping collision [" << collisions_it->first
        //       << "] for link [" << _link->name
        //       << "] to parent [" << _link->getParent()->name
        //       << "] with group name [" << lump_group_name << "]\n";
        for (std::vector<CollisionPtr>::iterator
          collision_it = collisions_it->second->begin();
          collision_it != collisions_it->second->end(); ++collision_it)
        {
          // transform collision origin from _link frame to
          // parent link frame before adding to parent
          (*collision_it)->origin = TransformToParentFrame(
            (*collision_it)->origin,
            _link->parent_joint->parent_to_joint_origin_transform);

          // add the modified collision to parent
          this->ReduceCollisionToParent(_link->getParent(), lump_group_name,
            *collision_it);
        }
      }
      else if (collisions_it->first.find(std::string("lump::")) == 0)
      {
        // if it's a previously lumped mesh, relump under same _groupName
        std::string lump_group_name = collisions_it->first;
        // gzdbg << "re-lumping collision [" << collisions_it->first
        //       << "] for link [" << _link->name
        //       << "] to parent [" << _link->getParent()->name
        //       << "] with group name [" << lump_group_name << "]\n";
        for (std::vector<CollisionPtr>::iterator
          collision_it = collisions_it->second->begin();
          collision_it != collisions_it->second->end(); ++collision_it)
        {
          // transform collision origin from _link frame to
          // parent link frame before adding to parent
          (*collision_it)->origin = TransformToParentFrame(
            (*collision_it)->origin,
            _link->parent_joint->parent_to_joint_origin_transform);
          // add the modified collision to parent
          this->ReduceCollisionToParent(_link->getParent(), lump_group_name,
            *collision_it);
        }
      }
    }
    // PrintCollisionGroups(_link->getParent());
}

void URDF2Gazebo::ReduceJointsToParent(LinkPtr _link)
{
    // set child link's parent_joint's parent link to
    // a parent link up stream that does not have a fixed parent_joint
    for (unsigned int i = 0 ; i < _link->child_links.size() ; ++i)
    {
      boost::shared_ptr<urdf::Joint> parent_joint =
        _link->child_links[i]->parent_joint;
      if (parent_joint->type != urdf::Joint::FIXED)
      {
        // go down the tree until we hit a parent joint that is not fixed
        LinkPtr new_parent_link = _link;
        gazebo::math::Pose joint_anchor_transform;
        while (new_parent_link->parent_joint &&
              new_parent_link->getParent()->name != "world" &&
              new_parent_link->parent_joint->type == urdf::Joint::FIXED)
        {
          joint_anchor_transform = joint_anchor_transform *
            joint_anchor_transform;
          parent_joint->parent_to_joint_origin_transform =
            TransformToParentFrame(
            parent_joint->parent_to_joint_origin_transform,
            new_parent_link->parent_joint->parent_to_joint_origin_transform);
          new_parent_link = new_parent_link->getParent();
        }
        // now set the _link->child_links[i]->parent_joint's parent link to
        // the new_parent_link
        _link->child_links[i]->setParent(new_parent_link);
        parent_joint->parent_link_name = new_parent_link->name;
        // and set the _link->child_links[i]->parent_joint's
        // parent_to_joint_orogin_transform as the aggregated anchor transform?
      }
    }
}

void URDF2Gazebo::ReduceGazeboExtensionSensorTransformReduction(
  std::vector<TiXmlElement*>::iterator _blobIt,
  gazebo::math::Pose _reductionTransform)
{
    // overwrite <xyz> and <rpy> if they exist
    if ((*_blobIt)->ValueStr() == "sensor")
    {
      /*
      // parse it and add/replace the reduction transform
      // find first instance of xyz and rpy, replace with reduction transform
      for (TiXmlNode* el_it = (*_blobIt)->FirstChild();
           el_it; el_it = el_it->NextSibling())
      {
        std::ostringstream stream_in;
        stream_in << *el_it;
        gzdbg << "    " << stream_in << "\n";
      }
      */

      {
        TiXmlNode* old_pose_key = (*_blobIt)->FirstChild("pose");
        /// @todo: FIXME:  we should read xyz, rpy and aggregate it to
        /// reductionTransform instead of just throwing the info away.
        if (old_pose_key)
          (*_blobIt)->RemoveChild(old_pose_key);
      }

      // convert reductionTransform to values
      urdf::Vector3 reduction_xyz(_reductionTransform.pos.x,
                                  _reductionTransform.pos.y,
                                  _reductionTransform.pos.z);
      urdf::Rotation reduction_q(_reductionTransform.rot.x,
                                 _reductionTransform.rot.y,
                                 _reductionTransform.rot.z,
                                 _reductionTransform.rot.w);

      urdf::Vector3 reduction_rpy;
      reduction_q.getRPY(reduction_rpy.x, reduction_rpy.y, reduction_rpy.z);

      // output updated pose to text
      std::ostringstream pose_stream;
      pose_stream << reduction_xyz.x << " " << reduction_xyz.y
                  << " " << reduction_xyz.z << " " << reduction_rpy.x
                  << " " << reduction_rpy.y << " " << reduction_rpy.z;
      TiXmlText* pose_txt = new TiXmlText(pose_stream.str());

      TiXmlElement* pose_key = new TiXmlElement("pose");
      pose_key->LinkEndChild(pose_txt);

      (*_blobIt)->LinkEndChild(pose_key);
    }
}

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
      for (TiXmlNode* el_it = (*_blobIt)->FirstChild();
        el_it; el_it = el_it->NextSibling())
      {
        std::ostringstream stream_in;
        stream_in << *el_it;
        gzdbg << "    " << stream_in << "\n";
      }
      */

      /* should read <pose>...</pose> and agregate reductionTransform */
      TiXmlNode* pose_key = (*_blobIt)->FirstChild("pose");
      // read pose and save it

      // remove the tag for now
      if (pose_key) (*_blobIt)->RemoveChild(pose_key);

      // convert reductionTransform to values
      urdf::Vector3 reduction_xyz(_reductionTransform.pos.x,
                                  _reductionTransform.pos.y,
                                  _reductionTransform.pos.z);
      urdf::Rotation reduction_q(_reductionTransform.rot.x,
                                 _reductionTransform.rot.y,
                                 _reductionTransform.rot.z,
                                 _reductionTransform.rot.w);

      urdf::Vector3 reduction_rpy;
      reduction_q.getRPY(reduction_rpy.x, reduction_rpy.y, reduction_rpy.z);

      // output updated pose to text
      std::ostringstream pose_stream;
      pose_stream << reduction_xyz.x << " " << reduction_xyz.y
                  << " " << reduction_xyz.z << " " << reduction_rpy.x
                  << " " << reduction_rpy.y << " " << reduction_rpy.z;
      TiXmlText* pose_txt = new TiXmlText(pose_stream.str());

      pose_key = new TiXmlElement("pose");
      pose_key->LinkEndChild(pose_txt);

      (*_blobIt)->LinkEndChild(pose_key);
    }
}

void URDF2Gazebo::ReduceGazeboExtensionContactSensorFrameReplace(
  std::vector<TiXmlElement*>::iterator _blobIt, LinkPtr _link)
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
        if (GetKeyValueAsString(collision->ToElement()) ==
          linkName + std::string("_collision"))
        {
          contact->RemoveChild(collision);
          TiXmlElement* collision_name_key = new TiXmlElement("collision");
          std::ostringstream collision_name_stream;
          collision_name_stream << newLinkName << "_collision_" << linkName;
          TiXmlText* collision_name_txt = new TiXmlText(
            collision_name_stream.str());
          collision_name_key->LinkEndChild(collision_name_txt);
          contact->LinkEndChild(collision_name_key);
        }
        // @todo: FIXME: chagning contact sensor's contact collision
        //   should trigger a update in sensor offset as well.
        //   But first we need to implement offsets in contact sensors
      }
    }
  }
}

void URDF2Gazebo::ReduceGazeboExtensionPluginFrameReplace(
  std::vector<TiXmlElement*>::iterator _blobIt, LinkPtr _link,
  std::string _pluginName, std::string _elementName,
  gazebo::math::Pose _reductionTransform)
{
  std::string linkName = _link->name;
  std::string newLinkName = _link->getParent()->name;
  if ((*_blobIt)->ValueStr() == _pluginName)
  {
    // replace element containing _link names to parent link names
    // find first instance of xyz and rpy, replace with reduction transform
    TiXmlNode* element_node = (*_blobIt)->FirstChild(_elementName);
    if (element_node)
    {
      if (GetKeyValueAsString(element_node->ToElement()) == linkName)
      {
        (*_blobIt)->RemoveChild(element_node);
        TiXmlElement* body_name_key = new TiXmlElement(_elementName);
        std::ostringstream body_name_stream;
        body_name_stream << newLinkName;
        TiXmlText* body_name_txt = new TiXmlText(body_name_stream.str());
        body_name_key->LinkEndChild(body_name_txt);
        (*_blobIt)->LinkEndChild(body_name_key);
        /// @todo update transforms for this gazebo plugin too

        // look for offset transforms, add reduction transform
        TiXmlNode* xyz_key = (*_blobIt)->FirstChild("xyzOffset");
        if (xyz_key)
        {
          urdf::Vector3 v1 = ParseVector3(xyz_key);
          _reductionTransform.pos = gazebo::math::Vector3(v1.x, v1.y, v1.z);
          // remove xyzOffset and rpyOffset
          (*_blobIt)->RemoveChild(xyz_key);
        }
        TiXmlNode* rpy_key = (*_blobIt)->FirstChild("rpyOffset");
        if (rpy_key)
        {
          urdf::Vector3 rpy = ParseVector3(rpy_key, M_PI/180.0);
          _reductionTransform.rot =
            gazebo::math::Quaternion::EulerToQuaternion(rpy.x, rpy.y, rpy.z);
          // remove xyzOffset and rpyOffset
          (*_blobIt)->RemoveChild(rpy_key);
        }

        // pass through the parent transform from fixed joint reduction
        _reductionTransform = inverseTransformToParentFrame(_reductionTransform,
          _link->parent_joint->parent_to_joint_origin_transform);

        // create new offset xml blocks
        xyz_key = new TiXmlElement("xyzOffset");
        rpy_key = new TiXmlElement("rpyOffset");

        // create new offset xml blocks
        urdf::Vector3 reduction_xyz(_reductionTransform.pos.x,
          _reductionTransform.pos.y,
          _reductionTransform.pos.z);
        urdf::Rotation reduction_q(_reductionTransform.rot.x,
          _reductionTransform.rot.y, _reductionTransform.rot.z,
          _reductionTransform.rot.w);

        std::ostringstream xyz_stream, rpy_stream;
        xyz_stream << reduction_xyz.x << " " << reduction_xyz.y << " "
                   << reduction_xyz.z;
        urdf::Vector3 reduction_rpy;
        reduction_q.getRPY(reduction_rpy.x, reduction_rpy.y, reduction_rpy.z);
        rpy_stream << reduction_rpy.x << " " << reduction_rpy.y << " "
                   << reduction_rpy.z;

        TiXmlText* xyz_txt = new TiXmlText(xyz_stream.str());
        TiXmlText* rpy_txt = new TiXmlText(rpy_stream.str());

        xyz_key->LinkEndChild(xyz_txt);
        rpy_key->LinkEndChild(rpy_txt);

        (*_blobIt)->LinkEndChild(xyz_key);
        (*_blobIt)->LinkEndChild(rpy_key);
      }
    }
  }
}

void URDF2Gazebo::ReduceGazeboExtensionProjectorFrameReplace(
  std::vector<TiXmlElement*>::iterator _blobIt, LinkPtr _link)
{
  std::string linkName = _link->name;
  std::string newLinkName = _link->getParent()->name;

  // updates _link reference for <projector> inside of
  // projector plugins
  // update from <projector>MyLinkName/MyProjectorName</projector>
  // to <projector>NewLinkName/MyProjectorName</projector>
  TiXmlNode* projector_elem = (*_blobIt)->FirstChild("projector");
  {
    if (projector_elem)
    {
      std::string projector_name =  GetKeyValueAsString(
        projector_elem->ToElement());
      // extract projector _link name and projector name
      size_t pos = projector_name.find("/");
      if (pos == std::string::npos)
        gzerr << "no slash in projector reference tag [" << projector_name
              << "], expecting linkName/projector_name.\n";
      std::string projector_link_name = projector_name.substr(0, pos);

      if (projector_link_name == linkName)
      {
        // do the replacement
        projector_name = newLinkName + "/" +
          projector_name.substr(pos+1, projector_name.size());

        (*_blobIt)->RemoveChild(projector_elem);
        TiXmlElement* body_name_key = new TiXmlElement("projector");
        std::ostringstream body_name_stream;
        body_name_stream << projector_name;
        TiXmlText* body_name_txt = new TiXmlText(body_name_stream.str());
        body_name_key->LinkEndChild(body_name_txt);
        (*_blobIt)->LinkEndChild(body_name_key);
      }
    }
  }
}

void URDF2Gazebo::ReduceGazeboExtensionGripperFrameReplace(
  std::vector<TiXmlElement*>::iterator _blobIt, LinkPtr _link)
{
  std::string linkName = _link->name;
  std::string newLinkName = _link->getParent()->name;

  if ((*_blobIt)->ValueStr() == "gripper")
  {
    TiXmlNode* gripper_link = (*_blobIt)->FirstChild("gripper_link");
    if (gripper_link)
    {
      if (GetKeyValueAsString(gripper_link->ToElement()) == linkName)
      {
        (*_blobIt)->RemoveChild(gripper_link);
        TiXmlElement* body_name_key = new TiXmlElement("gripper_link");
        std::ostringstream body_name_stream;
        body_name_stream << newLinkName;
        TiXmlText* body_name_txt = new TiXmlText(body_name_stream.str());
        body_name_key->LinkEndChild(body_name_txt);
        (*_blobIt)->LinkEndChild(body_name_key);
      }
    }
    TiXmlNode* palm_link = (*_blobIt)->FirstChild("palm_link");
    if (palm_link)
    {
      if (GetKeyValueAsString(palm_link->ToElement()) == linkName)
      {
        (*_blobIt)->RemoveChild(palm_link);
        TiXmlElement* body_name_key = new TiXmlElement("palm_link");
        std::ostringstream body_name_stream;
        body_name_stream << newLinkName;
        TiXmlText* body_name_txt = new TiXmlText(body_name_stream.str());
        body_name_key->LinkEndChild(body_name_txt);
        (*_blobIt)->LinkEndChild(body_name_key);
      }
    }
  }
}

void URDF2Gazebo::ReduceGazeboExtensionJointFrameReplace(
  std::vector<TiXmlElement*>::iterator _blobIt, LinkPtr _link)
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
      if (GetKeyValueAsString(parent->ToElement()) == linkName)
      {
        (*_blobIt)->RemoveChild(parent);
        TiXmlElement* parent_name_key = new TiXmlElement("parent");
        std::ostringstream parent_name_stream;
        parent_name_stream << newLinkName;
        TiXmlText* parent_name_txt = new TiXmlText(parent_name_stream.str());
        parent_name_key->LinkEndChild(parent_name_txt);
        (*_blobIt)->LinkEndChild(parent_name_key);
      }
    }
    TiXmlNode* child = (*_blobIt)->FirstChild("child");
    if (child)
    {
      if (GetKeyValueAsString(child->ToElement()) == linkName)
      {
        (*_blobIt)->RemoveChild(child);
        TiXmlElement* child_name_key = new TiXmlElement("child");
        std::ostringstream child_name_stream;
        child_name_stream << newLinkName;
        TiXmlText* child_name_txt = new TiXmlText(child_name_stream.str());
        child_name_key->LinkEndChild(child_name_txt);
        (*_blobIt)->LinkEndChild(child_name_key);
      }
    }
    /// @todo add anchor offsets if parent link changes location!
  }
}
}
