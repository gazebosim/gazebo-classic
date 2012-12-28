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

#include "common/SystemPaths.hh"

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
    this->enforce_limits = true;
    this->reduce_fixed_joints = true;
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

    boost::split(pieces, str, boost::is_any_of(" "));
    for (unsigned int i = 0; i < pieces.size(); ++i)
    {
      if (pieces[i] != "")
      {
        try
        {
          vals.push_back(scale
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

void URDF2Gazebo::reduceVisualToParent(boost::shared_ptr<urdf::Link> link,
       std::string group_name, boost::shared_ptr<urdf::Visual> visual)
{
  boost::shared_ptr<std::vector<boost::shared_ptr<urdf::Visual > > > viss
    = link->getVisuals(group_name);
  if (!viss)
  {
    // group does not exist, create one and add to map
    viss.reset(new std::vector<boost::shared_ptr<urdf::Visual > >);
    // new group name, create vector, add vector to map and
    //   add Visual to the vector
    link->visual_groups.insert(make_pair(group_name, viss));
    // gzdbg << "successfully added a new visual group name ["
    //       << group_name << "]\n";
  }

  // group exists, add Visual to the vector in the map if it's not there
  std::vector<boost::shared_ptr<urdf::Visual > >::iterator vis_it
    = find(viss->begin(), viss->end(), visual);
  if (vis_it != viss->end())
    gzwarn << "attempted to add visual to link ["
           << link->name
           << "], but it already exists under group ["
           << group_name << "]\n";
  else
    viss->push_back(visual);
}

void URDF2Gazebo::reduceCollisionToParent(boost::shared_ptr<urdf::Link> link,
      std::string group_name, boost::shared_ptr<urdf::Collision> collision)
{
  boost::shared_ptr<std::vector<boost::shared_ptr<urdf::Collision > > >
    cols = link->getCollisions(group_name);
  if (!cols)
  {
    // group does not exist, create one and add to map
    cols.reset(new std::vector<boost::shared_ptr<urdf::Collision > >);
    // new group name, create add vector to map and add Collision to the vector
    link->collision_groups.insert(make_pair(group_name, cols));
  }

  // group exists, add Collision to the vector in the map
  std::vector<boost::shared_ptr<urdf::Collision > >::iterator col_it =
    find(cols->begin(), cols->end(), collision);
  if (col_it != cols->end())
    gzwarn << "attempted to add collision to link ["
           << link->name
           << "], but it already exists under group ["
           << group_name << "]\n";
  else
    cols->push_back(collision);
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
  TiXmlElement* child_elem = elem->FirstChildElement(key);
  if (child_elem)
  {
    std::string old_value = getKeyValueAsString(child_elem);
    if (old_value != value)
      gzwarn << "multiple inconsistent <" << key
             << "> exists due to fixed joint reduction"
             << " overwriting previous value [" << old_value
             << "] with [" << value << "].\n";
    // else
    //   gzdbg << "multiple consistent <" << key
    //          << "> exists with [" << value
    //          << "] due to fixed joint reduction.\n";
    elem->RemoveChild(child_elem);  // remove old elem
  }

  TiXmlElement *ekey      = new TiXmlElement(key);
  TiXmlText    *text_ekey = new TiXmlText(value);
  ekey->LinkEndChild(text_ekey);
  elem->LinkEndChild(ekey);
}

void URDF2Gazebo::addTransform(TiXmlElement *elem,
  const::gazebo::math::Pose& transform)
{
  gazebo::math::Vector3 e = transform.rot.GetAsEuler();
  double cpose[6] = { transform.pos.x, transform.pos.y,
                      transform.pos.z, e.x, e.y, e.z };

  /* set geometry transform */
  addKeyValue(elem, "pose", values2str(6, cpose));
}

std::string URDF2Gazebo::getKeyValueAsString(TiXmlElement* elem)
{
  std::string value_str;
  if (elem->Attribute("value"))
  {
    value_str = elem->Attribute("value");
  }
  else if (elem->FirstChild())
  /// @todo: FIXME: comment out check for now, different tinyxml
  /// versions fails to compile:
  //  && elem->FirstChild()->Type() == TiXmlNode::TINYXML_TEXT)
  {
    value_str = elem->FirstChild()->ValueStr();
  }
  return value_str;
}

void URDF2Gazebo::parseGazeboExtension(TiXmlDocument &urdf_xml)
{
  TiXmlElement* robot_xml = urdf_xml.FirstChildElement("robot");

  // Get all Gazebo extension elements, put everything in
  //   this->gazebo_extensions_ map, containing a key string
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

    if (this->gazebo_extensions_.find(ref_str) ==
        this->gazebo_extensions_.end())
    {
        // create extension map for reference
        std::vector<GazeboExtension*> extensions;
        this->gazebo_extensions_.insert(std::make_pair(ref_str, extensions));
    }

    // create and insert a new GazeboExtension into the map
    GazeboExtension* gazebo = new GazeboExtension();

    // begin parsing xml node
    for (TiXmlElement *child_elem = gazebo_xml->FirstChildElement();
         child_elem; child_elem = child_elem->NextSiblingElement())
    {
      gazebo->old_link_name = ref_str;

      // go through all elements of the extension,
      //   extract what we know, and save the rest in blobs
      // @todo:  somehow use sdf definitions here instead of hard coded
      //         objects

      // material
      if (child_elem->ValueStr() == "material")
      {
          gazebo->material = getKeyValueAsString(child_elem);
      }
      else if (child_elem->ValueStr() == "static")
      {
        std::string value_str = getKeyValueAsString(child_elem);

        // default of setting static flag is false
        if (lowerStr(value_str) == "true" || lowerStr(value_str) == "yes" ||
            value_str == "1")
          gazebo->setStaticFlag = true;
        else
          gazebo->setStaticFlag = false;
      }
      else if (child_elem->ValueStr() == "gravity")
      {
        std::string value_str = getKeyValueAsString(child_elem);

        // default of gravity is true
        if (lowerStr(value_str) == "false" || lowerStr(value_str) == "no" ||
            value_str == "0")
          gazebo->gravity = false;
        else
          gazebo->gravity = true;
      }
      else if (child_elem->ValueStr() == "damping_factor")
      {
          gazebo->is_damping_factor = true;
          gazebo->damping_factor = boost::lexical_cast<double>(
              getKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "maxVel")
      {
          gazebo->is_maxVel = true;
          gazebo->maxVel = boost::lexical_cast<double>(
              getKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "minDepth")
      {
          gazebo->is_minDepth = true;
          gazebo->minDepth = boost::lexical_cast<double>(
            getKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "mu1")
      {
          gazebo->is_mu1 = true;
          gazebo->mu1 = boost::lexical_cast<double>(
            getKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "mu2")
      {
          gazebo->is_mu2 = true;
          gazebo->mu2 = boost::lexical_cast<double>(
            getKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "fdir1")
      {
          gazebo->fdir1 = getKeyValueAsString(child_elem);
      }
      else if (child_elem->ValueStr() == "kp")
      {
          gazebo->is_kp = true;
          gazebo->kp = boost::lexical_cast<double>(
            getKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "kd")
      {
          gazebo->is_kd = true;
          gazebo->kd = boost::lexical_cast<double>(
            getKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "self_collide")
      {
        std::string value_str = getKeyValueAsString(child_elem);

        // default of self_collide is false
        if (lowerStr(value_str) == "true" || lowerStr(value_str) == "yes" ||
            value_str == "1")
          gazebo->self_collide = true;
        else
          gazebo->self_collide = false;
      }
      else if (child_elem->ValueStr() == "laser_retro")
      {
          gazebo->is_laser_retro = true;
          gazebo->laser_retro = boost::lexical_cast<double>(
            getKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "stop_cfm")
      {
          gazebo->is_stop_cfm = true;
          gazebo->stop_cfm = boost::lexical_cast<double>(
            getKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "stop_erp")
      {
          gazebo->is_stop_erp = true;
          gazebo->stop_erp = boost::lexical_cast<double>(
            getKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "initial_joint_position")
      {
          gazebo->is_initial_joint_position = true;
          gazebo->initial_joint_position = boost::lexical_cast<double>(
            getKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "fudge_factor")
      {
          gazebo->is_fudge_factor = true;
          gazebo->fudge_factor = boost::lexical_cast<double>(
            getKeyValueAsString(child_elem).c_str());
      }
      else if (child_elem->ValueStr() == "provideFeedback")
      {
          std::string value_str = getKeyValueAsString(child_elem);

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
    (this->gazebo_extensions_.find(ref_str))->second.push_back(gazebo);
  }
}

void URDF2Gazebo::insertGazeboExtensionCollision(TiXmlElement *elem,
  std::string link_name)
{
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazebo_it = this->gazebo_extensions_.begin();
       gazebo_it != this->gazebo_extensions_.end(); ++gazebo_it)
  {
    for (std::vector<GazeboExtension*>::iterator ge = gazebo_it->second.begin();
         ge != gazebo_it->second.end(); ++ge)
    {
      if ((*ge)->old_link_name == link_name)
      {
        TiXmlElement *surface = new TiXmlElement("surface");
        TiXmlElement *friction = new TiXmlElement("friction");
        TiXmlElement *friction_ode = new TiXmlElement("ode");
        TiXmlElement *contact = new TiXmlElement("contact");
        TiXmlElement *contact_ode = new TiXmlElement("ode");

        // insert mu1, mu2, kp, kd for collision
        if ((*ge)->is_mu1)
          addKeyValue(friction_ode, "mu", values2str(1, &(*ge)->mu1));
        if ((*ge)->is_mu2)
          addKeyValue(friction_ode, "mu2", values2str(1, &(*ge)->mu2));
        if (!(*ge)->fdir1.empty())
          addKeyValue(friction_ode, "fdir1", (*ge)->fdir1);
        if ((*ge)->is_kp)
          addKeyValue(contact_ode, "kp", values2str(1, &(*ge)->kp));
        if ((*ge)->is_kd)
          addKeyValue(contact_ode, "kd", values2str(1, &(*ge)->kd));
        // max contact interpenetration correction velocity
        if ((*ge)->is_maxVel)
          addKeyValue(contact_ode, "max_vel", values2str(1, &(*ge)->maxVel));
        // contact interpenetration margin tolerance
        if ((*ge)->is_minDepth)
          addKeyValue(contact_ode, "min_depth",
                      values2str(1, &(*ge)->minDepth));
        if ((*ge)->is_laser_retro)
          addKeyValue(elem, "laser_retro", values2str(1, &(*ge)->laser_retro));

        contact->LinkEndChild(contact_ode);
        surface->LinkEndChild(contact);
        friction->LinkEndChild(friction_ode);
        surface->LinkEndChild(friction);
        elem->LinkEndChild(surface);
      }
    }
  }
}

void URDF2Gazebo::insertGazeboExtensionVisual(TiXmlElement *elem,
  std::string link_name)
{
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazebo_it = this->gazebo_extensions_.begin();
       gazebo_it != this->gazebo_extensions_.end(); ++gazebo_it)
  {
    for (std::vector<GazeboExtension*>::iterator ge = gazebo_it->second.begin();
         ge != gazebo_it->second.end(); ++ge)
    {
      if ((*ge)->old_link_name == link_name)
      {
        // insert material block
        if (!(*ge)->material.empty())
            addKeyValue(elem, "material", (*ge)->material);
      }
    }
  }
}

void URDF2Gazebo::insertGazeboExtensionLink(TiXmlElement *elem,
  std::string link_name)
{
    for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
         gazebo_it = this->gazebo_extensions_.begin();
         gazebo_it != this->gazebo_extensions_.end(); ++gazebo_it)
    {
      if (gazebo_it->first == link_name)
      {
        // gzdbg << "inserting extension with reference ["
        //       << link_name << "] into link.\n";
        for (std::vector<GazeboExtension*>::iterator ge =
             gazebo_it->second.begin(); ge != gazebo_it->second.end(); ++ge)
        {
          // insert gravity
          if ((*ge)->gravity)
              addKeyValue(elem, "gravity", "true");
          else
              addKeyValue(elem, "gravity", "false");

          // damping factor
          TiXmlElement *velocity_decay = new TiXmlElement("velocity_decay");
          if ((*ge)->is_damping_factor)
          {
            /// @todo separate linear and angular velocity decay
            addKeyValue(elem, "linear", values2str(1, &(*ge)->damping_factor));
            addKeyValue(elem, "angular", values2str(1, &(*ge)->damping_factor));
          }
          elem->LinkEndChild(velocity_decay);
          // self_collide tag
          if ((*ge)->self_collide)
              addKeyValue(elem, "self_collide", "true");
          else
              addKeyValue(elem, "self_collide", "false");
          // insert blobs into body
          for (std::vector<TiXmlElement*>::iterator
               blob_it = (*ge)->blobs.begin();
               blob_it != (*ge)->blobs.end(); ++blob_it)
          {
              elem->LinkEndChild((*blob_it)->Clone());
          }
        }
      }
    }
}

void URDF2Gazebo::insertGazeboExtensionJoint(TiXmlElement *elem,
  std::string joint_name)
{
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazebo_it = this->gazebo_extensions_.begin();
       gazebo_it != this->gazebo_extensions_.end(); ++gazebo_it)
  {
    if (gazebo_it->first == joint_name)
    {
      for (std::vector<GazeboExtension*>::iterator
           ge = gazebo_it->second.begin();
           ge != gazebo_it->second.end(); ++ge)
      {
        TiXmlElement *physics     = new TiXmlElement("physics");
        TiXmlElement *physicsOde = new TiXmlElement("ode");
        TiXmlElement *limit       = new TiXmlElement("limit");
        // insert stop_cfm, stop_erp, fudge_factor
        if ((*ge)->is_stop_cfm)
        {
          addKeyValue(limit, "erp", values2str(1, &(*ge)->stop_cfm));
        }
        if ((*ge)->is_stop_erp)
        {
          addKeyValue(limit, "cfm", values2str(1, &(*ge)->stop_erp));
        }
        /* gone
        if ((*ge)->is_initial_joint_position)
            addKeyValue(elem, "initial_joint_position",
              values2str(1, &(*ge)->initial_joint_position));
        */

        // insert provideFeedback
        if ((*ge)->provideFeedback)
            addKeyValue(physicsOde, "provide_feedback", "true");
        else
            addKeyValue(physicsOde, "provide_feedback", "false");

        // insert fudge_factor
        if ((*ge)->is_fudge_factor)
          addKeyValue(physicsOde, "fudge_factor",
                      values2str(1, &(*ge)->fudge_factor));

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
       gazebo_it = this->gazebo_extensions_.begin();
       gazebo_it != this->gazebo_extensions_.end(); ++gazebo_it)
  {
    if (gazebo_it->first.empty())
    {
      // no reference specified
      for (std::vector<GazeboExtension*>::iterator
        ge = gazebo_it->second.begin(); ge != gazebo_it->second.end(); ++ge)
      {
        // insert static flag
        if ((*ge)->setStaticFlag)
            addKeyValue(elem, "static", "true");
        else
            addKeyValue(elem, "static", "false");

        // copy extension containing blobs and without reference
        for (std::vector<TiXmlElement*>::iterator
             blob_it = (*ge)->blobs.begin();
             blob_it != (*ge)->blobs.end(); ++blob_it)
        {
            std::ostringstream stream_in;
            stream_in << *(*blob_it);
            elem->LinkEndChild((*blob_it)->Clone());
        }
      }
    }
  }
}

void URDF2Gazebo::createGeometry(TiXmlElement* elem,
  boost::shared_ptr<urdf::Geometry> geometry)
{
  int sizeCount;
  double sizeVals[3];

  TiXmlElement *gazebo_geometry = new TiXmlElement("geometry");

  std::string type;
  TiXmlElement *geometry_type = NULL;

  switch (geometry->type)
  {
  case urdf::Geometry::BOX:
    type = "box";
    sizeCount = 3;
    {
      boost::shared_ptr<const urdf::Box> box;
      box = boost::dynamic_pointer_cast< const urdf::Box >(geometry);
      sizeVals[0] = box->dim.x;
      sizeVals[1] = box->dim.y;
      sizeVals[2] = box->dim.z;
      geometry_type = new TiXmlElement(type);
      addKeyValue(geometry_type, "size", values2str(sizeCount, sizeVals));
    }
    break;
  case urdf::Geometry::CYLINDER:
    type = "cylinder";
    sizeCount = 2;
    {
      boost::shared_ptr<const urdf::Cylinder> cylinder;
      cylinder = boost::dynamic_pointer_cast<const urdf::Cylinder >(geometry);
      geometry_type = new TiXmlElement(type);
      addKeyValue(geometry_type, "length", values2str(1, &cylinder->length));
      addKeyValue(geometry_type, "radius", values2str(1, &cylinder->radius));
    }
    break;
  case urdf::Geometry::SPHERE:
    type = "sphere";
    sizeCount = 1;
    {
      boost::shared_ptr<const urdf::Sphere> sphere;
      sphere = boost::dynamic_pointer_cast<const urdf::Sphere >(geometry);
      geometry_type = new TiXmlElement(type);
      addKeyValue(geometry_type, "radius", values2str(1, &sphere->radius));
    }
    break;
  case urdf::Geometry::MESH:
    type = "mesh";
    sizeCount = 3;
    {
      boost::shared_ptr<const urdf::Mesh> mesh;
      mesh = boost::dynamic_pointer_cast<const urdf::Mesh >(geometry);
      sizeVals[0] = mesh->scale.x;
      sizeVals[1] = mesh->scale.y;
      sizeVals[2] = mesh->scale.z;
      geometry_type = new TiXmlElement(type);
      addKeyValue(geometry_type, "scale", vector32str(mesh->scale));
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
        addKeyValue(geometry_type, "uri", model_filename);
      }
    }
    break;
  default:
    sizeCount = 0;
    gzwarn << "Unknown body type: [" << geometry->type
           << "] skipped in geometry\n";
    break;
  }

  if (geometry_type)
  {
    gazebo_geometry->LinkEndChild(geometry_type);
    elem->LinkEndChild(gazebo_geometry);
  }
}


std::string URDF2Gazebo::getGeometryBoundingBox(
  boost::shared_ptr<urdf::Geometry> geometry, double *sizeVals)
{
  std::string type;

  switch (geometry->type)
  {
  case urdf::Geometry::BOX:
      type = "box";
      {
        boost::shared_ptr<const urdf::Box> box;
        box = boost::dynamic_pointer_cast<const urdf::Box >(geometry);
        sizeVals[0] = box->dim.x;
        sizeVals[1] = box->dim.y;
        sizeVals[2] = box->dim.z;
      }
      break;
  case urdf::Geometry::CYLINDER:
      type = "cylinder";
      {
        boost::shared_ptr<const urdf::Cylinder> cylinder;
        cylinder = boost::dynamic_pointer_cast<const urdf::Cylinder >(geometry);
        sizeVals[0] = cylinder->radius * 2;
        sizeVals[1] = cylinder->radius * 2;
        sizeVals[2] = cylinder->length;
      }
      break;
  case urdf::Geometry::SPHERE:
      type = "sphere";
      {
        boost::shared_ptr<const urdf::Sphere> sphere;
        sphere = boost::dynamic_pointer_cast<const urdf::Sphere >(geometry);
        sizeVals[0] = sizeVals[1] = sizeVals[2] = sphere->radius * 2;
      }
      break;
  case urdf::Geometry::MESH:
      type = "trimesh";
      {
        boost::shared_ptr<const urdf::Mesh> mesh;
        mesh = boost::dynamic_pointer_cast<const urdf::Mesh >(geometry);
        sizeVals[0] = mesh->scale.x;
        sizeVals[1] = mesh->scale.y;
        sizeVals[2] = mesh->scale.z;
      }
      break;
  default:
      sizeVals[0] = sizeVals[1] = sizeVals[2] = 0;
      gzwarn << "Unknown body type: [" << geometry->type
             << "] skipped in geometry\n";
      break;
  }

  return type;
}

void URDF2Gazebo::printMass(std::string link_name, dMass mass)
{
  gzdbg << "LINK NAME: [" << link_name << "] from dMass\n";
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

void URDF2Gazebo::printMass(boost::shared_ptr<urdf::Link> link)
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
  boost::shared_ptr<urdf::Link> link)
{
  // if child is attached to self by fixed link first go up the tree,
  //   check it's children recursively
  for (unsigned int i = 0 ; i < link->child_links.size() ; ++i)
    if (link->child_links[i]->parent_joint->type == urdf::Joint::FIXED)
      reduceFixedJoints(root, link->child_links[i]);

  // reduce this link's stuff up the tree to parent but skip first joint
  //   if it's the world
  if (link->getParent() && link->getParent()->name != "world" &&
      link->parent_joint && link->parent_joint->type == urdf::Joint::FIXED)
  {
    // gzdbg << "Fixed Joint Reduction: extension lumping from ["
    //       << link->name << "] to [" << link->getParent()->name << "]\n";

    // lump gazebo extensions to parent, (give them new reference link names)
    reduceGazeboExtensionToParent(link);

    // reduce link elements to parent
    this->reduceInertialToParent(link);
    this->reduceVisualsToParent(link);
    this->reduceCollisionsToParent(link);
    this->reduceJointsToParent(link);
  }

  // continue down the tree for non-fixed joints
  for (unsigned int i = 0 ; i < link->child_links.size() ; ++i)
    if (link->child_links[i]->parent_joint->type != urdf::Joint::FIXED)
      reduceFixedJoints(root, link->child_links[i]);
}

void URDF2Gazebo::printCollisionGroups(boost::shared_ptr<urdf::Link> link)
{
  gzdbg << "COLLISION LUMPING: link: [" << link->name << "] contains ["
        << static_cast<int>(link->collision_groups.size())
        << "] collisions.\n";
  for (std::map<std::string,
    boost::shared_ptr<std::vector<CollisionPtr > > >::iterator
    cols_it = link->collision_groups.begin();
    cols_it != link->collision_groups.end(); ++cols_it)
  {
    gzdbg << "    collision_groups: [" << cols_it->first << "] has ["
          << static_cast<int>(cols_it->second->size())
          << "] Collision objects\n";
  }
}

/// \brief returns the same transform in parent frame
urdf::Pose  URDF2Gazebo::transformToParentFrame(
  urdf::Pose transform_in_link_frame, urdf::Pose parent_to_link_transform)
{
  // transform to gazebo::math::Pose then call transformToParentFrame
  gazebo::math::Pose p1 = URDF2Gazebo::copyPose(transform_in_link_frame);
  gazebo::math::Pose p2 = URDF2Gazebo::copyPose(parent_to_link_transform);
  return URDF2Gazebo::copyPose(transformToParentFrame(p1, p2));
}
gazebo::math::Pose  URDF2Gazebo::transformToParentFrame(
  gazebo::math::Pose transform_in_link_frame,
  urdf::Pose parent_to_link_transform)
{
  // transform to gazebo::math::Pose then call transformToParentFrame
  gazebo::math::Pose p2 = URDF2Gazebo::copyPose(parent_to_link_transform);
  return transformToParentFrame(transform_in_link_frame, p2);
}

gazebo::math::Pose  URDF2Gazebo::transformToParentFrame(
  gazebo::math::Pose transform_in_link_frame,
  gazebo::math::Pose parent_to_link_transform)
{
  gazebo::math::Pose transform_in_parent_link_frame;
  // rotate link pose to parent_link frame
  transform_in_parent_link_frame.pos =
    parent_to_link_transform.rot * transform_in_link_frame.pos;
  transform_in_parent_link_frame.rot =
    parent_to_link_transform.rot * transform_in_link_frame.rot;
  // translate link to parent_link frame
  transform_in_parent_link_frame.pos =
    parent_to_link_transform.pos + transform_in_parent_link_frame.pos;

  return transform_in_parent_link_frame;
}

gazebo::math::Pose  URDF2Gazebo::inverseTransformToParentFrame(
  gazebo::math::Pose transform_in_link_frame,
  urdf::Pose parent_to_link_transform)
{
  gazebo::math::Pose transform_in_parent_link_frame;
  //   rotate link pose to parent_link frame
  urdf::Rotation ri = parent_to_link_transform.rotation.GetInverse();
  gazebo::math::Quaternion q1(ri.w, ri.x, ri.y, ri.z);
  transform_in_parent_link_frame.pos = q1 * transform_in_link_frame.pos;
  urdf::Rotation r2 = parent_to_link_transform.rotation.GetInverse();
  gazebo::math::Quaternion q3(r2.w, r2.x, r2.y, r2.z);
  transform_in_parent_link_frame.rot = q3 * transform_in_link_frame.rot;
  //   translate link to parent_link frame
  transform_in_parent_link_frame.pos.x = transform_in_parent_link_frame.pos.x
    - parent_to_link_transform.position.x;
  transform_in_parent_link_frame.pos.y = transform_in_parent_link_frame.pos.y
    - parent_to_link_transform.position.y;
  transform_in_parent_link_frame.pos.z = transform_in_parent_link_frame.pos.z
    - parent_to_link_transform.position.z;

  return transform_in_parent_link_frame;
}

/// Take the link's existing list of gazebo extensions, transfer them
/// into parent link.  Along the way, update local transforms by adding
/// the additional transform to parent.  Also, look through all
/// referenced link names with plugins and update references to current
/// link to the parent link. (reduceGazeboExtensionFrameReplace())
void URDF2Gazebo::reduceGazeboExtensionToParent(
  boost::shared_ptr<urdf::Link> link)
{
  /// @todo: this is a very complicated module that updates the plugins
  /// based on fixed joint reduction really wish this could be a lot cleaner

  std::string link_name = link->name;

  // update extension map with references to link_name
  // this->listGazeboExtensions();
  std::map<std::string, std::vector<GazeboExtension*> >::iterator ext =
    this->gazebo_extensions_.find(link_name);
  if (ext != this->gazebo_extensions_.end())
  {
    // gzdbg << "  REDUCE EXTENSION: moving reference from ["
    //       << link_name << "] to [" << link->getParent()->name << "]\n";

    // update reduction transform (for rays, cameras for now).
    //   FIXME: contact frames too?
    for (std::vector<GazeboExtension*>::iterator ge = ext->second.begin();
         ge != ext->second.end(); ++ge)
    {
      (*ge)->reduction_transform = transformToParentFrame(
        (*ge)->reduction_transform,
        link->parent_joint->parent_to_joint_origin_transform);
      // for sensor and projector blocks only
      reduceGazeboExtensionsTransformReduction((*ge));
    }

    // find pointer to the existing extension with the new link reference
    std::string new_link_name = link->getParent()->name;
    std::map<std::string, std::vector<GazeboExtension*> >::iterator
      new_ext = this->gazebo_extensions_.find(new_link_name);

    // if none exist, create new_extension with new_link_name
    if (new_ext == this->gazebo_extensions_.end())
    {
      std::vector<GazeboExtension*> extensions;
      this->gazebo_extensions_.insert(std::make_pair(
        new_link_name, extensions));
      new_ext = this->gazebo_extensions_.find(new_link_name);
    }

    // move gazebo extensions from link into the parent link's extensions
    for (std::vector<GazeboExtension*>::iterator ge = ext->second.begin();
         ge != ext->second.end(); ++ge)
      new_ext->second.push_back(*ge);
    ext->second.clear();
  }

  // for extensions with empty reference, search and replace
  // link name patterns within the plugin with new link name
  // and assign the proper reduction transform for the link name pattern
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazebo_it = this->gazebo_extensions_.begin();
       gazebo_it != this->gazebo_extensions_.end(); ++gazebo_it)
    {
      // update reduction transform (for contacts, rays, cameras for now).
      for (std::vector<GazeboExtension*>::iterator
        ge = gazebo_it->second.begin(); ge != gazebo_it->second.end(); ++ge)
        reduceGazeboExtensionFrameReplace(*ge, link);
    }

  // this->listGazeboExtensions();
}

void URDF2Gazebo::reduceGazeboExtensionFrameReplace(GazeboExtension* ge,
  boost::shared_ptr<urdf::Link> link)
{
  // std::string link_name = link->name;
  // std::string new_link_name = link->getParent()->name;
  std::vector<TiXmlElement*> blobs = ge->blobs;
  gazebo::math::Pose reduction_transform = ge->reduction_transform;

  // HACK: need to do this more generally, but we also need to replace
  //       all instances of link name with new link name
  //       e.g. contact sensor refers to
  //         <collision>base_link_collision</collision>
  //         and it needs to be reparented to
  //         <collision>base_footprint_collision</collision>
  // gzdbg << "  STRING REPLACE: instances of link name ["
  //       << link_name << "] with [" << new_link_name << "]\n";
  for (std::vector<TiXmlElement*>::iterator blob_it = blobs.begin();
       blob_it != blobs.end(); ++blob_it)
  {
    std::ostringstream debug_stream_in;
    debug_stream_in << *(*blob_it);
    // std::string debug_blob = debug_stream_in.str();
    // gzdbg << "        INITIAL STRING link ["
    //       << link_name << "]-->[" << new_link_name << "]: ["
    //       << debug_blob << "]\n";

    this->reduceGazeboExtensionContactSensorFrameReplace(blob_it, link);
    this->reduceGazeboExtensionPluginFrameReplace(blob_it, link,
      "plugin", "bodyName", reduction_transform);
    this->reduceGazeboExtensionPluginFrameReplace(blob_it, link,
      "plugin", "frameName", reduction_transform);
    this->reduceGazeboExtensionProjectorFrameReplace(blob_it, link);
    this->reduceGazeboExtensionGripperFrameReplace(blob_it, link);
    this->reduceGazeboExtensionJointFrameReplace(blob_it, link);

    std::ostringstream debug_stream_out;
    debug_stream_out << *(*blob_it);
  }
}

void URDF2Gazebo::reduceGazeboExtensionsTransformReduction(GazeboExtension* ge)
{
  gazebo::math::Pose reduction_transform = ge->reduction_transform;
  for (std::vector<TiXmlElement*>::iterator blob_it = ge->blobs.begin();
       blob_it != ge->blobs.end(); ++blob_it)
  {
    /// @todo make sure we are not missing any additional transform reductions
    this->reduceGazeboExtensionSensorTransformReduction(blob_it,
      reduction_transform);
    this->reduceGazeboExtensionProjectorTransformReduction(blob_it,
      reduction_transform);
  }
}


void URDF2Gazebo::listGazeboExtensions()
{
  gzdbg << "================================================================\n";
  for (std::map<std::string, std::vector<GazeboExtension*> >::iterator
       gazebo_it = this->gazebo_extensions_.begin();
       gazebo_it != this->gazebo_extensions_.end(); ++gazebo_it)
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
          blob_it = (*ge)->blobs.begin();
          blob_it != (*ge)->blobs.end(); ++blob_it)
        {
          std::ostringstream stream_in;
          stream_in << *(*blob_it);
          gzdbg << "    BLOB: [" << stream_in.str() << "]\n";
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
       gazebo_it = this->gazebo_extensions_.begin();
       gazebo_it != this->gazebo_extensions_.end(); ++gazebo_it)
  {
    if (gazebo_it->first == reference)
    {
        gzdbg <<  "  PRINTING [" << static_cast<int>(gazebo_it->second.size())
              << "] extensions referencing [" << reference << "]\n";
      for (std::vector<GazeboExtension*>::iterator
           ge = gazebo_it->second.begin(); ge != gazebo_it->second.end(); ++ge)
      {
        for (std::vector<TiXmlElement*>::iterator
          blob_it = (*ge)->blobs.begin();
          blob_it != (*ge)->blobs.end(); ++blob_it)
        {
          std::ostringstream stream_in;
          stream_in << *(*blob_it);
          gzdbg << "    BLOB: [" << stream_in.str() << "]\n";
        }
      }
    }
  }
  gzdbg << "================================================================\n";
}

void URDF2Gazebo::createSDF(TiXmlElement *root,
  boost::shared_ptr<const urdf::Link> link,
  const gazebo::math::Pose &transform)
{
    gazebo::math::Pose currentTransform = transform;

    // must have an <inertial> block and cannot have zero mass.
    //  allow det(I) == zero, in the case of point mass geoms.
    // @todo:  keyword "world" should be a constant defined somewhere else
    if (link->name != "world" &&
      ((!link->inertial) || gazebo::math::equal(link->inertial->mass, 0.0)))
    {
      if (!link->child_links.empty())
        gzwarn << "urdf2gazebo: link[" << link->name
               << "] has no inertia, ["
               << static_cast<int>(link->child_links.size())
               << "] children links ignored\n.";

      if (!link->child_joints.empty())
        gzwarn << "urdf2gazebo: link[" << link->name
               << "] has no inertia, ["
               << static_cast<int>(link->child_links.size())
               << "] children joints ignored\n.";

      if (link->parent_joint)
        gzwarn << "urdf2gazebo: link[" << link->name
               << "] has no inertia, parent joint [" << link->parent_joint->name
               << "] ignored\n.";

        gzwarn << "urdf2gazebo: link[" << link->name
               << "] has no inertia, not modeled in gazebo\n";
      return;
    }

    /* create <body:...> block for non fixed joint attached bodies */
    if ((link->getParent() && link->getParent()->name == "world") ||
        !this->reduce_fixed_joints ||
        (!link->parent_joint || link->parent_joint->type != urdf::Joint::FIXED))
      createLink(root, link, currentTransform);

    // recurse into children
    for (unsigned int i = 0 ; i < link->child_links.size() ; ++i)
        createSDF(root, link->child_links[i], currentTransform);
}

gazebo::math::Pose  URDF2Gazebo::copyPose(urdf::Pose pose)
{
  gazebo::math::Pose p;
  p.pos.x = pose.position.x;
  p.pos.y = pose.position.y;
  p.pos.z = pose.position.z;
  p.rot.x = pose.rotation.x;
  p.rot.y = pose.rotation.y;
  p.rot.z = pose.rotation.z;
  p.rot.w = pose.rotation.w;
  return p;
}
urdf::Pose  URDF2Gazebo::copyPose(gazebo::math::Pose pose)
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
  boost::shared_ptr<const urdf::Link> link,
  gazebo::math::Pose &currentTransform)
{
  /* create new body */
  TiXmlElement *elem     = new TiXmlElement("link");

  /* set body name */
  elem->SetAttribute("name", link->name);

  /* compute global transform */
  gazebo::math::Pose localTransform;
  // this is the transform from parent link to current link
  // this transform does not exist for the root link
  if (link->parent_joint)
  {
    localTransform = URDF2Gazebo::copyPose(
      link->parent_joint->parent_to_joint_origin_transform);
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

  /* copy gazebo extensions data */
  insertGazeboExtensionLink(elem, link->name);

  /* add body to document */
  root->LinkEndChild(elem);

  /* make a <joint:...> block */
  createJoint(root, link, currentTransform);
}

void URDF2Gazebo::createCollisions(TiXmlElement* elem,
  boost::shared_ptr<const urdf::Link> link)
{
  // loop through all collisions. make additional geoms using the lumped stuff
  for (std::map<std::string,
    boost::shared_ptr<std::vector<CollisionPtr> > >::const_iterator
    collisions_it = link->collision_groups.begin();
    collisions_it != link->collision_groups.end(); ++collisions_it)
  {
    boost::shared_ptr<urdf::Collision> collision =
      *(collisions_it->second->begin());

    if (collisions_it->first == "default")
    {
      // gzdbg << "creating default collision for link [" << link->name
      //       << "]";

      /* make a <collision> block */
      createCollision(elem, link, collision, link->name);
    }
    else if (collisions_it->first.find(std::string("lump::")) == 0)
    {
      // if collision name starts with "lump::", pass through
      //   original parent link name
      // gzdbg << "creating lump collision [" << collisions_it->first
      //       << "] for link [" << link->name << "].\n";
      /// old_link_name is the original name before lumping
      std::string old_link_name = collisions_it->first.substr(6);
      createCollision(elem, link, collision, old_link_name);
    }
  }
}

void URDF2Gazebo::createVisuals(TiXmlElement* elem,
  boost::shared_ptr<const urdf::Link> link)
{
  // loop through all visuals. make additional collisions using the
  //   lumped stuff
  for (std::map<std::string,
    boost::shared_ptr<std::vector<VisualPtr> > >::const_iterator
    visuals_it = link->visual_groups.begin();
    visuals_it != link->visual_groups.end(); ++visuals_it)
  {
    boost::shared_ptr<urdf::Visual> visual = *(visuals_it->second->begin());

    if (visuals_it->first == "default")
    {
      /* make a visual block */
      createVisual(elem, link, visual, link->name);
    }
    else if (visuals_it->first.find(std::string("lump::")) == 0)
    {
      std::string old_link_name = visuals_it->first.substr(6);
      /* make a visual block */
      createVisual(elem, link, visual, old_link_name);
    }
  }
}


void URDF2Gazebo::createInertial(TiXmlElement *elem,
  boost::shared_ptr<const urdf::Link> link)
{
  TiXmlElement *inertial = new TiXmlElement("inertial");

  /* set mass properties */
  // check and print a warning message
  double roll, pitch, yaw;
  link->inertial->origin.rotation.getRPY(roll, pitch, yaw);
  if (!gazebo::math::equal(roll, 0.0) ||
    !gazebo::math::equal(pitch, 0.0) || !gazebo::math::equal(yaw, 0.0))
      gzerr << "rotation of inertial frame in link ["
            << link->name << "] is not supported\n";

  /// add pose
  gazebo::math::Pose pose = URDF2Gazebo::copyPose(link->inertial->origin);
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
  boost::shared_ptr<const urdf::Link> link,
  gazebo::math::Pose &currentTransform)
{
    /* compute the joint tag */
    std::string jtype;
    jtype.clear();
    if (link->parent_joint != NULL)
    {
      switch (link->parent_joint->type)
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
            gzwarn << "Unknown joint type: [" << link->parent_joint->type
                   << "] in link [" << link->name << "]\n";
            break;
      }
    }

    // skip if joint type is fixed and we are not faking it with a hinge,
    //   skip/return with the exception of root link being world,
    //   because there's no lumping there
    if (link->getParent() && link->getParent()->name != "world"
        && jtype == "fixed" && this->reduce_fixed_joints) return;

    if (!jtype.empty())
    {
        TiXmlElement *joint = new TiXmlElement("joint");
        if (jtype == "fixed")
          joint->SetAttribute("type", "revolute");
        else
          joint->SetAttribute("type", jtype);
        joint->SetAttribute("name", link->parent_joint->name);
        addKeyValue(joint, "child", link->name);
        addKeyValue(joint, "parent", link->getParent()->name);

        TiXmlElement *joint_axis = new TiXmlElement("axis");
        TiXmlElement *joint_axis_limit = new TiXmlElement("limit");
        TiXmlElement *joint_axis_dynamics = new TiXmlElement("dynamics");
        if (jtype == "fixed")
        {
          addKeyValue(joint_axis_limit, "lower", "0");
          addKeyValue(joint_axis_limit, "upper", "0");
          addKeyValue(joint_axis_dynamics, "damping", "0");
        }
        else
        {
          gazebo::math::Vector3 rotatedJointAxis =
            currentTransform.rot.RotateVector(
            gazebo::math::Vector3(link->parent_joint->axis.x,
                                  link->parent_joint->axis.y,
                                  link->parent_joint->axis.z));
          double rotatedJointAxisArray[3] =
            { rotatedJointAxis.x, rotatedJointAxis.y, rotatedJointAxis.z };
          addKeyValue(joint_axis, "xyz", values2str(3, rotatedJointAxisArray));
          if (link->parent_joint->dynamics)
            addKeyValue(joint_axis_dynamics, "damping",
              values2str(1, &link->parent_joint->dynamics->damping));

          if (this->enforce_limits && link->parent_joint->limits)
          {
            if (jtype == "slider")
            {
              addKeyValue(joint_axis_limit, "lower",
                values2str(1, &link->parent_joint->limits->lower));
              addKeyValue(joint_axis_limit, "upper",
                values2str(1, &link->parent_joint->limits->upper));
            }
            else if (link->parent_joint->type != urdf::Joint::CONTINUOUS)
            {
              double *lowstop  = &link->parent_joint->limits->lower;
              double *highstop = &link->parent_joint->limits->upper;
              // enforce ode bounds, this will need to be fixed
              if (*lowstop > *highstop)
              {
                gzwarn << "urdf2gazebo: revolute joint ["
                       << link->parent_joint->name
                       << "] with limits: lowStop[" << *lowstop
                       << "] > highStop[" << highstop
                       << "], switching the two.\n";
                double tmp = *lowstop;
                *lowstop = *highstop;
                *highstop = tmp;
              }
              addKeyValue(joint_axis_limit, "lower",
                values2str(1, &link->parent_joint->limits->lower));
              addKeyValue(joint_axis_limit, "upper",
                values2str(1, &link->parent_joint->limits->upper));
            }
          }
        }
        joint_axis->LinkEndChild(joint_axis_limit);
        joint_axis->LinkEndChild(joint_axis_dynamics);
        joint->LinkEndChild(joint_axis);

        /* copy gazebo extensions data */
        insertGazeboExtensionJoint(joint, link->parent_joint->name);

        /* add joint to document */
        root->LinkEndChild(joint);
    }
}


void URDF2Gazebo::createCollision(TiXmlElement* elem,
  boost::shared_ptr<const urdf::Link> link,
  boost::shared_ptr<urdf::Collision> collision,
  std::string old_link_name)
{
    /* begin create geometry node, skip if no collision specified */
    TiXmlElement *gazebo_collision = new TiXmlElement("collision");

    /* set its name, if lumped, add original link name */
    if (old_link_name == link->name)
      gazebo_collision->SetAttribute("name",
        link->name + std::string("_collision"));
    else
      gazebo_collision->SetAttribute("name",
        link->name + std::string("_collision_")+old_link_name);

    /* set transform */
    double pose[6];
    pose[0] = collision->origin.position.x;
    pose[1] = collision->origin.position.y;
    pose[2] = collision->origin.position.z;
    collision->origin.rotation.getRPY(pose[3], pose[4], pose[5]);
    addKeyValue(gazebo_collision, "pose", values2str(6, pose));


    /* add geometry block */
    if (!collision || !collision->geometry)
    {
      // gzdbg << "urdf2gazebo: collision of link [" << link->name
      //       << "] has no <geometry>.\n";
    }
    else
    {
      createGeometry(gazebo_collision, collision->geometry);
    }

    /* set additional data from extensions */
    insertGazeboExtensionCollision(gazebo_collision, old_link_name);

    /* add geometry to body */
    elem->LinkEndChild(gazebo_collision);
}

void URDF2Gazebo::createVisual(TiXmlElement *elem,
  boost::shared_ptr<const urdf::Link> link,
  boost::shared_ptr<urdf::Visual> visual, std::string old_link_name)
{
    /* begin create gazebo visual node */
    TiXmlElement *gazebo_visual = new TiXmlElement("visual");

    /* set its name */
    // gzdbg << "original link name [" << old_link_name
    //       << "] new link name [" << link->name << "]\n";
    if (old_link_name == link->name)
      gazebo_visual->SetAttribute("name", link->name + std::string("_vis"));
    else
      gazebo_visual->SetAttribute("name", link->name + std::string("_vis_")
        + old_link_name);

    /* add the visualisation transfrom */
    double pose[6];
    pose[0] = visual->origin.position.x;
    pose[1] = visual->origin.position.y;
    pose[2] = visual->origin.position.z;
    visual->origin.rotation.getRPY(pose[3], pose[4], pose[5]);
    addKeyValue(gazebo_visual, "pose", values2str(6, pose));

    /* insert geometry */
    if (!visual || !visual->geometry)
    {
      // gzdbg << "urdf2gazebo: visual of link [" << link->name
      //       << "] has no <geometry>\n.";
    }
    else
      createGeometry(gazebo_visual, visual->geometry);

    /* set additional data from extensions */
    insertGazeboExtensionVisual(gazebo_visual, old_link_name);

    /* end create visual node */
    elem->LinkEndChild(gazebo_visual);
}

TiXmlDocument URDF2Gazebo::initModelString(std::string urdf_str,
  bool _enforce_limits)
{
    this->enforce_limits = _enforce_limits;
    return this->initModelString(urdf_str);
}

TiXmlDocument URDF2Gazebo::initModelString(std::string urdf_str)
{
    /* Create a RobotModel from string */
    boost::shared_ptr<urdf::ModelInterface> robot_model =
      urdf::parseURDF(urdf_str.c_str());

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
    TiXmlDocument urdf_xml;
    urdf_xml.Parse(urdf_str.c_str());
    parseGazeboExtension(urdf_xml);

    boost::shared_ptr<const urdf::Link> root_link = robot_model->getRoot();

    /* Fixed Joint Reduction */
    /* if link connects to parent via fixed joint, lump down and remove link */
    /* set reduce_fixed_joints to false will replace fixed joints with
       zero limit revolute joints, otherwise, we reduce it down to its
       parent link recursively */
    if (this->reduce_fixed_joints)
      reduceFixedJoints(robot,
        (boost::const_pointer_cast< urdf::Link >(root_link)));

    if (root_link->name == "world")
    {
      /* convert all children link */
      for (std::vector<boost::shared_ptr<urdf::Link> >::const_iterator
        child = root_link->child_links.begin();
        child != root_link->child_links.end(); ++child)
          createSDF(robot, (*child), transform);
    }
    else
    {
      /* convert, starting from root link */
      createSDF(robot, root_link, transform);
    }

    /* insert the extensions without reference into <robot> root level */
    insertGazeboExtensionRobot(robot);

    // add robot to gazebo_xml_out
    TiXmlElement *gazebo_sdf = new TiXmlElement("sdf");
    gazebo_sdf->SetAttribute("version", SDF_VERSION);
    gazebo_sdf->LinkEndChild(robot);
    gazebo_xml_out.LinkEndChild(gazebo_sdf);

    // debug
    // gazebo_xml_out.Print();

    return gazebo_xml_out;
}

TiXmlDocument URDF2Gazebo::initModelDoc(TiXmlDocument* _xmlDoc)
{
    std::ostringstream stream;
    stream << *_xmlDoc;
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
    gzerr << "Unable to load file[" << filename << "].\n";

    return xmlDoc;
}

void URDF2Gazebo::reduceInertialToParent(boost::shared_ptr<urdf::Link> link)
{
    // gzdbg << "TREE:   mass lumping from [" << link->name
    //      << "] to [" << link->getParent()->name << "]\n.";
    /* now lump all contents of this link to parent */
    if (link->inertial)
    {
      // get parent mass (in parent link frame)
      dMass parent_mass;
      if (!link->getParent()->inertial)
        link->getParent()->inertial.reset(new urdf::Inertial);
      dMassSetParameters(&parent_mass, link->getParent()->inertial->mass,
        link->getParent()->inertial->origin.position.x,
        link->getParent()->inertial->origin.position.y,
        link->getParent()->inertial->origin.position.z,
        link->getParent()->inertial->ixx, link->getParent()->inertial->iyy,
        link->getParent()->inertial->izz, link->getParent()->inertial->ixy,
         link->getParent()->inertial->ixz, link->getParent()->inertial->iyz);
      // printMass(link->getParent()->name, parent_mass);
      // printMass(link->getParent());
      // set link mass (in link frame)
      dMass link_mass;
      dMassSetParameters(&link_mass, link->inertial->mass,
        link->inertial->origin.position.x,
        link->inertial->origin.position.y,
        link->inertial->origin.position.z,
        link->inertial->ixx, link->inertial->iyy, link->inertial->izz,
        link->inertial->ixy, link->inertial->ixz, link->inertial->iyz);
      // printMass(link->name, link_mass);
      // printMass(link);
      // un-rotate link mass into parent link frame
      dMatrix3 R;
      double phi, theta, psi;
      link->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(
        phi, theta, psi);
      dRFromEulerAngles(R, phi, theta, psi);
      dMassRotate(&link_mass, R);
      // printMass(link->name, link_mass);
      // un-translate link mass into parent link frame
      dMassTranslate(&link_mass,
        link->parent_joint->parent_to_joint_origin_transform.position.x,
        link->parent_joint->parent_to_joint_origin_transform.position.y,
        link->parent_joint->parent_to_joint_origin_transform.position.z);
      // printMass(link->name, link_mass);
      // now link_mass is in the parent frame, add link_mass to parent_mass
      dMassAdd(&parent_mass, &link_mass);
      // printMass(link->getParent()->name, parent_mass);
      // update parent mass
      link->getParent()->inertial->mass = parent_mass.mass;
      link->getParent()->inertial->ixx  = parent_mass.I[0+4*0];
      link->getParent()->inertial->iyy  = parent_mass.I[1+4*1];
      link->getParent()->inertial->izz  = parent_mass.I[2+4*2];
      link->getParent()->inertial->ixy  = parent_mass.I[0+4*1];
      link->getParent()->inertial->ixz  = parent_mass.I[0+4*2];
      link->getParent()->inertial->iyz  = parent_mass.I[1+4*2];
      link->getParent()->inertial->origin.position.x  = parent_mass.c[0];
      link->getParent()->inertial->origin.position.y  = parent_mass.c[1];
      link->getParent()->inertial->origin.position.z  = parent_mass.c[2];
      // printMass(link->getParent());
    }
}

void URDF2Gazebo::reduceVisualsToParent(boost::shared_ptr<urdf::Link> link)
{
  // lump visual to parent
  // lump all visual to parent, assign group name
  // "lump::"+group name+"::'+link name
  // lump but keep the link name in(/as) the group name,
  // so we can correlate visuals to visuals somehow.
  for (std::map<std::string,
    boost::shared_ptr<std::vector<VisualPtr> > >::iterator
    visuals_it = link->visual_groups.begin();
    visuals_it != link->visual_groups.end(); ++visuals_it)
  {
    /// @todo: extend to different groups,
    /// only work with default meshes right now.
    if (visuals_it->first == "default")
    {
      std::string lump_group_name = std::string("lump::")+link->name;
      // gzdbg << "adding modified lump group name [" << lump_group_name
      //       << "] to link [" << link->getParent()->name << "]\n.";
      for (std::vector<boost::shared_ptr<urdf::Visual> >::iterator
        visual_it = visuals_it->second->begin();
        visual_it != visuals_it->second->end(); ++visual_it)
      {
        // transform visual origin from link frame to
        // parent link frame before adding to parent
        (*visual_it)->origin = transformToParentFrame((*visual_it)->origin,
          link->parent_joint->parent_to_joint_origin_transform);
        // add the modified visual to parent
        this->reduceVisualToParent(link->getParent(), lump_group_name,
          *visual_it);
      }
    }
    else if (visuals_it->first.find(std::string("lump::")) == 0)
    {
      // it's a previously lumped mesh, re-lump under same group_name
      std::string lump_group_name = visuals_it->first;
      // gzdbg << "re-lumping group name [" << lump_group_name
      //       << "] to link [" << link->getParent()->name << "]\n";
      for (std::vector<boost::shared_ptr<urdf::Visual> >::iterator
           visual_it = visuals_it->second->begin();
           visual_it != visuals_it->second->end(); ++visual_it)
      {
        // transform visual origin from link frame to parent link
        // frame before adding to parent
        (*visual_it)->origin = transformToParentFrame((*visual_it)->origin,
          link->parent_joint->parent_to_joint_origin_transform);
        // add the modified visual to parent
        this->reduceVisualToParent(link->getParent(), lump_group_name,
          *visual_it);
      }
    }
  }
}

void URDF2Gazebo::reduceCollisionsToParent(boost::shared_ptr<urdf::Link> link)
{
    // lump collision parent
    // lump all collision to parent, assign group name
    // "lump::"+group name+"::'+link name
    // lump but keep the link name in(/as) the group name,
    // so we can correlate visuals to collisions somehow.
    for (std::map<std::string,
      boost::shared_ptr<std::vector<CollisionPtr> > >::iterator
      collisions_it = link->collision_groups.begin();
      collisions_it != link->collision_groups.end(); ++collisions_it)
    {
      if (collisions_it->first == "default")
      {
        // if it's a "default" mesh, it will be added under "lump::"+link name
        std::string lump_group_name = std::string("lump::")+link->name;
        // gzdbg << "lumping collision [" << collisions_it->first
        //       << "] for link [" << link->name
        //       << "] to parent [" << link->getParent()->name
        //       << "] with group name [" << lump_group_name << "]\n";
        for (std::vector<boost::shared_ptr<urdf::Collision> >::iterator
          collision_it = collisions_it->second->begin();
          collision_it != collisions_it->second->end(); ++collision_it)
        {
          // transform collision origin from link frame to
          // parent link frame before adding to parent
          (*collision_it)->origin = transformToParentFrame(
            (*collision_it)->origin,
            link->parent_joint->parent_to_joint_origin_transform);

          // add the modified collision to parent
          this->reduceCollisionToParent(link->getParent(), lump_group_name,
            *collision_it);
        }
      }
      else if (collisions_it->first.find(std::string("lump::")) == 0)
      {
        // if it's a previously lumped mesh, relump under same group_name
        std::string lump_group_name = collisions_it->first;
        // gzdbg << "re-lumping collision [" << collisions_it->first
        //       << "] for link [" << link->name
        //       << "] to parent [" << link->getParent()->name
        //       << "] with group name [" << lump_group_name << "]\n";
        for (std::vector<boost::shared_ptr<urdf::Collision> >::iterator
          collision_it = collisions_it->second->begin();
          collision_it != collisions_it->second->end(); ++collision_it)
        {
          // transform collision origin from link frame to
          // parent link frame before adding to parent
          (*collision_it)->origin = transformToParentFrame(
            (*collision_it)->origin,
            link->parent_joint->parent_to_joint_origin_transform);
          // add the modified collision to parent
          this->reduceCollisionToParent(link->getParent(), lump_group_name,
            *collision_it);
        }
      }
    }
    // printCollisionGroups(link->getParent());
}

void URDF2Gazebo::reduceJointsToParent(boost::shared_ptr<urdf::Link> link)
{
    // set child link's parent_joint's parent link to
    // a parent link up stream that does not have a fixed parent_joint
    for (unsigned int i = 0 ; i < link->child_links.size() ; ++i)
    {
      boost::shared_ptr<urdf::Joint> parent_joint =
        link->child_links[i]->parent_joint;
      if (parent_joint->type != urdf::Joint::FIXED)
      {
        // go down the tree until we hit a parent joint that is not fixed
        boost::shared_ptr<urdf::Link> new_parent_link = link;
        gazebo::math::Pose joint_anchor_transform;
        while (new_parent_link->parent_joint &&
              new_parent_link->getParent()->name != "world" &&
              new_parent_link->parent_joint->type == urdf::Joint::FIXED)
        {
          joint_anchor_transform = joint_anchor_transform *
            joint_anchor_transform;
          parent_joint->parent_to_joint_origin_transform =
            transformToParentFrame(
            parent_joint->parent_to_joint_origin_transform,
            new_parent_link->parent_joint->parent_to_joint_origin_transform);
          new_parent_link = new_parent_link->getParent();
        }
        // now set the link->child_links[i]->parent_joint's parent link to
        // the new_parent_link
        link->child_links[i]->setParent(new_parent_link);
        parent_joint->parent_link_name = new_parent_link->name;
        // and set the link->child_links[i]->parent_joint's
        // parent_to_joint_orogin_transform as the aggregated anchor transform?
      }
    }
}

void URDF2Gazebo::reduceGazeboExtensionSensorTransformReduction(
  std::vector<TiXmlElement*>::iterator blob_it,
  gazebo::math::Pose reduction_transform)
{
    // overwrite <xyz> and <rpy> if they exist
    if ((*blob_it)->ValueStr() == "sensor")
    {
      /*
      // parse it and add/replace the reduction transform
      // find first instance of xyz and rpy, replace with reduction transform
      for (TiXmlNode* el_it = (*blob_it)->FirstChild();
           el_it; el_it = el_it->NextSibling())
      {
        std::ostringstream stream_in;
        stream_in << *el_it;
        gzdbg << "    " << stream_in << "\n";
      }
      */

      {
        TiXmlNode* old_pose_key = (*blob_it)->FirstChild("pose");
        /// @todo: FIXME:  we should read xyz, rpy and aggregate it to
        /// reduction_transform instead of just throwing the info away.
        if (old_pose_key)
          (*blob_it)->RemoveChild(old_pose_key);
      }

      // convert reduction_transform to values
      urdf::Vector3 reduction_xyz(reduction_transform.pos.x,
        reduction_transform.pos.y,
        reduction_transform.pos.z);
      urdf::Rotation reduction_q(reduction_transform.rot.x,
        reduction_transform.rot.y,
        reduction_transform.rot.z, reduction_transform.rot.w);

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

      (*blob_it)->LinkEndChild(pose_key);
    }
}

void URDF2Gazebo::reduceGazeboExtensionProjectorTransformReduction(
  std::vector<TiXmlElement*>::iterator blob_it,
  gazebo::math::Pose reduction_transform)
{
    // overwrite <pose> (xyz/rpy) if it exists
    if ((*blob_it)->ValueStr() == "projector")
    {
      /*
      // parse it and add/replace the reduction transform
      // find first instance of xyz and rpy, replace with reduction transform
      for (TiXmlNode* el_it = (*blob_it)->FirstChild();
        el_it; el_it = el_it->NextSibling())
      {
        std::ostringstream stream_in;
        stream_in << *el_it;
        gzdbg << "    " << stream_in << "\n";
      }
      */

      /* should read <pose>...</pose> and agregate reduction_transform */
      TiXmlNode* pose_key = (*blob_it)->FirstChild("pose");
      // read pose and save it

      // remove the tag for now
      if (pose_key) (*blob_it)->RemoveChild(pose_key);

      // convert reduction_transform to values
      urdf::Vector3 reduction_xyz(reduction_transform.pos.x,
        reduction_transform.pos.y,
        reduction_transform.pos.z);
      urdf::Rotation reduction_q(reduction_transform.rot.x,
        reduction_transform.rot.y,
        reduction_transform.rot.z,
        reduction_transform.rot.w);

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

      (*blob_it)->LinkEndChild(pose_key);
    }
}

void URDF2Gazebo::reduceGazeboExtensionContactSensorFrameReplace(
  std::vector<TiXmlElement*>::iterator blob_it,
  boost::shared_ptr<urdf::Link> link)
{
  std::string link_name = link->name;
  std::string new_link_name = link->getParent()->name;
  if ((*blob_it)->ValueStr() == "sensor")
  {
    // parse it and add/replace the reduction transform
    // find first instance of xyz and rpy, replace with reduction transform
    TiXmlNode* contact = (*blob_it)->FirstChild("contact");
    if (contact)
    {
      TiXmlNode* collision = contact->FirstChild("collision");
      if (collision)
      {
        if (getKeyValueAsString(collision->ToElement()) ==
          link_name + std::string("_collision"))
        {
          contact->RemoveChild(collision);
          TiXmlElement* collision_name_key = new TiXmlElement("collision");
          std::ostringstream collision_name_stream;
          collision_name_stream << new_link_name << "_collision_" << link_name;
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

void URDF2Gazebo::reduceGazeboExtensionPluginFrameReplace(
  std::vector<TiXmlElement*>::iterator blob_it,
  boost::shared_ptr<urdf::Link> link,
  std::string plugin_name, std::string element_name,
  gazebo::math::Pose reduction_transform)
{
  std::string link_name = link->name;
  std::string new_link_name = link->getParent()->name;
  if ((*blob_it)->ValueStr() == plugin_name)
  {
    // replace element containing link names to parent link names
    // find first instance of xyz and rpy, replace with reduction transform
    TiXmlNode* element_node = (*blob_it)->FirstChild(element_name);
    if (element_node)
    {
      if (getKeyValueAsString(element_node->ToElement()) == link_name)
      {
        (*blob_it)->RemoveChild(element_node);
        TiXmlElement* body_name_key = new TiXmlElement(element_name);
        std::ostringstream body_name_stream;
        body_name_stream << new_link_name;
        TiXmlText* body_name_txt = new TiXmlText(body_name_stream.str());
        body_name_key->LinkEndChild(body_name_txt);
        (*blob_it)->LinkEndChild(body_name_key);
        /// @todo update transforms for this gazebo plugin too

        // look for offset transforms, add reduction transform
        TiXmlNode* xyz_key = (*blob_it)->FirstChild("xyzOffset");
        if (xyz_key)
        {
          urdf::Vector3 v1 = parseVector3(xyz_key);
          reduction_transform.pos = gazebo::math::Vector3(v1.x, v1.y, v1.z);
          // remove xyzOffset and rpyOffset
          (*blob_it)->RemoveChild(xyz_key);
        }
        TiXmlNode* rpy_key = (*blob_it)->FirstChild("rpyOffset");
        if (rpy_key)
        {
          urdf::Vector3 rpy = parseVector3(rpy_key, M_PI/180.0);
          reduction_transform.rot =
            gazebo::math::Quaternion::EulerToQuaternion(rpy.x, rpy.y, rpy.z);
          // remove xyzOffset and rpyOffset
          (*blob_it)->RemoveChild(rpy_key);
        }

        // pass through the parent transform from fixed joint reduction
        reduction_transform = inverseTransformToParentFrame(reduction_transform,
          link->parent_joint->parent_to_joint_origin_transform);

        // create new offset xml blocks
        xyz_key = new TiXmlElement("xyzOffset");
        rpy_key = new TiXmlElement("rpyOffset");

        // create new offset xml blocks
        urdf::Vector3 reduction_xyz(reduction_transform.pos.x,
          reduction_transform.pos.y,
          reduction_transform.pos.z);
        urdf::Rotation reduction_q(reduction_transform.rot.x,
          reduction_transform.rot.y, reduction_transform.rot.z,
          reduction_transform.rot.w);

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

        (*blob_it)->LinkEndChild(xyz_key);
        (*blob_it)->LinkEndChild(rpy_key);
      }
    }
  }
}

void URDF2Gazebo::reduceGazeboExtensionProjectorFrameReplace(
  std::vector<TiXmlElement*>::iterator blob_it,
  boost::shared_ptr<urdf::Link> link)
{
  std::string link_name = link->name;
  std::string new_link_name = link->getParent()->name;

  // updates link reference for <projector> inside of
  // projector plugins
  // update from <projector>MyLinkName/MyProjectorName</projector>
  // to <projector>NewLinkName/MyProjectorName</projector>
  TiXmlNode* projector_elem = (*blob_it)->FirstChild("projector");
  {
    if (projector_elem)
    {
      std::string projector_name =  getKeyValueAsString(
        projector_elem->ToElement());
      // extract projector link name and projector name
      size_t pos = projector_name.find("/");
      if (pos == std::string::npos)
        gzerr << "no slash in projector reference tag [" << projector_name
              << "], expecting link_name/projector_name.\n";
      std::string projector_link_name = projector_name.substr(0, pos);

      if (projector_link_name == link_name)
      {
        // do the replacement
        projector_name = new_link_name + "/" +
          projector_name.substr(pos+1, projector_name.size());

        (*blob_it)->RemoveChild(projector_elem);
        TiXmlElement* body_name_key = new TiXmlElement("projector");
        std::ostringstream body_name_stream;
        body_name_stream << projector_name;
        TiXmlText* body_name_txt = new TiXmlText(body_name_stream.str());
        body_name_key->LinkEndChild(body_name_txt);
        (*blob_it)->LinkEndChild(body_name_key);
      }
    }
  }
}

void URDF2Gazebo::reduceGazeboExtensionGripperFrameReplace(
  std::vector<TiXmlElement*>::iterator blob_it,
  boost::shared_ptr<urdf::Link> link)
{
  std::string link_name = link->name;
  std::string new_link_name = link->getParent()->name;

  if ((*blob_it)->ValueStr() == "gripper")
  {
    TiXmlNode* gripper_link = (*blob_it)->FirstChild("gripper_link");
    if (gripper_link)
    {
      if (getKeyValueAsString(gripper_link->ToElement()) == link_name)
      {
        (*blob_it)->RemoveChild(gripper_link);
        TiXmlElement* body_name_key = new TiXmlElement("gripper_link");
        std::ostringstream body_name_stream;
        body_name_stream << new_link_name;
        TiXmlText* body_name_txt = new TiXmlText(body_name_stream.str());
        body_name_key->LinkEndChild(body_name_txt);
        (*blob_it)->LinkEndChild(body_name_key);
      }
    }
    TiXmlNode* palm_link = (*blob_it)->FirstChild("palm_link");
    if (palm_link)
    {
      if (getKeyValueAsString(palm_link->ToElement()) == link_name)
      {
        (*blob_it)->RemoveChild(palm_link);
        TiXmlElement* body_name_key = new TiXmlElement("palm_link");
        std::ostringstream body_name_stream;
        body_name_stream << new_link_name;
        TiXmlText* body_name_txt = new TiXmlText(body_name_stream.str());
        body_name_key->LinkEndChild(body_name_txt);
        (*blob_it)->LinkEndChild(body_name_key);
      }
    }
  }
}

void URDF2Gazebo::reduceGazeboExtensionJointFrameReplace(
  std::vector<TiXmlElement*>::iterator blob_it,
  boost::shared_ptr<urdf::Link> link)
{
  std::string link_name = link->name;
  std::string new_link_name = link->getParent()->name;

  if ((*blob_it)->ValueStr() == "joint")
  {
    // parse it and add/replace the reduction transform
    // find first instance of xyz and rpy, replace with reduction transform
    TiXmlNode* parent = (*blob_it)->FirstChild("parent");
    if (parent)
    {
      if (getKeyValueAsString(parent->ToElement()) == link_name)
      {
        (*blob_it)->RemoveChild(parent);
        TiXmlElement* parent_name_key = new TiXmlElement("parent");
        std::ostringstream parent_name_stream;
        parent_name_stream << new_link_name;
        TiXmlText* parent_name_txt = new TiXmlText(parent_name_stream.str());
        parent_name_key->LinkEndChild(parent_name_txt);
        (*blob_it)->LinkEndChild(parent_name_key);
      }
    }
    TiXmlNode* child = (*blob_it)->FirstChild("child");
    if (child)
    {
      if (getKeyValueAsString(child->ToElement()) == link_name)
      {
        (*blob_it)->RemoveChild(child);
        TiXmlElement* child_name_key = new TiXmlElement("child");
        std::ostringstream child_name_stream;
        child_name_stream << new_link_name;
        TiXmlText* child_name_txt = new TiXmlText(child_name_stream.str());
        child_name_key->LinkEndChild(child_name_txt);
        (*blob_it)->LinkEndChild(child_name_key);
      }
    }
    /// @todo add anchor offsets if parent link changes location!
  }
}
}
