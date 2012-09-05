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

#include <sdf/interface/parser_urdf.hh>
/*
#include "ros/ros.h"
#include "ros/package.h"
*/
#include "console_bridge/console.h"
#include <urdf_parser/urdf_parser.h>

#include <fstream>
#include <sstream>

#undef TO_SDF
#ifdef TO_SDF
#include "common/SystemPaths.hh"
#include "sdf/interface/parser.hh"
#include "sdf/interface/parser_deprecated.hh"
#endif

using namespace urdf2gazebo;

URDF2Gazebo::URDF2Gazebo()
{
#ifdef TO_SDF
  // set gazebo media paths by adding all packages that exports "gazebo_media_path" for gazebo
  std::list<std::string> paths = gazebo::common::SystemPaths::Instance()->GetGazeboPaths();
  std::vector<std::string> gazebo_media_paths;
  ros::package::getPlugins("gazebo","gazebo_media_path",gazebo_media_paths);
  for (std::vector<std::string>::iterator iter=gazebo_media_paths.begin(); iter != gazebo_media_paths.end(); iter++)
  {
    logDebug("med path %s",iter->c_str());


    // search for path, add if not there already
    bool found = false;
    for (std::list<std::string>::const_iterator p_iter = paths.begin();
        p_iter != paths.end(); ++p_iter)
    {
      if ((*p_iter) == (*iter))
      {
        found = true;
        break;
      }
    }
    if (!found)
      gazebo::common::SystemPaths::Instance()->AddGazeboPaths(iter->c_str());
  }
#endif
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
          vals.push_back(scale * boost::lexical_cast<double>(pieces[i].c_str()));
        }
        catch(boost::bad_lexical_cast &e)
        {
          logError("xml key [%s][%d] value [%s] is not a valid double from a 3-tuple\n",str.c_str(),i,pieces[i].c_str());
          return urdf::Vector3(0,0,0);
        }
      }
    }
    return urdf::Vector3(vals[0],vals[1],vals[3]);
  }
  else
    return urdf::Vector3(0,0,0);
}

void URDF2Gazebo::addVisual(boost::shared_ptr<urdf::Link> link, std::string group_name, boost::shared_ptr<urdf::Visual> visual)
{
  boost::shared_ptr<std::vector<boost::shared_ptr<urdf::Visual > > > viss = link->getVisuals(group_name);
  if (!viss)
  {
    // group does not exist, create one and add to map
    viss.reset(new std::vector<boost::shared_ptr<urdf::Visual > >);
    // new group name, create vector, add vector to map and add Visual to the vector
    link->visual_groups.insert(make_pair(group_name,viss));
    logDebug("successfully added a new visual group name '%s'",group_name.c_str());
  }

  // group exists, add Visual to the vector in the map
  std::vector<boost::shared_ptr<urdf::Visual > >::iterator vis_it = find(viss->begin(),viss->end(),visual);
  if (vis_it != viss->end())
    logWarn("attempted to add a visual that already exists under group name '%s', skipping.",group_name.c_str());
  else
    viss->push_back(visual);
  logDebug("successfully added a new visual under group name '%s'",group_name.c_str());

}

void URDF2Gazebo::addCollision(boost::shared_ptr<urdf::Link> link, std::string group_name, boost::shared_ptr<urdf::Collision> collision)
{
  boost::shared_ptr<std::vector<boost::shared_ptr<urdf::Collision > > > viss = link->getCollisions(group_name);
  if (!viss)
  {
    // group does not exist, create one and add to map
    viss.reset(new std::vector<boost::shared_ptr<urdf::Collision > >);
    // new group name, create vector, add vector to map and add Collision to the vector
    link->collision_groups.insert(make_pair(group_name,viss));
    logDebug("successfully added a new collision group name '%s'",group_name.c_str());
  }

  // group exists, add Collision to the vector in the map
  std::vector<boost::shared_ptr<urdf::Collision > >::iterator vis_it = find(viss->begin(),viss->end(),collision);
  if (vis_it != viss->end())
    logWarn("attempted to add a collision that already exists under group name '%s', skipping.",group_name.c_str());
  else
    viss->push_back(collision);
  logDebug("successfully added a new collision under group name '%s'",group_name.c_str());

}



std::string URDF2Gazebo::vector32str(const urdf::Vector3 vector, double (*conv)(double) = NULL)
{
    std::stringstream ss;
    ss << (conv ? conv(vector.x) : vector.x);
    ss << " ";
    ss << (conv ? conv(vector.y) : vector.y);
    ss << " ";
    ss << (conv ? conv(vector.z) : vector.z);
    return ss.str();
}

std::string URDF2Gazebo::values2str(unsigned int count, const double *values, double (*conv)(double) = NULL)
{
    std::stringstream ss;
    for (unsigned int i = 0 ; i < count ; i++)
    {
        if (i > 0)
            ss << " ";
        ss << (conv ? conv(values[i]) : values[i]);
    }
    return ss.str();
}

void URDF2Gazebo::addKeyValue(TiXmlElement *elem, const std::string& key, const std::string &value)
{
    TiXmlElement* child_elem = elem->FirstChildElement(key);
    if (child_elem)
    {
      std::string old_value = getGazeboValue(child_elem);
      if (old_value != value)
        logWarn("multiple inconsistent <%s> exists due to fixed joint reduction, overwriting previous value [%s] with [%s].",key.c_str(),old_value.c_str(),value.c_str());
      else
        logDebug("multiple <%s> exists due to fixed joint reduction, overwriting previous value.",key.c_str());
      elem->RemoveChild(child_elem); // remove if one already exists
    }

    TiXmlElement *ekey      = new TiXmlElement(key);
    TiXmlText    *text_ekey = new TiXmlText(value);
    ekey->LinkEndChild(text_ekey);    
    elem->LinkEndChild(ekey); 
}

void URDF2Gazebo::addTransform(TiXmlElement *elem, const::gazebo::math::Pose& transform)
{
    double cpos[3] = { transform.pos.x, transform.pos.y, transform.pos.z };
    gazebo::math::Vector3 e = transform.rot.GetAsEuler();
    double crot[3] = { e.x, e.y, e.z };

    // Notes:
    // mat.getRPY is broken, get back quaternion, use urdf::Rotation to get from quaternion to rpy
    //q.getRPY(crot[0],crot[1],crot[2]);
    //mat.getEulerYPR(crot[2],crot[1],crot[0]); // equivalent to below // bug with gimbal lock
    //mat.getRPY(crot[0],crot[1],crot[2]); // bug with gimbal lock
    
    /* set geometry transform */
    addKeyValue(elem, "xyz", values2str(3, cpos));
    addKeyValue(elem, "rpy", values2str(3, crot, rad2deg));  
}

std::string URDF2Gazebo::getGazeboValue(TiXmlElement* elem)
{
    std::string value_str;
    if (elem->Attribute("value"))
    {
      value_str = elem->Attribute("value");
    }
    else if(elem->FirstChild()) // FIXME: comment out check for now, as ros and system tinyxml does not match.  elem->FirstChild()->Type() == TiXmlNode::TINYXML_TEXT
    {
      value_str = elem->FirstChild()->ValueStr();
    }
    return value_str;
}

void URDF2Gazebo::parseGazeboExtension(TiXmlDocument &urdf_in)
{
  logDebug("parsing gazebo extension");
  TiXmlElement* robot_xml = urdf_in.FirstChildElement("robot");

  // Get all Gazebo extension elements, put everything in this->gazebo_extensions_ map, containing a key string (link/joint name) and values
  for (TiXmlElement* gazebo_xml = robot_xml->FirstChildElement("gazebo"); gazebo_xml; gazebo_xml = gazebo_xml->NextSiblingElement("gazebo"))
  {
    const char* ref = gazebo_xml->Attribute("reference");
    std::string ref_str;
    if (!ref)
    {
      logDebug("parsing gazebo extension for robot, reference is not specified");
      // copy extensions for robot (outside of link/joint)
      ref_str.clear();
    }
    else
    {
      logDebug("parsing gazebo extension for %s",ref);
      // copy extensions for link/joint
      ref_str = std::string(ref);
    }

    if (this->gazebo_extensions_.find(ref_str) == this->gazebo_extensions_.end())
    {
        std::vector<GazeboExtension*> extensions;
        logDebug("extension map for reference [%s] does not exist yet, creating new extension map",ref);
        this->gazebo_extensions_.insert(std::make_pair( ref_str, extensions ) );
    }

    // create and insert a new GazeboExtension into the map
    GazeboExtension* gazebo = new GazeboExtension();


    // begin parsing xml node
    for (TiXmlElement *child_elem = gazebo_xml->FirstChildElement(); child_elem; child_elem = child_elem->NextSiblingElement())
    {
      logDebug("child element : %s ", child_elem->ValueStr().c_str());

      gazebo->original_reference = ref_str;

      // go through all elements of the extension, extract what we know, and save the rest in blobs

      // material
      if (child_elem->ValueStr() == "material")
      {
          gazebo->material = getGazeboValue(child_elem);
          logDebug("   material %s",gazebo->material.c_str());
      }
      else if (child_elem->ValueStr() == "static")
      {
        std::string value_str = getGazeboValue(child_elem);

        // default of setting static flag is false
        if (value_str == "true" || value_str == "True" || value_str == "TRUE" || value_str == "yes" || value_str == "1")
          gazebo->setStaticFlag = true;
        else
          gazebo->setStaticFlag = false;

        logDebug("   setStaticFlag %d",gazebo->setStaticFlag);

      }
      else if (child_elem->ValueStr() == "turnGravityOff")
      {
        std::string value_str = getGazeboValue(child_elem);

        // default of turnGravityOff is false
        if (value_str == "true" || value_str == "True" || value_str == "TRUE" || value_str == "yes" || value_str == "1")
          gazebo->turnGravityOff = true;
        else
          gazebo->turnGravityOff = false;

        logDebug("   turnGravityOff %d",gazebo->turnGravityOff);

      }
      else if (child_elem->ValueStr() == "dampingFactor")
      {
          gazebo->is_dampingFactor = true;
          gazebo->dampingFactor = atof(getGazeboValue(child_elem).c_str());
          logDebug("   dampingFactor %f",gazebo->dampingFactor);
      }
      else if (child_elem->ValueStr() == "maxVel")
      {
          gazebo->is_maxVel = true;
          gazebo->maxVel = atof(getGazeboValue(child_elem).c_str());
          logDebug("   maxVel %f",gazebo->maxVel);
      }
      else if (child_elem->ValueStr() == "minDepth")
      {
          gazebo->is_minDepth = true;
          gazebo->minDepth = atof(getGazeboValue(child_elem).c_str());
          logDebug("   minDepth %f",gazebo->minDepth);
      }
      else if (child_elem->ValueStr() == "mu1")
      {
          gazebo->is_mu1 = true;
          gazebo->mu1 = atof(getGazeboValue(child_elem).c_str());
          logDebug("   mu1 %f",gazebo->mu1);
      }
      else if (child_elem->ValueStr() == "mu2")
      {
          gazebo->is_mu2 = true;
          gazebo->mu2 = atof(getGazeboValue(child_elem).c_str());
          logDebug("   mu2 %f",gazebo->mu2);
      }
      else if (child_elem->ValueStr() == "fdir1")
      {
          gazebo->fdir1 = getGazeboValue(child_elem);
          logDebug("   fdir1 %s",gazebo->fdir1.c_str());
      }
      else if (child_elem->ValueStr() == "kp")
      {
          gazebo->is_kp = true;
          gazebo->kp = atof(getGazeboValue(child_elem).c_str());
          logDebug("   kp %f",gazebo->kp);
      }
      else if (child_elem->ValueStr() == "kd")
      {
          gazebo->is_kd = true;
          gazebo->kd = atof(getGazeboValue(child_elem).c_str());
          logDebug("   kd %f",gazebo->kd);
      }
      else if (child_elem->ValueStr() == "genTexCoord")
      {
        std::string value_str = getGazeboValue(child_elem);

        // default of genTexCoord is false
        if (value_str == "true" || value_str == "True" || value_str == "TRUE" || value_str == "yes" || value_str == "1")
          gazebo->genTexCoord = true;
        else
          gazebo->genTexCoord = false;

        logDebug("   genTexCoord %d",gazebo->genTexCoord);

      }
      else if (child_elem->ValueStr() == "selfCollide")
      {
        std::string value_str = getGazeboValue(child_elem);

        // default of selfCollide is false
        if (value_str == "true" || value_str == "True" || value_str == "TRUE" || value_str == "yes" || value_str == "1")
          gazebo->selfCollide = true;
        else
          gazebo->selfCollide = false;

        logDebug("   selfCollide %d",gazebo->selfCollide);

      }
      else if (child_elem->ValueStr() == "laserRetro")
      {
          gazebo->is_laserRetro = true;
          gazebo->laserRetro = atof(getGazeboValue(child_elem).c_str());
          logDebug("   laserRetro %f",gazebo->laserRetro);
      }
      else if (child_elem->ValueStr() == "stopKp")
      {
          gazebo->is_stopKp = true;
          gazebo->stopKp = atof(getGazeboValue(child_elem).c_str());
          logDebug("   stopKp %f",gazebo->stopKp);
      }
      else if (child_elem->ValueStr() == "stopKd")
      {
          gazebo->is_stopKd = true;
          gazebo->stopKd = atof(getGazeboValue(child_elem).c_str());
          logDebug("   stopKd %f",gazebo->stopKd);
      }
      else if (child_elem->ValueStr() == "initial_joint_position")
      {
          gazebo->is_initial_joint_position = true;
          gazebo->initial_joint_position = atof(getGazeboValue(child_elem).c_str());
          logDebug("   initial_joint_position %f",gazebo->initial_joint_position);
      }
      else if (child_elem->ValueStr() == "fudgeFactor")
      {
          gazebo->is_fudgeFactor = true;
          gazebo->fudgeFactor = atof(getGazeboValue(child_elem).c_str());
          logDebug("   fudgeFactor %f",gazebo->fudgeFactor);
      }
      else if (child_elem->ValueStr() == "provideFeedback")
      {
          std::string value_str = getGazeboValue(child_elem);

          if (value_str == "true" || value_str == "True" || value_str == "TRUE" || value_str == "yes" || value_str == "1")
            gazebo->provideFeedback = true;
          else
            gazebo->provideFeedback = false;

          logDebug("   provideFeedback %d",(int)gazebo->provideFeedback);
      }
      else
      {
          std::ostringstream stream;
          stream << *child_elem;
          // save all unknown stuff in a vector of blobs
          TiXmlElement *blob = new TiXmlElement(*child_elem);
          gazebo->blobs.push_back(blob);
          logDebug("    blobs %s",stream.str().c_str());
      }
    }

    // insert into my map
    (this->gazebo_extensions_.find(ref_str))->second.push_back(gazebo);
  }
}

void URDF2Gazebo::insertGazeboExtensionGeom(TiXmlElement *elem,std::string link_name)
{
    for (std::map<std::string,std::vector<GazeboExtension*> >::iterator gazebo_it = this->gazebo_extensions_.begin();
         gazebo_it != this->gazebo_extensions_.end(); gazebo_it++)
    {
      for (std::vector<GazeboExtension*>::iterator ge = gazebo_it->second.begin(); ge != gazebo_it->second.end(); ge++)
      {
        if ((*ge)->original_reference == link_name)
        {
          // insert mu1, mu2, kp, kd for geom
          if ((*ge)->is_mu1)
              addKeyValue(elem, "mu1", values2str(1, &(*ge)->mu1) );
          if ((*ge)->is_mu2)
              addKeyValue(elem, "mu2", values2str(1, &(*ge)->mu2) );
          if (!(*ge)->fdir1.empty())
              addKeyValue(elem, "fdir1", (*ge)->fdir1);
          if ((*ge)->is_kp)
              addKeyValue(elem, "kp", values2str(1, &(*ge)->kp) );
          if ((*ge)->is_kd)
              addKeyValue(elem, "kd", values2str(1, &(*ge)->kd) );
          // max contact interpenetration correction velocity
          if ((*ge)->is_maxVel)
              addKeyValue(elem, "maxVel", values2str(1, &(*ge)->maxVel) );
          // contact interpenetration margin tolerance
          if ((*ge)->is_minDepth)
              addKeyValue(elem, "minDepth", values2str(1, &(*ge)->minDepth) );
          if ((*ge)->genTexCoord)
              addKeyValue(elem, "genTexCoord", "true");
          else
              addKeyValue(elem, "genTexCoord", "false");
          if ((*ge)->is_laserRetro)
              addKeyValue(elem, "laserRetro", values2str(1, &(*ge)->laserRetro) );
        }
      }
    }
}

void URDF2Gazebo::insertGazeboExtensionVisual(TiXmlElement *elem,std::string link_name)
{
    for (std::map<std::string,std::vector<GazeboExtension*> >::iterator gazebo_it = this->gazebo_extensions_.begin();
         gazebo_it != this->gazebo_extensions_.end(); gazebo_it++)
    {
      for (std::vector<GazeboExtension*>::iterator ge = gazebo_it->second.begin(); ge != gazebo_it->second.end(); ge++)
      {
        if ((*ge)->original_reference == link_name)
        {
          // insert material block
          if (!(*ge)->material.empty())
              addKeyValue(elem, "material", (*ge)->material);
        }
      }
    }
}

void URDF2Gazebo::insertGazeboExtensionBody(TiXmlElement *elem,std::string link_name)
{
    for (std::map<std::string,std::vector<GazeboExtension*> >::iterator gazebo_it = this->gazebo_extensions_.begin();
         gazebo_it != this->gazebo_extensions_.end(); gazebo_it++)
    {
      if (gazebo_it->first == link_name)
      {
        logDebug("body: reference %s link name %s",gazebo_it->first.c_str(),link_name.c_str());
        for (std::vector<GazeboExtension*>::iterator ge = gazebo_it->second.begin(); ge != gazebo_it->second.end(); ge++)
        {
          // insert turnGravityOff
          if ((*ge)->turnGravityOff)
              addKeyValue(elem, "turnGravityOff", "true");
          else
              addKeyValue(elem, "turnGravityOff", "false");

          // damping factor
          if ((*ge)->is_dampingFactor)
              addKeyValue(elem, "dampingFactor", values2str(1, &(*ge)->dampingFactor) );
          // selfCollide tag
          if ((*ge)->selfCollide)
              addKeyValue(elem, "selfCollide", "true");
          else
              addKeyValue(elem, "selfCollide", "false");
          // insert blobs into body
          for (std::vector<TiXmlElement*>::iterator blob_it = (*ge)->blobs.begin();
               blob_it != (*ge)->blobs.end(); blob_it++)
          {
              elem->LinkEndChild((*blob_it)->Clone());
          }
        }
      }
    }
}
void URDF2Gazebo::insertGazeboExtensionJoint(TiXmlElement *elem,std::string joint_name)
{
    for (std::map<std::string,std::vector<GazeboExtension*> >::iterator gazebo_it = this->gazebo_extensions_.begin();
         gazebo_it != this->gazebo_extensions_.end(); gazebo_it++)
    {
      if (gazebo_it->first == joint_name)
      {
        for (std::vector<GazeboExtension*>::iterator ge = gazebo_it->second.begin(); ge != gazebo_it->second.end(); ge++)
        {
          logDebug("geom: reference %s joint name %s, stopKp %f",gazebo_it->first.c_str(),joint_name.c_str(),(*ge)->stopKp);
          // insert stopKp, stopKd, fudgeFactor
          if ((*ge)->is_stopKp)
              addKeyValue(elem, "stopKp", values2str(1, &(*ge)->stopKp) );
          if ((*ge)->is_stopKd)
              addKeyValue(elem, "stopKd", values2str(1, &(*ge)->stopKd) );
          if ((*ge)->is_initial_joint_position)
              addKeyValue(elem, "initial_joint_position", values2str(1, &(*ge)->initial_joint_position) );
          if ((*ge)->is_fudgeFactor)
              addKeyValue(elem, "fudgeFactor", values2str(1, &(*ge)->fudgeFactor) );

          // insert provideFeedback
          if ((*ge)->provideFeedback)
              addKeyValue(elem, "provideFeedback", "true");
          else
              addKeyValue(elem, "provideFeedback", "false");
        }
      }
    }
}
void URDF2Gazebo::insertGazeboExtensionRobot(TiXmlElement *elem)
{
    for (std::map<std::string,std::vector<GazeboExtension*> >::iterator gazebo_it = this->gazebo_extensions_.begin();
         gazebo_it != this->gazebo_extensions_.end(); gazebo_it++)
    {
      if (gazebo_it->first.empty()) // no reference
      {
        for (std::vector<GazeboExtension*>::iterator ge = gazebo_it->second.begin(); ge != gazebo_it->second.end(); ge++)
        {
          // insert static flag
          if ((*ge)->setStaticFlag)
              addKeyValue(elem, "static", "true");
          else
              addKeyValue(elem, "static", "false");
          // insert blobs into robot
          for (std::vector<TiXmlElement*>::iterator blob_it = (*ge)->blobs.begin();
               blob_it != (*ge)->blobs.end(); blob_it++)
          {
              std::ostringstream stream_in;
              stream_in << *(*blob_it);
              logDebug("robot: reference empty, blobs for robot\n%s",stream_in.str().c_str());
              elem->LinkEndChild((*blob_it)->Clone());
          }
        }
      }
    }
}

std::string URDF2Gazebo::getGeometrySize(boost::shared_ptr<urdf::Geometry> geometry, int *sizeCount, double *sizeVals)
{
    std::string type("empty");
    
    switch (geometry->type)
    {
    case urdf::Geometry::BOX:
        type = "box";
        *sizeCount = 3;
        {
            boost::shared_ptr<const urdf::Box> box;
            box = boost::dynamic_pointer_cast< const urdf::Box >(geometry);
            sizeVals[0] = box->dim.x;
            sizeVals[1] = box->dim.y;
            sizeVals[2] = box->dim.z;
        }
        break;
    case urdf::Geometry::CYLINDER:
        type = "cylinder";
        *sizeCount = 2;
        {
            boost::shared_ptr<const urdf::Cylinder> cylinder;
            cylinder = boost::dynamic_pointer_cast<const urdf::Cylinder >(geometry);
            sizeVals[0] = cylinder->radius;
            sizeVals[1] = cylinder->length;
        }
        break;
    case urdf::Geometry::SPHERE:
        type = "sphere";
        *sizeCount = 1;
        {
          boost::shared_ptr<const urdf::Sphere> sphere;
          sphere = boost::dynamic_pointer_cast<const urdf::Sphere >(geometry);
          sizeVals[0] = sphere->radius;
        }
        break;
    case urdf::Geometry::MESH:
        type = "trimesh";
        *sizeCount = 3;
        {
          boost::shared_ptr<const urdf::Mesh> mesh;
          mesh = boost::dynamic_pointer_cast<const urdf::Mesh >(geometry);
          sizeVals[0] = mesh->scale.x;
          sizeVals[1] = mesh->scale.y;
          sizeVals[2] = mesh->scale.z;
        }
        break;
    default:
        *sizeCount = 0;
        printf("Unknown body type: %d in geometry\n", geometry->type);
        break;
    }
    
    return type;
}

std::string URDF2Gazebo::getGeometryBoundingBox(boost::shared_ptr<urdf::Geometry> geometry, double *sizeVals)
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
        printf("Unknown body type: %d in geometry\n", geometry->type);
        break;
    }
    
    return type;
}

void URDF2Gazebo::printMass(std::string link_name, dMass mass)
{
      logDebug("LINK NAME: [%s] from dMass",link_name.c_str());
      logDebug("     MASS: [%f]",mass.mass);
      logDebug("       CG: [%f %f %f]",mass.c[0],mass.c[1],mass.c[2]);
      logDebug("        I: [%f %f %f]",mass.I[0],mass.I[1],mass.I[2]);
      logDebug("           [%f %f %f]",mass.I[4],mass.I[5],mass.I[6]);
      logDebug("           [%f %f %f]",mass.I[8],mass.I[9],mass.I[10]);
}

void URDF2Gazebo::printMass(boost::shared_ptr<urdf::Link> link)
{
      logDebug("LINK NAME: [%s] from urdf::Link",link->name.c_str());
      logDebug("     MASS: [%f]",link->inertial->mass);
      logDebug("       CG: [%f %f %f]",link->inertial->origin.position.x,link->inertial->origin.position.y,link->inertial->origin.position.z);
      logDebug("        I: [%f %f %f]",link->inertial->ixx,link->inertial->ixy,link->inertial->ixz);
      logDebug("           [%f %f %f]",link->inertial->ixy,link->inertial->iyy,link->inertial->iyz);
      logDebug("           [%f %f %f]",link->inertial->ixz,link->inertial->iyz,link->inertial->izz);
}


void URDF2Gazebo::reduceFixedJoints(TiXmlElement *root, boost::shared_ptr<urdf::Link> link)
{

  logDebug("TREE: at [%s]",link->name.c_str());

  // if child is attached to self by fixed link first go up the tree, check it's children recursively
  for (unsigned int i = 0 ; i < link->child_links.size() ; ++i)
    if (link->child_links[i]->parent_joint->type == urdf::Joint::FIXED)
      reduceFixedJoints(root, link->child_links[i]);

  // reduce this link's stuff up the tree to parent but skip first joint if it's the world
  if (link->getParent() && link->getParent()->name != "world" && link->parent_joint && link->parent_joint->type == urdf::Joint::FIXED)
  {
    logDebug("TREE:   extension lumping from [%s] to [%s]",link->name.c_str(),link->getParent()->name.c_str());

    // lump gazebo extensions to parent, (give them new reference link names)
    reduceGazeboExtensionToParent(link);

    logDebug("TREE:   mass lumping from [%s] to [%s]",link->name.c_str(),link->getParent()->name.c_str());
    /* now lump all contents of this link to parent */
    if (link->inertial)
    {
      // get parent mass (in parent link frame)
      dMass parent_mass;
      if (!link->getParent()->inertial)
        link->getParent()->inertial.reset(new urdf::Inertial);
      dMassSetParameters(&parent_mass, link->getParent()->inertial->mass,
        link->getParent()->inertial->origin.position.x, link->getParent()->inertial->origin.position.y, link->getParent()->inertial->origin.position.z,
        link->getParent()->inertial->ixx, link->getParent()->inertial->iyy, link->getParent()->inertial->izz,
        link->getParent()->inertial->ixy, link->getParent()->inertial->ixz, link->getParent()->inertial->iyz);
      printMass(link->getParent()->name,parent_mass);
      printMass(link->getParent());
      // set link mass (in link frame)
      dMass link_mass;
      dMassSetParameters(&link_mass, link->inertial->mass,
        link->inertial->origin.position.x, link->inertial->origin.position.y, link->inertial->origin.position.z,
        link->inertial->ixx, link->inertial->iyy, link->inertial->izz,
        link->inertial->ixy, link->inertial->ixz, link->inertial->iyz);
      printMass(link->name,link_mass);
      printMass(link);
      // un-rotate link mass into parent link frame
      dMatrix3 R;
      double phi, theta, psi;
      link->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(phi,theta,psi);
      logDebug("debug: %f %f %f",phi,theta,psi);
      dRFromEulerAngles(R, phi, theta, psi);
      dMassRotate(&link_mass, R);
      printMass(link->name,link_mass);
      // un-translate link mass into parent link frame
      dMassTranslate(&link_mass, link->parent_joint->parent_to_joint_origin_transform.position.x
                               , link->parent_joint->parent_to_joint_origin_transform.position.y
                               , link->parent_joint->parent_to_joint_origin_transform.position.z);
      printMass(link->name,link_mass);
      // now link_mass is in the parent frame, add link_mass to parent_mass
      dMassAdd(&parent_mass,&link_mass); // now parent_mass contains link_mass in parent frame
      printMass(link->getParent()->name,parent_mass);
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
      printMass(link->getParent());
    }

    // lump visual to parent
    // lump all visual to parent, assign group name "lump::"+group name+"::'+link name
    // lump but keep the link name in(/as) the group name, so we can correlate visuals to visuals somehow.
    for (std::map<std::string, boost::shared_ptr<std::vector<boost::shared_ptr<urdf::Visual> > > >::iterator visuals_it = link->visual_groups.begin();
         visuals_it != link->visual_groups.end(); visuals_it++)
    {
      // if it's a "default" mesh, it should be added under "lump:"+link name
      if (visuals_it->first == "default")
      {
        std::string lump_group_name = std::string("lump::")+link->name;
        logDebug("adding modified lump group name [%s] to link [%s]",lump_group_name.c_str(),link->getParent()->name.c_str());
        for (std::vector<boost::shared_ptr<urdf::Visual> >::iterator visual_it = visuals_it->second->begin(); visual_it != visuals_it->second->end(); visual_it++)
        {
          // transform visual origin from link frame to parent link frame before adding to parent
          (*visual_it)->origin = transformToParentFrame((*visual_it)->origin, link->parent_joint->parent_to_joint_origin_transform);
          // add the modified visual to parent
          this->addVisual(link->getParent(),lump_group_name,*visual_it);
        }
      }
      else if (visuals_it->first.find(std::string("lump::")) == 0) // starts with "lump::"
      {
        // if it's a previously lumped mesh, relump under same group_name
        std::string lump_group_name = visuals_it->first;
        logDebug("re-lumping group name [%s] to link [%s]",lump_group_name.c_str(),link->getParent()->name.c_str());
        for (std::vector<boost::shared_ptr<urdf::Visual> >::iterator visual_it = visuals_it->second->begin(); visual_it != visuals_it->second->end(); visual_it++)
        {
          // transform visual origin from link frame to parent link frame before adding to parent
          (*visual_it)->origin = transformToParentFrame((*visual_it)->origin, link->parent_joint->parent_to_joint_origin_transform);
          // add the modified visual to parent
          this->addVisual(link->getParent(),lump_group_name,*visual_it);
        }
      }
    }

    // lump collision parent
    // lump all collision to parent, assign group name "lump::"+group name+"::'+link name
    // lump but keep the link name in(/as) the group name, so we can correlate visuals to collisions somehow.
    for (std::map<std::string, boost::shared_ptr<std::vector<boost::shared_ptr<urdf::Collision> > > >::iterator collisions_it = link->collision_groups.begin();
         collisions_it != link->collision_groups.end(); collisions_it++)
    {
      // if it's a "default" mesh, it should be added under "lump:"+link name
      if (collisions_it->first == "default")
      {
        std::string lump_group_name = std::string("lump::")+link->name;
        logDebug("adding modified lump group name [%s] to link [%s]",lump_group_name.c_str(),link->getParent()->name.c_str());
        for (std::vector<boost::shared_ptr<urdf::Collision> >::iterator collision_it = collisions_it->second->begin(); collision_it != collisions_it->second->end(); collision_it++)
        {
          // transform collision origin from link frame to parent link frame before adding to parent
          (*collision_it)->origin = transformToParentFrame((*collision_it)->origin, link->parent_joint->parent_to_joint_origin_transform);
          // add the modified collision to parent
          this->addCollision(link->getParent(),lump_group_name,*collision_it);
        }
      }
      else if (collisions_it->first.find(std::string("lump::")) == 0) // starts with "lump::"
      {
        // if it's a previously lumped mesh, relump under same group_name
        std::string lump_group_name = collisions_it->first;
        logDebug("re-lumping group name [%s] to link [%s]",lump_group_name.c_str(),link->getParent()->name.c_str());
        for (std::vector<boost::shared_ptr<urdf::Collision> >::iterator collision_it = collisions_it->second->begin(); collision_it != collisions_it->second->end(); collision_it++)
        {
          // transform collision origin from link frame to parent link frame before adding to parent
          (*collision_it)->origin = transformToParentFrame((*collision_it)->origin, link->parent_joint->parent_to_joint_origin_transform);
          // add the modified collision to parent
          this->addCollision(link->getParent(),lump_group_name,*collision_it);
        }
      }
    }
    printCollisionGroups(link->getParent());

    // STRATEGY: later when extracting visuals and collision pairs to construct <geom> for gazebo, given that geom(coillision) wraps visual,
    //           loop through lumped collisions, for each collision,
    //             call createGeom() using visual_groups.find(lump group name) to find potential visuals to create the <geom> tag for the body.
    //           This way, <geom> is created only if collision exists in first place, and if matching visual exists it is used, other a sphere default
    //             is created


    logDebug("BEGIN JOINT LUMPING");
    // set child link's parent_joint's parent link to a parent link up stream that does not have a fixed parent_joint
    for (unsigned int i = 0 ; i < link->child_links.size() ; ++i) {
      boost::shared_ptr<urdf::Joint> parent_joint = link->child_links[i]->parent_joint;
      if (parent_joint->type != urdf::Joint::FIXED) {
        // go down the tree until we hit a parent joint that is not fixed
        boost::shared_ptr<urdf::Link> new_parent_link = link;
        gazebo::math::Pose joint_anchor_transform; // set to identity first
        while(new_parent_link->parent_joint && new_parent_link->getParent()->name != "world" && new_parent_link->parent_joint->type == urdf::Joint::FIXED) {
          logDebug("  ...JOINT: searching: at [%s] checking if parent [%s] is attachable",new_parent_link->name.c_str(),new_parent_link->getParent()->name.c_str());
          joint_anchor_transform = joint_anchor_transform*joint_anchor_transform;
          parent_joint->parent_to_joint_origin_transform = transformToParentFrame(parent_joint->parent_to_joint_origin_transform,
                                                                 new_parent_link->parent_joint->parent_to_joint_origin_transform);
          new_parent_link = new_parent_link->getParent();
        }
        // now set the link->child_links[i]->parent_joint's parent link to the new_parent_link
        link->child_links[i]->setParent(new_parent_link);
        parent_joint->parent_link_name = new_parent_link->name;
        logDebug("  JOINT: reparenting for link %s joint %s from %s to %s",link->child_links[i]->name.c_str(),link->child_links[i]->parent_joint->name.c_str(),link->name.c_str(),new_parent_link->name.c_str());
        // and set the link->child_links[i]->parent_joint's parent_to_joint_orogin_transform as the aggregated
        //   anchor transform?
      }
    }
  }

  // continue down the tree for non-fixed joints
  for (unsigned int i = 0 ; i < link->child_links.size() ; ++i)
    if (link->child_links[i]->parent_joint->type != urdf::Joint::FIXED)
      reduceFixedJoints(root, link->child_links[i]);
}


void URDF2Gazebo::printCollisionGroups(boost::shared_ptr<urdf::Link> link)
{
  logDebug("COLLISION LUMPING: link: [%s] contains [%d] collisions",link->name.c_str(),(int)link->collision_groups.size());
  for (std::map<std::string, boost::shared_ptr<std::vector<boost::shared_ptr<urdf::Collision> > > >::iterator cols_it = link->collision_groups.begin();
       cols_it != link->collision_groups.end(); cols_it++)
  {
    logDebug("    collision_groups: [%s] has [%d] Collision objects",cols_it->first.c_str(),(int)cols_it->second->size());
    //for (std::vector<boost::shared_ptr<urdf::Collision> >::iterator collision_it = cols_it->second->begin(); collision_it != cols_it->second->end(); collision_it++)
    //  logDebug("     origins for debugging: [%s]",...cols_it->first.c_str());
  }
}

/// \brief returns the same transform in parent frame
urdf::Pose  URDF2Gazebo::transformToParentFrame(urdf::Pose transform_in_link_frame, urdf::Pose parent_to_link_transform)
{
    // transform to gazebo::math::Pose then call transformToParentFrame
    gazebo::math::Pose p1 = URDF2Gazebo::copyPose(transform_in_link_frame);
    gazebo::math::Pose p2 = URDF2Gazebo::copyPose(parent_to_link_transform);
    return URDF2Gazebo::copyPose(transformToParentFrame(p1,p2));
}
gazebo::math::Pose  URDF2Gazebo::transformToParentFrame(gazebo::math::Pose transform_in_link_frame, urdf::Pose parent_to_link_transform)
{
    // transform to gazebo::math::Pose then call transformToParentFrame
    gazebo::math::Pose p2 = URDF2Gazebo::copyPose(parent_to_link_transform);
    return transformToParentFrame(transform_in_link_frame,p2);
}

gazebo::math::Pose  URDF2Gazebo::transformToParentFrame(gazebo::math::Pose transform_in_link_frame, gazebo::math::Pose parent_to_link_transform)
{
    gazebo::math::Pose transform_in_parent_link_frame;
    // in the new revision, gazebo::math::Pose has some transform operator overrides
    //   rotate link pose to parent_link frame
    transform_in_parent_link_frame.pos = parent_to_link_transform.rot * transform_in_link_frame.pos;
    transform_in_parent_link_frame.rot = parent_to_link_transform.rot * transform_in_link_frame.rot;
    //   translate link to parent_link frame
    transform_in_parent_link_frame.pos = parent_to_link_transform.pos + transform_in_parent_link_frame.pos;

    return transform_in_parent_link_frame;
}

gazebo::math::Pose  URDF2Gazebo::inverseTransformToParentFrame(gazebo::math::Pose transform_in_link_frame, urdf::Pose parent_to_link_transform)
{
    gazebo::math::Pose transform_in_parent_link_frame;
    // in the new revision, gazebo::math::Pose has some transform operator overrides
    //   rotate link pose to parent_link frame
    urdf::Rotation ri = parent_to_link_transform.rotation.GetInverse();
    gazebo::math::Quaternion q1(ri.w, ri.x, ri.y, ri.z);
    transform_in_parent_link_frame.pos = q1 * transform_in_link_frame.pos;
    urdf::Rotation r2 = parent_to_link_transform.rotation.GetInverse();
    gazebo::math::Quaternion q3(r2.w, r2.x, r2.y, r2.z);
    transform_in_parent_link_frame.rot = q3 * transform_in_link_frame.rot;
    //   translate link to parent_link frame
    transform_in_parent_link_frame.pos.x = transform_in_parent_link_frame.pos.x - parent_to_link_transform.position.x;
    transform_in_parent_link_frame.pos.y = transform_in_parent_link_frame.pos.y - parent_to_link_transform.position.y;
    transform_in_parent_link_frame.pos.z = transform_in_parent_link_frame.pos.z - parent_to_link_transform.position.z;

    return transform_in_parent_link_frame;
}

void URDF2Gazebo::reduceGazeboExtensionToParent(boost::shared_ptr<urdf::Link> link)
{
  // this is a very complicated module that updates the plugins based on fixed joint reduction
  // really wish this could be a lot cleaner

  std::string link_name = link->name;
  std::string new_link_name = link->getParent()->name;

  logDebug("  EXTENSION: Reference lumping from [%s] to [%s]",link_name.c_str(), new_link_name.c_str());

  // update extension map with references to link_name
  listGazeboExtensions();

  std::map<std::string,std::vector<GazeboExtension*> >::iterator ext = this->gazebo_extensions_.find(link_name);
  if (ext != this->gazebo_extensions_.end())
  {

    // update reduction transform (for rays, cameras for now).  FIXME: contact frames too?
    for (std::vector<GazeboExtension*>::iterator ge = ext->second.begin(); ge != ext->second.end(); ge++)
    {
      (*ge)->reduction_transform = transformToParentFrame((*ge)->reduction_transform, link->parent_joint->parent_to_joint_origin_transform);
      // FIXME: if the sensor block has sensor:ray or sensor:camera or sensor:stereo_camera, look for/replace/insert reduction transform into xml block
      updateGazeboExtensionBlobsReductionTransform((*ge));
      // replace all instances of the link_name string in blobs with new_link_name
      updateGazeboExtensionFrameReplace(*ge, link, new_link_name);
    }

    // find existing extension with the new link reference
    std::map<std::string,std::vector<GazeboExtension*> >::iterator new_ext = this->gazebo_extensions_.find(new_link_name);
    // create new_extension if none exists
    if (new_ext == this->gazebo_extensions_.end())
    {
      std::vector<GazeboExtension*> extensions;
      this->gazebo_extensions_.insert(std::make_pair( new_link_name, extensions ) );
      new_ext = this->gazebo_extensions_.find(new_link_name);
    }

    // pop extensions from link's vector and push them into the new link's vector
    for (std::vector<GazeboExtension*>::iterator ge = ext->second.begin(); ge != ext->second.end(); ge++)
    {
      new_ext->second.push_back(*ge);
    }
    ext->second.clear();
  }

  listGazeboExtensions();

  // for extensions with empty reference, search and replace link name patterns within the plugin with new link name
  //   and assign the proper reduction transform for the link name pattern
  for (std::map<std::string,std::vector<GazeboExtension*> >::iterator gazebo_it = this->gazebo_extensions_.begin();
                                                        gazebo_it != this->gazebo_extensions_.end(); gazebo_it++)
    if (gazebo_it->first.empty())
    {
      // update reduction transform (for contacts, rays, cameras for now).
      for (std::vector<GazeboExtension*>::iterator ge = gazebo_it->second.begin(); ge != gazebo_it->second.end(); ge++)
        updateGazeboExtensionFrameReplace(*ge, link, new_link_name);
    }

  listGazeboExtensions();
}

void URDF2Gazebo::updateGazeboExtensionFrameReplace(GazeboExtension* ge, boost::shared_ptr<urdf::Link> link, std::string new_link_name)
{
  std::vector<TiXmlElement*> blobs = ge->blobs;

  std::string link_name = link->name;
  // HACK: need to do this more generally, but we also need to replace all instances of link name with new link name
  //       e.g. contact sensor refers to <geom>base_link_geom</geom> and it needs to be reparented to <geom>base_footprint_geom</geom>
  logDebug("  STRING REPLACE: instances of link name [%s] with new link name [%s]",link_name.c_str(),new_link_name.c_str());
  for (std::vector<TiXmlElement*>::iterator blob_it = blobs.begin(); blob_it != blobs.end(); blob_it++)
  {


      std::ostringstream stream_in;
      stream_in << *(*blob_it);
      std::string blob = stream_in.str();
      logDebug("        INITIAL STRING [%s-->%s]: %s",link_name.c_str(),new_link_name.c_str(),blob.c_str());

      if ((*blob_it)->ValueStr() == "sensor:contact")
      {
        // parse it and add/replace the reduction transform
        // find first instance of xyz and rpy, replace with reduction transform
        TiXmlNode* geomName = (*blob_it)->FirstChild("geom");
        if (geomName)
        {
          logDebug("  <geom>%s</geom> : %s --> %s",getGazeboValue(geomName->ToElement()).c_str(),link_name.c_str(),new_link_name.c_str());
          if (getGazeboValue(geomName->ToElement()) == link_name + std::string("_geom"))
          {
            (*blob_it)->RemoveChild(geomName);
            TiXmlElement* geom_name_key = new TiXmlElement("geom");
            std::ostringstream geom_name_stream;
            geom_name_stream << new_link_name << "_geom_" << link_name;
            TiXmlText* geom_name_txt = new TiXmlText(geom_name_stream.str());
            geom_name_key->LinkEndChild(geom_name_txt);
            (*blob_it)->LinkEndChild(geom_name_key);
          }
          // FIXME: chagning contact sensor's contact geom should trigger a update in sensor offset as well.
          // FIXME: but first we need to implement offsets in contact sensors
        }
      }

      if ((*blob_it)->ValueStr() == "controller:gazebo_ros_p3d" ||
          (*blob_it)->ValueStr() == "controller:gazebo_ros_imu" ||
          (*blob_it)->ValueStr() == "controller:gazebo_ros_projector")
      {
        // parse it and add/replace the reduction transform
        // find first instance of xyz and rpy, replace with reduction transform
        TiXmlNode* bodyName = (*blob_it)->FirstChild("bodyName");
        if (bodyName)
        {
          if (getGazeboValue(bodyName->ToElement()) == link_name)
          {
            logDebug("  <bodyName>%s</bodyName> : %s --> %s",getGazeboValue(bodyName->ToElement()).c_str(),link_name.c_str(),new_link_name.c_str());
            (*blob_it)->RemoveChild(bodyName);
            TiXmlElement* body_name_key = new TiXmlElement("bodyName");
            std::ostringstream body_name_stream;
            body_name_stream << new_link_name;
            TiXmlText* body_name_txt = new TiXmlText(body_name_stream.str());
            body_name_key->LinkEndChild(body_name_txt);
            (*blob_it)->LinkEndChild(body_name_key);
            logDebug("Re-parenting extension [%s] from [%s] to [%s] is broken, needs the transform to be complete",(*blob_it)->ValueStr().c_str(),link_name.c_str(),new_link_name.c_str());
          }
        }

        TiXmlNode* frameName = (*blob_it)->FirstChild("frameName");
        if (frameName)
        {
          if (getGazeboValue(frameName->ToElement()) == link_name)
          {
            logDebug("  <frameName>%s</frameName> : %s --> %s",getGazeboValue(frameName->ToElement()).c_str(),link_name.c_str(),new_link_name.c_str());
            (*blob_it)->RemoveChild(frameName);
            TiXmlElement* body_name_key = new TiXmlElement("frameName");
            std::ostringstream body_name_stream;
            body_name_stream << new_link_name;
            TiXmlText* body_name_txt = new TiXmlText(body_name_stream.str());
            body_name_key->LinkEndChild(body_name_txt);
            (*blob_it)->LinkEndChild(body_name_key);
            logDebug("Re-parenting extension [%s] from [%s] to [%s] is broken, needs the transform to be complete",(*blob_it)->ValueStr().c_str(),link_name.c_str(),new_link_name.c_str());


            // overwrite <xyzOffset> and <rpyOffset> if they exist
            if ((*blob_it)->ValueStr() == "controller:gazebo_ros_p3d" || (*blob_it)->ValueStr() == "controller:gazebo_ros_imu")
            {
              // parse it and add/replace the reduction transform
              // find first instance of xyzOffset and rpyOffset, replace with reduction transform
              logDebug("testing reduction transform for [%s] for frameName",(*blob_it)->ValueStr().c_str());
              for (TiXmlNode* el_it = (*blob_it)->FirstChild(); el_it; el_it = el_it->NextSibling())
              {
                std::ostringstream stream_debug;
                stream_debug << *el_it;
                logDebug("    %s",stream_debug.str().c_str());
              }

              // read user specified xyzOffset and rpyOffset, fill in the current reduction transform
              TiXmlNode* xyz_key = (*blob_it)->FirstChild("xyzOffset");
              if (xyz_key)
              {
                urdf::Vector3 v1 = parseVector3(xyz_key);
                ge->reduction_transform.pos = gazebo::math::Vector3(v1.x, v1.y, v1.z);
                // remove xyzOffset and rpyOffset
                (*blob_it)->RemoveChild(xyz_key);
              }
              TiXmlNode* rpy_key = (*blob_it)->FirstChild("rpyOffset");
              if (rpy_key)
              {
                urdf::Vector3 rpy = parseVector3(rpy_key, M_PI/180.0);
                ge->reduction_transform.rot = gazebo::math::Quaternion::EulerToQuaternion(rpy.x, rpy.y, rpy.z);
                // remove xyzOffset and rpyOffset
                (*blob_it)->RemoveChild(rpy_key);
              }

              // pass through the parent transform from fixed joint reduction
              ge->reduction_transform = inverseTransformToParentFrame(ge->reduction_transform, link->parent_joint->parent_to_joint_origin_transform);

              // create new offset xml blocks
              xyz_key = new TiXmlElement("xyzOffset");
              rpy_key = new TiXmlElement("rpyOffset");

              // create new offset xml blocks
              urdf::Vector3 reduction_xyz(ge->reduction_transform.pos.x,ge->reduction_transform.pos.y,ge->reduction_transform.pos.z);
              urdf::Rotation reduction_q(ge->reduction_transform.rot.x,ge->reduction_transform.rot.y,ge->reduction_transform.rot.z,ge->reduction_transform.rot.w);

              std::ostringstream xyz_stream, rpy_stream;
              xyz_stream << reduction_xyz.x << " " << reduction_xyz.y << " " << reduction_xyz.z;
              urdf::Vector3 reduction_rpy;
              reduction_q.getRPY(reduction_rpy.x,reduction_rpy.y,reduction_rpy.z); // convert to Euler angles for Gazebo XML
              rpy_stream << reduction_rpy.x*180./M_PI << " " << reduction_rpy.y*180./M_PI << " " << reduction_rpy.z*180./M_PI; // convert to degrees for Gazebo (though ROS is in Radians)

              TiXmlText* xyz_txt = new TiXmlText(xyz_stream.str());
              TiXmlText* rpy_txt = new TiXmlText(rpy_stream.str());

              xyz_key->LinkEndChild(xyz_txt);
              rpy_key->LinkEndChild(rpy_txt);

              (*blob_it)->LinkEndChild(xyz_key);
              (*blob_it)->LinkEndChild(rpy_key);
            }
          }
        }

        // updates link reference for <projector> inside of controller:gazebo_ros_projector
        TiXmlNode* projectorName = (*blob_it)->FirstChild("projector");
        {
          // change body name, in this case, projector element is "link name"/"projector name"
          // and we only want to replace link name
          if (projectorName)
          {
            std::string projector_name =  getGazeboValue(projectorName->ToElement());
            // extract projector link name and projector name
            size_t pos = projector_name.find("/");
            if (pos == std::string::npos)
              logError("no slash in projector tag, should be link_name/projector_name");
            std::string projector_link_name = projector_name.substr(0, pos);

            if (projector_link_name == link_name)
            {
              // do the replacement
              projector_name = new_link_name+"/"+projector_name.substr(pos+1, projector_name.size());

              logDebug("  <projector>%s</projector> : %s --> %s",projector_link_name.c_str(),link_name.c_str(),new_link_name.c_str());
              (*blob_it)->RemoveChild(projectorName);
              TiXmlElement* body_name_key = new TiXmlElement("projector");
              std::ostringstream body_name_stream;
              body_name_stream << projector_name;
              TiXmlText* body_name_txt = new TiXmlText(body_name_stream.str());
              body_name_key->LinkEndChild(body_name_txt);
              (*blob_it)->LinkEndChild(body_name_key);
              logDebug("Re-parenting extension [%s] from [%s] to [%s] is broken, needs the transform to be complete",(*blob_it)->ValueStr().c_str(),link_name.c_str(),new_link_name.c_str());
            }
          }
        }
      }
      if ((*blob_it)->ValueStr() == "gripper")
      {
        TiXmlNode* gripper_link = (*blob_it)->FirstChild("gripper_link");
        if (gripper_link)
        {
          if (getGazeboValue(gripper_link->ToElement()) == link_name)
          {
            logDebug("  <gripper_link>%s</gripper_link> : %s --> %s",getGazeboValue(gripper_link->ToElement()).c_str(),link_name.c_str(),new_link_name.c_str());
            (*blob_it)->RemoveChild(gripper_link);
            TiXmlElement* body_name_key = new TiXmlElement("gripper_link");
            std::ostringstream body_name_stream;
            body_name_stream << new_link_name;
            TiXmlText* body_name_txt = new TiXmlText(body_name_stream.str());
            body_name_key->LinkEndChild(body_name_txt);
            (*blob_it)->LinkEndChild(body_name_key);
            logDebug("Re-parenting extension [%s] from [%s] to [%s] is broken, needs the transform to be complete",(*blob_it)->ValueStr().c_str(),link_name.c_str(),new_link_name.c_str());
          }
        }
        TiXmlNode* palm_link = (*blob_it)->FirstChild("palm_link");
        if (palm_link)
        {
          if (getGazeboValue(palm_link->ToElement()) == link_name)
          {
            logDebug("  <palm_link>%s</palm_link> : %s --> %s",getGazeboValue(palm_link->ToElement()).c_str(),link_name.c_str(),new_link_name.c_str());
            (*blob_it)->RemoveChild(palm_link);
            TiXmlElement* body_name_key = new TiXmlElement("palm_link");
            std::ostringstream body_name_stream;
            body_name_stream << new_link_name;
            TiXmlText* body_name_txt = new TiXmlText(body_name_stream.str());
            body_name_key->LinkEndChild(body_name_txt);
            (*blob_it)->LinkEndChild(body_name_key);
            logDebug("Re-parenting extension [%s] from [%s] to [%s] is broken, needs the transform to be complete",(*blob_it)->ValueStr().c_str(),link_name.c_str(),new_link_name.c_str());
          }
        }
      }
      if ((*blob_it)->ValueStr() == "joint:hinge")
      {
        // parse it and add/replace the reduction transform
        // find first instance of xyz and rpy, replace with reduction transform
        TiXmlNode* body1 = (*blob_it)->FirstChild("body1");
        if (body1)
        {
          if (getGazeboValue(body1->ToElement()) == link_name)
          {
            logDebug("  <body1>%s</body1> : %s --> %s",getGazeboValue(body1->ToElement()).c_str(),link_name.c_str(),new_link_name.c_str());
            (*blob_it)->RemoveChild(body1);
            TiXmlElement* body1_name_key = new TiXmlElement("body1");
            std::ostringstream body1_name_stream;
            body1_name_stream << new_link_name;
            TiXmlText* body1_name_txt = new TiXmlText(body1_name_stream.str());
            body1_name_key->LinkEndChild(body1_name_txt);
            (*blob_it)->LinkEndChild(body1_name_key);
          }
        }
        TiXmlNode* body2 = (*blob_it)->FirstChild("body2");
        if (body2)
        {
          if (getGazeboValue(body2->ToElement()) == link_name)
          {
            logDebug("  <body2>%s</body2> : %s --> %s",getGazeboValue(body2->ToElement()).c_str(),link_name.c_str(),new_link_name.c_str());
            (*blob_it)->RemoveChild(body2);
            TiXmlElement* body2_name_key = new TiXmlElement("body2");
            std::ostringstream body2_name_stream;
            body2_name_stream << new_link_name;
            TiXmlText* body2_name_txt = new TiXmlText(body2_name_stream.str());
            body2_name_key->LinkEndChild(body2_name_txt);
            (*blob_it)->LinkEndChild(body2_name_key);
          }
        }
        TiXmlNode* anchor = (*blob_it)->FirstChild("anchor");
        if (anchor)
        {
          if (getGazeboValue(anchor->ToElement()) == link_name)
          {
            logDebug("  <anchor>%s</anchor> : %s --> %s",getGazeboValue(anchor->ToElement()).c_str(),link_name.c_str(),new_link_name.c_str());
            (*blob_it)->RemoveChild(anchor);
            TiXmlElement* anchor_name_key = new TiXmlElement("anchor");
            std::ostringstream anchor_name_stream;
            anchor_name_stream << new_link_name;
            TiXmlText* anchor_name_txt = new TiXmlText(anchor_name_stream.str());
            anchor_name_key->LinkEndChild(anchor_name_txt);
            (*blob_it)->LinkEndChild(anchor_name_key);
          }
        }
      }


      std::ostringstream stream_out;
      stream_out << *(*blob_it);
      logDebug("        STRING REPLACED: %s",stream_out.str().c_str());
  }
}

void URDF2Gazebo::updateGazeboExtensionBlobsReductionTransform(GazeboExtension* ge)
{
  for (std::vector<TiXmlElement*>::iterator blob_it = ge->blobs.begin(); blob_it != ge->blobs.end(); blob_it++)
  {
    // overwrite <xyz> and <rpy> if they exist
    if ((*blob_it)->ValueStr() == "sensor:ray" || (*blob_it)->ValueStr() == "sensor:camera")
    {
      // parse it and add/replace the reduction transform
      // find first instance of xyz and rpy, replace with reduction transform
      logDebug("testing reduction transform for [%s]",(*blob_it)->ValueStr().c_str());
      for (TiXmlNode* el_it = (*blob_it)->FirstChild(); el_it; el_it = el_it->NextSibling())
      {
        std::ostringstream stream_in;
        stream_in << *el_it;
        logDebug("    %s",stream_in.str().c_str());
      }

      TiXmlNode* xyz_key = (*blob_it)->FirstChild("xyz");
      if (xyz_key)
        (*blob_it)->RemoveChild(xyz_key);
      TiXmlNode* rpy_key = (*blob_it)->FirstChild("rpy");
      if (rpy_key)
        (*blob_it)->RemoveChild(rpy_key);

      xyz_key = new TiXmlElement("xyz");
      rpy_key = new TiXmlElement("rpy");

      // FIXME:  we should read xyz, rpy and aggregate it to reduction_transform instead of just throwing the info away.
      urdf::Vector3 reduction_xyz(ge->reduction_transform.pos.x,ge->reduction_transform.pos.y,ge->reduction_transform.pos.z);
      urdf::Rotation reduction_q(ge->reduction_transform.rot.x,ge->reduction_transform.rot.y,ge->reduction_transform.rot.z,ge->reduction_transform.rot.w);

      std::ostringstream xyz_stream, rpy_stream;
      xyz_stream << reduction_xyz.x << " " << reduction_xyz.y << " " << reduction_xyz.z;
      urdf::Vector3 reduction_rpy;
      reduction_q.getRPY(reduction_rpy.x,reduction_rpy.y,reduction_rpy.z); // convert to Euler angles for Gazebo XML
      rpy_stream << reduction_rpy.x*180./M_PI << " " << reduction_rpy.y*180./M_PI << " " << reduction_rpy.z*180./M_PI; // convert to degrees for Gazebo (though ROS is in Radians)

      TiXmlText* xyz_txt = new TiXmlText(xyz_stream.str());
      TiXmlText* rpy_txt = new TiXmlText(rpy_stream.str());

      xyz_key->LinkEndChild(xyz_txt);
      rpy_key->LinkEndChild(rpy_txt);

      (*blob_it)->LinkEndChild(xyz_key);
      (*blob_it)->LinkEndChild(rpy_key);
    }

    // overwrite <pose> (xyz/rpy) if it exists
    if ((*blob_it)->ValueStr() == "projector")
    {
      // parse it and add/replace the reduction transform
      // find first instance of xyz and rpy, replace with reduction transform
      logDebug("testing reduction transform for [%s]",(*blob_it)->ValueStr().c_str());
      for (TiXmlNode* el_it = (*blob_it)->FirstChild(); el_it; el_it = el_it->NextSibling())
      {
        std::ostringstream stream_in;
        stream_in << *el_it;
        logDebug("    %s",stream_in.str().c_str());
      }

      /* should read <pose>...</pose> and agregate reduction_transform */
      TiXmlNode* pose_key = (*blob_it)->FirstChild("pose");
      // read pose and save it

      // remove the tag for now
      if (pose_key) (*blob_it)->RemoveChild(pose_key);

      // convert reduction_transform to values
      urdf::Vector3 reduction_xyz(ge->reduction_transform.pos.x,ge->reduction_transform.pos.y,ge->reduction_transform.pos.z);
      urdf::Rotation reduction_q(ge->reduction_transform.rot.x,ge->reduction_transform.rot.y,ge->reduction_transform.rot.z,ge->reduction_transform.rot.w);
      urdf::Vector3 reduction_rpy;
      reduction_q.getRPY(reduction_rpy.x,reduction_rpy.y,reduction_rpy.z); // convert to Euler angles for Gazebo XML
      // output updated pose to text
      std::ostringstream pose_stream;
      pose_stream << reduction_xyz.x << " " << reduction_xyz.y << " " << reduction_xyz.z << " "
                  << reduction_rpy.x << " " << reduction_rpy.y << " " << reduction_rpy.z; // convert to degrees for Gazebo (though ROS is in Radians)
      TiXmlText* pose_txt = new TiXmlText(pose_stream.str());

      pose_key = new TiXmlElement("pose");
      pose_key->LinkEndChild(pose_txt);

      (*blob_it)->LinkEndChild(pose_key);
    }
  }
}


void URDF2Gazebo::listGazeboExtensions()
{
  logDebug("====================================================================");
  for (std::map<std::string,std::vector<GazeboExtension*> >::iterator gazebo_it = this->gazebo_extensions_.begin(); gazebo_it != this->gazebo_extensions_.end(); gazebo_it++)
  {
    int ext_count = 0;
    for (std::vector<GazeboExtension*>::iterator ge = gazebo_it->second.begin(); ge != gazebo_it->second.end(); ge++)
    {
      if ((*ge)->blobs.size() > 0)
      {
        logDebug("  PRINTING [%d] BLOBS for extension [%d] referencing [%s]",(int)(*ge)->blobs.size(),ext_count++,gazebo_it->first.c_str());
        for (std::vector<TiXmlElement*>::iterator blob_it = (*ge)->blobs.begin(); blob_it != (*ge)->blobs.end(); blob_it++)
        {
            std::ostringstream stream_in;
            stream_in << *(*blob_it);
            logDebug("    BLOB: %s",stream_in.str().c_str());
        }
      }
    }
  }
  logDebug("====================================================================");
}

void URDF2Gazebo::listGazeboExtensions(std::string reference)
{
  logDebug("--------------------------------------------------------------------");
  for (std::map<std::string,std::vector<GazeboExtension*> >::iterator gazebo_it = this->gazebo_extensions_.begin(); gazebo_it != this->gazebo_extensions_.end(); gazebo_it++)
  {
    if (gazebo_it->first == reference)
    {
      logDebug("  PRINTING [%d] EXTENSIONS referencing [%s]",(int)gazebo_it->second.size(),reference.c_str());
      for (std::vector<GazeboExtension*>::iterator ge = gazebo_it->second.begin(); ge != gazebo_it->second.end(); ge++)
      {
        for (std::vector<TiXmlElement*>::iterator blob_it = (*ge)->blobs.begin(); blob_it != (*ge)->blobs.end(); blob_it++)
        {
          std::ostringstream stream_in;
          stream_in << *(*blob_it);
          logDebug("    BLOB: %s",stream_in.str().c_str());
        }
      }
    }
  }
  logDebug("--------------------------------------------------------------------");
}

void URDF2Gazebo::convertLink(TiXmlElement *root, boost::shared_ptr<const urdf::Link> link, const gazebo::math::Pose &transform, bool enforce_limits,bool reduce_fixed_joints)
{
    gazebo::math::Pose currentTransform = transform;

    // must have an <inertial> block and cannot have zero mass.  allow det(I) == zero, such as in the case of point mass geoms.
    if (link->name != "world" && ((!link->inertial) || gazebo::math::equal(link->inertial->mass, 0.0)))
    {
      if(!link->child_links.empty())
        logWarn("urdf2gazebo: link(%s) has no inertia, %d children link ignored.", link->name.c_str(),(int)link->child_links.size());

      if(!link->child_joints.empty())
        logWarn("urdf2gazebo: link(%s) has no inertia, %d children joint ignored.", link->name.c_str(),(int)link->child_joints.size());

      if(link->parent_joint)
        logWarn("urdf2gazebo: link(%s) has no inertia, parent joint(%s) is ignored.",link->name.c_str(), link->parent_joint->name.c_str());

      logWarn("urdf2gazebo: link(%s) has no inertia, not modeled in gazebo.", link->name.c_str());
      return;
    }

    /* create <body:...> block for non fixed joint attached bodies */
    if ((link->getParent() && link->getParent()->name == "world") || 
        !reduce_fixed_joints || (!link->parent_joint || link->parent_joint->type != urdf::Joint::FIXED))
      createBody(root, link, currentTransform, enforce_limits, reduce_fixed_joints);

    // recurse into children
    for (unsigned int i = 0 ; i < link->child_links.size() ; ++i)
        convertLink(root, link->child_links[i], currentTransform, enforce_limits, reduce_fixed_joints);
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

void URDF2Gazebo::createBody(TiXmlElement *root, boost::shared_ptr<const urdf::Link> link, gazebo::math::Pose &currentTransform, bool enforce_limits, bool reduce_fixed_joints)
{
    int linkGeomSize;
    double linkSize[3];

    /* create new body */
    TiXmlElement *elem     = new TiXmlElement("body:empty"); // FIXME:  does it matter what the collision type of the body is?

    /* set body name */
    elem->SetAttribute("name", link->name);

    /* set mass properties */
    addKeyValue(elem, "massMatrix", "true");
    addKeyValue(elem, "mass", values2str(1, &link->inertial->mass));
    
    addKeyValue(elem, "ixx", values2str(1, &link->inertial->ixx));
    addKeyValue(elem, "ixy", values2str(1, &link->inertial->ixy));
    addKeyValue(elem, "ixz", values2str(1, &link->inertial->ixz));
    addKeyValue(elem, "iyy", values2str(1, &link->inertial->iyy));
    addKeyValue(elem, "iyz", values2str(1, &link->inertial->iyz));
    addKeyValue(elem, "izz", values2str(1, &link->inertial->izz));
    
    addKeyValue(elem, "cx", values2str(1, &link->inertial->origin.position.x));
    addKeyValue(elem, "cy", values2str(1, &link->inertial->origin.position.y));
    addKeyValue(elem, "cz", values2str(1, &link->inertial->origin.position.z));
  
    double roll,pitch,yaw;
    link->inertial->origin.rotation.getRPY(roll,pitch,yaw);
    if (!gazebo::math::equal(roll, 0.0) || !gazebo::math::equal(pitch, 0.0) || !gazebo::math::equal(yaw, 0.0))
        logError("rotation of inertial frame is not supported\n");

    /* compute global transform */
    gazebo::math::Pose localTransform;
    // this is the transform from parent link to current link
    // this transform does not exist for the root link
    if (link->parent_joint)
    {
      logDebug("%s has a parent joint",link->name.c_str());
      localTransform = URDF2Gazebo::copyPose(link->parent_joint->parent_to_joint_origin_transform);
      currentTransform = localTransform * currentTransform;
    }
    else
      logDebug("%s has no parent joint",link->name.c_str());


    // create origin tag for this element
    addTransform(elem, currentTransform);

    // make additional geoms using the lumped stuff
    // FIXME:  what's the pairing strategy between visuals and collisions?
    //      when only visuals exists?
    //      when only collisions exists?
    //         originally, there's one visual and one collision per link, which maps nicely to <body><geom><visual> heirarchy
    //         now we have multiple visuals and collisions...  how do we map?
    // loop through all default visuals and lump visuals
    for (std::map<std::string, boost::shared_ptr<std::vector<boost::shared_ptr<urdf::Collision> > > >::const_iterator collisions_it = link->collision_groups.begin();
         collisions_it != link->collision_groups.end(); collisions_it++)
    {
      if (collisions_it->first == "default")
      {
        logDebug("creating default geom for link [%s]",link->name.c_str());
        boost::shared_ptr<urdf::Collision> collision = *(collisions_it->second->begin());
        boost::shared_ptr<urdf::Visual> visual = *(link->visual_groups.find(collisions_it->first)->second->begin());

        std::string collision_type = "empty";
        if (!collision || !collision->geometry)
          logDebug("urdf2gazebo: link(%s) does not have a collision or geometry tag. No <geometry> will be assigned.", link->name.c_str());
        else
          collision_type = getGeometrySize(collision->geometry, &linkGeomSize, linkSize); // add collision information
        
        /* make a <geom:...> block */
        createGeom(elem, link, collision_type, collision, visual, linkGeomSize, linkSize, link->name);
      }
      else if (collisions_it->first.find(std::string("lump::")) == 0) // starts with "lump::"
      {
        logDebug("creating lump geom [%s] for link [%s]",collisions_it->first.c_str(),link->name.c_str());
        boost::shared_ptr<urdf::Collision> collision = *(collisions_it->second->begin());
        boost::shared_ptr<urdf::Visual> visual = *(link->visual_groups.find(collisions_it->first)->second->begin());

        std::string collision_type = "empty";
        if (!collision || !collision->geometry)
          logDebug("urdf2gazebo: link(%s) does not have a collision or geometry tag. No <geometry> will be assigned.", link->name.c_str());
        else
          collision_type = getGeometrySize(collision->geometry, &linkGeomSize, linkSize); // add collision information
        
        std::string original_reference = collisions_it->first.substr(6);
        /* make a <geom:...> block */
        createGeom(elem, link, collision_type, collision, visual, linkGeomSize, linkSize, original_reference);
      }
    }

    /* copy gazebo extensions data */
    insertGazeboExtensionBody(elem,link->name);
    
    /* add body to document */
    root->LinkEndChild(elem);

    /* make a <joint:...> block */
    createJoint(root, link, currentTransform, enforce_limits, reduce_fixed_joints);

}


void URDF2Gazebo::createJoint(TiXmlElement *root, boost::shared_ptr<const urdf::Link> link, gazebo::math::Pose &currentTransform, bool enforce_limits,bool reduce_fixed_joints)
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
            jtype = "hinge";
            break;
        case urdf::Joint::PRISMATIC:
            jtype = "slider";
            break;
        case urdf::Joint::FLOATING:
        case urdf::Joint::PLANAR:
            break;
        case urdf::Joint::FIXED:
            jtype = "fixed";
            break;
        default:
            printf("Unknown joint type: %d in link '%s'\n", link->parent_joint->type, link->name.c_str());
            break;
      }
    }
    
    // skip if joint type is fixed and we are not faking it with a hinge, skip/return
    //   with the exception of root link being world, because there's no lumping there
    if (link->getParent() && link->getParent()->name != "world" && jtype == "fixed" && reduce_fixed_joints) return;

    if (!jtype.empty())
    {
        TiXmlElement *joint;
        if (jtype == "fixed") joint = new TiXmlElement("joint:hinge");
        else                  joint = new TiXmlElement("joint:" + jtype);
        joint->SetAttribute("name", link->parent_joint->name);
        addKeyValue(joint, "body1", link->name);
        addKeyValue(joint, "body2", link->getParent()->name);
        addKeyValue(joint, "anchor", link->name);
        
        if (jtype == "fixed")
        {
            addKeyValue(joint, "lowStop", "0");
            addKeyValue(joint, "highStop", "0");
            addKeyValue(joint, "axis", "0 0 1");
            addKeyValue(joint, "damping", "0");
        }
        else
        {
            gazebo::math::Vector3 rotatedJointAxis = currentTransform.rot.RotateVector(gazebo::math::Vector3(link->parent_joint->axis.x,
                                                                                                             link->parent_joint->axis.y,
                                                                                                             link->parent_joint->axis.z ));
            double rotatedJointAxisArray[3] = { rotatedJointAxis.x, rotatedJointAxis.y, rotatedJointAxis.z };
            addKeyValue(joint, "axis", values2str(3, rotatedJointAxisArray));
            if (link->parent_joint->dynamics)
              addKeyValue(joint, "damping",  values2str(1, &link->parent_joint->dynamics->damping       ));

            // Joint Anchor is deprecated
            // urdf::Transform currentAnchorTransform( currentTransform ); 
            // currentAnchorTransform.setOrigin( urdf::Vector3(0, 0, 0) ); 
            // urdf::Vector3 jointAnchor = currentAnchorTransform * urdf::Vector3( link->parent_joint->anchor.x,
            //                                                             link->parent_joint->anchor.y,
            //                                                             link->parent_joint->anchor.z ); 
            // double tmpAnchor[3] =  { jointAnchor.x(), jointAnchor.y(), jointAnchor.z() };
            // addKeyValue(joint, "anchorOffset", values2str(3, tmpAnchor));
            
            if (enforce_limits && link->parent_joint->limits)
            {
                if (jtype == "slider")
                {
                    addKeyValue(joint, "lowStop",  values2str(1, &link->parent_joint->limits->lower       ));
                    addKeyValue(joint, "highStop", values2str(1, &link->parent_joint->limits->upper       ));
                }
                else if (link->parent_joint->type != urdf::Joint::CONTINUOUS)
                {
                    double *lowstop  = &link->parent_joint->limits->lower;
                    double *highstop = &link->parent_joint->limits->upper;
                    // enforce ode bounds, this will need to be fixed
                    if (*lowstop > *highstop)
                    {
                      logWarn("urdf2gazebo: hinge joint limits: lowStop > highStop, setting lowStop to highStop.");
                      *lowstop = *highstop;
                    }
                    addKeyValue(joint, "lowStop",  values2str(1, &link->parent_joint->limits->lower, rad2deg));
                    addKeyValue(joint, "highStop", values2str(1, &link->parent_joint->limits->upper, rad2deg));
                }
            }
        }
        /* copy gazebo extensions data */
        insertGazeboExtensionJoint(joint,link->parent_joint->name);

        /* add joint to document */
        root->LinkEndChild(joint);
    }
}


void URDF2Gazebo::createGeom(TiXmlElement* elem, boost::shared_ptr<const urdf::Link> link,std::string collision_type, boost::shared_ptr<urdf::Collision> collision, boost::shared_ptr<urdf::Visual> visual, int linkGeomSize, double* linkSize, std::string original_reference)
{
    /* begin create geometry node, skip if no collision specified */
    if (collision_type != "empty")
    {
        TiXmlElement *geom     = new TiXmlElement("geom:" + collision_type);
        /* set its name */
        if (original_reference == link->name)
          geom->SetAttribute("name", link->name + std::string("_geom"));
        else
          geom->SetAttribute("name", link->name + std::string("_geom_")+original_reference);
        
        /* set transform */
        addKeyValue(geom, "xyz", vector32str(collision->origin.position));
        double rpy[3];
        collision->origin.rotation.getRPY(rpy[0],rpy[1],rpy[2]);
        addKeyValue(geom, "rpy", values2str(3, rpy, rad2deg));
        
        if (collision->geometry->type == urdf::Geometry::MESH)
        {  
            boost::shared_ptr<const urdf::Mesh> mesh;
            mesh = boost::dynamic_pointer_cast<const urdf::Mesh >(collision->geometry);
            /* set mesh size or scale */
            addKeyValue(geom, "scale", vector32str(mesh->scale));

            /* set mesh file */
            /* expand to get absolute mesh filename */
            std::string fullname = mesh->filename;

            /* resource retriever is ros based
            if (fullname.find("package://") == 0)
            {
              fullname.erase(0, strlen("package://"));
              size_t pos = fullname.find("/");
              if (pos == std::string::npos)
              {
                logFatal("Could not parse package:// format for [%s]",mesh->filename.c_str());
              }

              std::string package = fullname.substr(0, pos);
              fullname.erase(0, pos);
              std::string package_path = ros::package::getPath(package);

              if (package_path.empty())
              {
                logFatal("%s Package[%s] does not exist",mesh->filename.c_str(),package.c_str());
              }

              fullname = package_path + fullname;
            }
            */

            // give some warning if file does not exist.
            std::ifstream fin; fin.open(fullname.c_str(), std::ios::in); fin.close();
            if (fin.fail())
              logError("filename referred by mesh [%s] does not appear to exist.",mesh->filename.c_str());

            // add mesh filename
            addKeyValue(geom, "mesh", fullname);
            
        }
        else
        {
            /* set geometry size */
            addKeyValue(geom, "size", values2str(linkGeomSize, linkSize));
        }
        
        /* set additional data from extensions */
        insertGazeboExtensionGeom(geom,original_reference);

        // FIXME: unfortunately, visual is not created if collision is missing!!!
        createVisual(geom, link, collision_type, collision, visual,original_reference);

        /* add geometry to body */
        elem->LinkEndChild(geom);
    }
}

void URDF2Gazebo::createVisual(TiXmlElement *geom, boost::shared_ptr<const urdf::Link> link, std::string collision_type, boost::shared_ptr<urdf::Collision> collision, boost::shared_ptr<urdf::Visual> visual,std::string original_reference)
{
    /* begin create gazebo visual node */
    TiXmlElement *gazebo_visual = new TiXmlElement("visual");

    /* compute the visualisation transfrom */
    gazebo::math::Pose coll = URDF2Gazebo::copyPose(collision->origin);

    // coll = coll.GetInverse();
    
    gazebo::math::Pose vis;
    if(!visual) 
    {
      logWarn("urdf2gazebo: link(%s) does not have a visual tag, using zero transform.", link->name.c_str());
      vis = gazebo::math::Pose();
    }
    else
      vis = URDF2Gazebo::copyPose(visual->origin);

    coll = coll.GetInverse()*vis;
    
    addTransform(gazebo_visual, coll);
    
    /* set geometry size */                
    
    if (!visual || !visual->geometry)
    {
        logWarn("urdf2gazebo: link(%s) does not have a visual->geometry tag, using default SPHERE with 1mm radius.", link->name.c_str());
        double visualSize[3];
        visualSize[0]=visualSize[1]=visualSize[2]=0.002;
        addKeyValue(gazebo_visual, "scale", values2str(3, visualSize));
        addKeyValue(gazebo_visual, "mesh", "unit_sphere");
    }
    else if (visual->geometry->type == urdf::Geometry::MESH)
    {
        boost::shared_ptr<const urdf::Mesh>  mesh;
        mesh = boost::dynamic_pointer_cast<const urdf::Mesh >(visual->geometry);
        /* set mesh size or scale */
        /*
        if (visual->geometry->isSet["size"])
            addKeyValue(gazebo_visual, "size", values2str(3, mesh->size));        
        else
            addKeyValue(gazebo_visual, "scale", values2str(3, mesh->scale));        
        */
        addKeyValue(gazebo_visual, "scale", vector32str(mesh->scale));

        /* set mesh file */
        if (mesh->filename.empty())
        {
            logError("urdf2gazebo: mesh geometry (%s) was specified but no filename given? using collision type.",link->name.c_str());
            addKeyValue(gazebo_visual, "mesh", "unit_" + collision_type); // reuse collision type if mesh does not specify filename
        }
        else
        {
            /* set mesh file */
            /* expand to get absolute mesh filename */
            std::string fullname = mesh->filename;

            /* resource retriever is ros based
            if (fullname.find("package://") == 0)
            {
              fullname.erase(0, strlen("package://"));
              size_t pos = fullname.find("/");
              if (pos == std::string::npos)
              {
                logFatal("Could not parse package:// format for [%s]",mesh->filename.c_str());
              }

              std::string package = fullname.substr(0, pos);
              fullname.erase(0, pos);
              std::string package_path = ros::package::getPath(package);

              if (package_path.empty())
              {
                logFatal("%s Package[%s] does not exist",mesh->filename.c_str(),package.c_str());
              }

              fullname = package_path + fullname;
            }
            */

            // add mesh filename
            addKeyValue(gazebo_visual, "mesh", fullname);
        }
        
    }
    else
    {
        double visualSize[3];
        std::string visual_geom_type = getGeometryBoundingBox(visual->geometry, visualSize);
        addKeyValue(gazebo_visual, "scale", values2str(3, visualSize));
        addKeyValue(gazebo_visual, "mesh", "unit_" + visual_geom_type);
    }
    
    /* set additional data from extensions */
    insertGazeboExtensionVisual(gazebo_visual,original_reference);

    /* end create visual node */
    geom->LinkEndChild(gazebo_visual);
}

void URDF2Gazebo::walkChildAddNamespace(TiXmlNode* robot_xml,std::string robot_namespace)
{
  TiXmlNode* child = 0;
  child = robot_xml->IterateChildren(child);
  while (child != NULL)
  {
    logDebug("recursively walking gazebo extension for %s --> %d",child->ValueStr().c_str(),(int)child->ValueStr().find(std::string("controller")));
    if (child->ValueStr().find(std::string("controller")) == 0 && child->ValueStr().find(std::string("controller")) != std::string::npos)
    {
      if (child->FirstChildElement("robotNamespace") == NULL)
      {
        logDebug("    adding robotNamespace for %s",child->ValueStr().c_str());
        addKeyValue(child->ToElement(), "robotNamespace", robot_namespace);
      }
      else
      {
        logDebug("    robotNamespace already exists for %s",child->ValueStr().c_str());
      }
    }
    this->walkChildAddNamespace(child,robot_namespace);
    child = robot_xml->IterateChildren(child);
  }
}

bool URDF2Gazebo::convert(TiXmlDocument &urdf_in, TiXmlDocument &gazebo_xml_out, bool enforce_limits, urdf::Vector3 initial_xyz, urdf::Vector3 initial_rpy,std::string model_name , std::string robot_namespace, bool xml_declaration)
{

    // copy model to a string
    std::ostringstream stream_in;
    stream_in << urdf_in;

    /* Create a RobotModel from string */
    boost::shared_ptr<urdf::ModelInterface> robot_model = urdf::parseURDF(stream_in.str().c_str());

    if (!robot_model)
    {
        logError("Unable to load robot model from param server robot_description\n");  
        return false;
    }


#ifndef TO_SDF
    // add xml declaration if needed
    if (xml_declaration)
    {
      TiXmlDeclaration *decl = new TiXmlDeclaration("1.0", "", "");
      gazebo_xml_out.LinkEndChild(decl);  // uncomment if returning old gazebo_xml
    }
#endif
    
    /* create root element and define needed namespaces */
    TiXmlElement *robot = new TiXmlElement("model:physical");
    robot->SetAttribute("xmlns:gazebo", "http://playerstage.sourceforge.net/gazebo/xmlschema/#gz");
    robot->SetAttribute("xmlns:model", "http://playerstage.sourceforge.net/gazebo/xmlschema/#model");
    robot->SetAttribute("xmlns:sensor", "http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor");
    robot->SetAttribute("xmlns:body", "http://playerstage.sourceforge.net/gazebo/xmlschema/#body");
    robot->SetAttribute("xmlns:geom", "http://playerstage.sourceforge.net/gazebo/xmlschema/#geom");
    robot->SetAttribute("xmlns:joint", "http://playerstage.sourceforge.net/gazebo/xmlschema/#joint");
    robot->SetAttribute("xmlns:controller", "http://playerstage.sourceforge.net/gazebo/xmlschema/#controller");
    robot->SetAttribute("xmlns:interface", "http://playerstage.sourceforge.net/gazebo/xmlschema/#interface");
    robot->SetAttribute("xmlns:rendering", "http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering");
    robot->SetAttribute("xmlns:physics", "http://playerstage.sourceforge.net/gazebo/xmlschema/#physics");
    
    // set model name to urdf robot name if not specified
    if (model_name.empty())
      robot->SetAttribute("name", robot_model->getName());
    else
      robot->SetAttribute("name", model_name);
    
    /* set the transform for the whole model to identity */
    addKeyValue(robot, "xyz", vector32str(initial_xyz));
    addKeyValue(robot, "rpy", vector32str(initial_rpy));
    gazebo::math::Pose transform;
    
    /* parse gazebo extension */
    parseGazeboExtension( urdf_in );

    /* start conversion */

    boost::shared_ptr<const urdf::Link> root_link = robot_model->getRoot();
    /* if link connects to parent via fixed joint, lump down and remove link */
    // set reduce_fixed_joints to true will use hinge joints replacements, otherwise, we reduce it down to its parent link recursively
    bool reduce_fixed_joints = true;
    if (reduce_fixed_joints)
      reduceFixedJoints(robot, (boost::const_pointer_cast< urdf::Link >(root_link))); // uncomment to test reduction

    if (root_link->name == "world")
    {
      /* convert all children link */
      for (std::vector<boost::shared_ptr<urdf::Link> >::const_iterator child = root_link->child_links.begin(); child != root_link->child_links.end(); child++)
          convertLink(robot, (*child), transform, enforce_limits, reduce_fixed_joints);
    }
    else
    {
      /* convert, starting from root link */
      convertLink(robot, root_link, transform, enforce_limits, reduce_fixed_joints);
    }
    
    /* find all data item types */
    insertGazeboExtensionRobot(robot);
    
    if (!robot_namespace.empty())
    {
      // traverse all of the: controller::*, add robotNamespace if does not exist already
      logDebug("walking gazebo extension for %s",robot->ValueStr().c_str());
      this->walkChildAddNamespace(robot,robot_namespace);
    }

    std::ostringstream stream_robot;
    stream_robot << *(robot);
    logDebug("\n--------------entire robot------------------\n%s\n--------------------\n",stream_robot.str().c_str());
#ifndef TO_SDF
    gazebo_xml_out.LinkEndChild(robot);  // uncomment if returning old gazebo_xml
#endif


#ifdef TO_SDF
    //
    // step 2 of the two step conversion from URDF --> XML --> SDF
    // now convert the old gazebo xml into sdf
    //
    sdf::SDFPtr robot_sdf(new sdf::SDF());
    if (!sdf::init(robot_sdf))  // FIXME: this step requires that GAZEBO_RESOURCE_PATH be set to where *.sdf are located
    {
      std::cerr << "ERROR: SDF parsing the xml failed" << std::endl;
      return -1;
    }
    deprecated_sdf::initModelString(stream_robot.str(), robot_sdf);
    // write out sdf as a string, then read string into TiXmlDocument gazebo_xml_out
    std::string sdf_string = robot_sdf->ToString();
    logDebug("--------------sdf---------------\n%s\n--------------------\n",sdf_string.c_str());
    gazebo_xml_out.Parse(sdf_string.c_str());
#endif

    return true;
}










