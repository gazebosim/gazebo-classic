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

/* Author: Nate Koenig, John Hsu */

#ifndef URDF_PARSER_HH
#define URDF_PARSER_HH

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <libxml/parser.h>

#include "sdf/interface/Sensor.hh"
#include "sdf/interface/Link.hh"
#include "sdf/interface/Model.hh"
#include "sdf/interface/World.hh"
#include "sdf/interface/Joint.hh"
#include "sdf/interface/Plugin.hh"
#include "math/Pose.hh"
#include "sdf/parser_deprecated/controller.hh"

namespace sdf
{
  bool initXml(xmlNodePtr _config, boost::shared_ptr<Sensor> &_sensor);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<SensorType> &_sensor_type);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<ContactSensor> &_contact);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<CameraSensor> &_sensor);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<RaySensor> &_sensor);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<Material> &_material);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<Inertial> &_inertial);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<Collision> &_collision);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<Sphere> &_sphere);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<Box> &_box);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<Cylinder> &_cylinder);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<Mesh> &_mesh);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<Link> &_link);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<Visual> &_visual);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<JointDynamics> &_jointDynamics);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<JointLimits> &_jointLimits);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<Joint> &_joint);

  bool initXml(xmlNodePtr _config, boost::shared_ptr<Joint> &_joint);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<Geometry> &_geom);
  bool initXml(xmlNodePtr _config, boost::shared_ptr<Controller> &_controller);

  /// \brief Load Model given a filename
  bool initFile(const std::string &_filename, boost::shared_ptr<Model> &_model);

 /// \brief Load Model from a XML-string
  bool initString(const std::string &_xmlstring, boost::shared_ptr<Model> &_model);

  /// \brief Load Model from TiXMLDocument
  bool initDoc(xmlDocPtr _xml, boost::shared_ptr<Model> &_model);

  /// \brief Load Model from TiXMLElement
  bool initXml(xmlNodePtr _xml, boost::shared_ptr<Model> &_model);


  /// \brief Load world given a filename
  bool initFile(const std::string &_filename, boost::shared_ptr<World> &_world);

  /// \brief Load world from a XML-string
  bool initString(const std::string &_xmlstring, 
                  boost::shared_ptr<World> &_world);

   /// \brief Load World from TiXMLDocument
  bool initDoc(xmlDocPtr _xml, boost::shared_ptr<World> &_world);

   /// \brief Load Model from TiXMLElement
  bool initXml(xmlNodePtr _xml, boost::shared_ptr<World> &_world);

  /// scene
  bool initXml(xmlNodePtr _config, boost::shared_ptr<Scene> &_scene);
  
  /// physics
  bool initXml(xmlNodePtr _config, boost::shared_ptr<Physics> &_physics);

  /// OpenDynamicsEngine
  bool initXml(xmlNodePtr _config, boost::shared_ptr<OpenDynamicsEngine> &_open_dynamics_engine);

  /// Pose
  bool InitXml(xmlNodePtr _xml, gazebo::math::Pose &_pose);

/*
  bool saveXml(const std::string &filename, const boost::shared_ptr<World> &_world);
  bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Scene> &_scene);
  bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Physics> &_physics);
  bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<OpenDynamicsEngine> &_engine);
  bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Model> &_model);
  bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Link> &_link);
  bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Joint> &_joint);
  bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Plugin> &_plugin);
  bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Visual> &_visual);
  bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Collision> &_collision);
  bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Inertial> &_inertial);
  bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Sensor> &_sensor);
  bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Material> &_mat);
  bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<Geometry> &_geom);
  bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<CameraSensor> &_camera);
  bool saveXml(xmlNodePtr _parent, const boost::shared_ptr<RaySensor> &_ray);

  bool saveXml(xmlNodePtr _parent, const Vector3 &_vec);
  bool saveXml(xmlNodePtr _parent, const Rotation &_rot);
  bool saveXml(xmlNodePtr _parent, const ParamT<Pose,true> &_pose);
  bool saveXml(xmlNodePtr _parent, const ParamT<Pose,false> &_pose);
*/

  bool getPlugins(xmlNodePtr pluginXml, std::map<std::string, 
                  boost::shared_ptr<Plugin> > &plugins);

  ////////////////////////////////////////////////////////////////////////////
  // Get a child based on a name. Returns null if not found
  xmlNodePtr FirstChildElement( xmlNodePtr node, const std::string &name)
  {
    xmlNodePtr tmp;
    for (tmp = node->xmlChildrenNode; tmp != NULL; tmp = tmp->next )
      if (tmp->name && name == (const char*)tmp->name) break;

    return tmp;
  }
  xmlNodePtr NextSiblingElement( xmlNodePtr node, const std::string &name)
  {
    xmlNodePtr tmp;
    for (tmp = node->next; tmp != NULL; tmp = tmp->next )
      if (tmp->name && name == (const char*)tmp->name) break;

    return tmp;
  }

  void PreParser(const std::string &fname, std::string &output)
  {
    std::ifstream ifs(fname.c_str(), std::ios::in);
    std::string line;

    while (ifs.good())
    {
      std::getline(ifs, line);
      boost::trim(line);

      if (boost::find_first(line,"<include"))
      {
        int start = line.find("filename=");
        start += strlen("filename=") + 1;
        int end = line.find_first_of("'\"", start);
        std::string fname2 = line.substr(start, end-start);
        PreParser(fname2, output);
      }
      else
        output += line + "\n";
    }

    ifs.close();
  }


}

#endif
