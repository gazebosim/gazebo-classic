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

#ifndef SDF_PARSER_HH
#define SDF_PARSER_HH

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <tinyxml.h>

#include "sdf/interface/Sensor.hh"
#include "sdf/interface/Link.hh"
#include "sdf/interface/Model.hh"
#include "sdf/interface/World.hh"
#include "sdf/interface/Joint.hh"
#include "sdf/interface/Plugin.hh"
#include "math/Pose3d.hh"
#include "math/Vector3.hh"
#include "math/Quatern.hh"

namespace sdf
{
  bool initXml(TiXmlElement *_config, boost::shared_ptr<Sensor> &_sensor);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<SensorType> &_sensor_type);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<ContactSensor> &_contact);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<CameraSensor> &_sensor);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<RaySensor> &_sensor);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<Material> &_material);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<Inertial> &_inertial);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<Collision> &_collision);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<Sphere> &_sphere);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<Box> &_box);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<Cylinder> &_cylinder);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<Mesh> &_mesh);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<Link> &_link);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<Visual> &_visual);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<JointDynamics> &_jointDynamics);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<JointLimits> &_jointLimits);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<Joint> &_joint);

  bool initXml(TiXmlElement *_config, boost::shared_ptr<Joint> &_joint);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<Geometry> &_geom);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<Plugin> &_plugin);

  /// \brief Load Model given a filename
  bool initFile(const std::string &_filename, boost::shared_ptr<Model> &_model);

 /// \brief Load Model from a XML-string
  bool initString(const std::string &_xmlstring, boost::shared_ptr<Model> &_model);

  /// \brief Load Model from TiXMLDocument
  bool initDoc(TiXmlDocument *_xml, boost::shared_ptr<Model> &_model);

  /// \brief Load Model from TiXMLElement
  bool initXml(TiXmlElement *_xml, boost::shared_ptr<Model> &_model);



  /// \brief Load world given a filename
  bool initFile(const std::string &_filename, 
                boost::shared_ptr<World> &_world);

  /// \brief Load world from a XML-string
  bool initString(const std::string &_xmlstring, 
                  boost::shared_ptr<World> &_world);

   /// \brief Load World from TiXMLDocument
  bool initDoc(TiXmlDocument *_xml, boost::shared_ptr<World> &_world);

   /// \brief Load Model from TiXMLElement
  bool initXml(TiXmlElement *_xml, boost::shared_ptr<World> &_world);

  /// scene
  bool initXml(TiXmlElement *_config, boost::shared_ptr<Scene> &_scene);
  
  /// physics
  bool initXml(TiXmlElement *_config, boost::shared_ptr<Physics> &_physics);

  /// OpenDynamicsEngine
  bool initXml(TiXmlElement *_config, boost::shared_ptr<OpenDynamicsEngine> &_open_dynamics_engine);

  bool initXml(TiXmlElement *_config, boost::shared_ptr<Surface> &_surface);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<ODEFriction> &_friction);
  bool initXml(TiXmlElement *_config, boost::shared_ptr<ODEContact> &_contact);

  /// Pose
  bool InitXml(TiXmlElement *_xml, gazebo::math::Pose3d &_pose);

  bool saveXml(const std::string &filename, const boost::shared_ptr<World> &_world);
  bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Scene> &_scene);
  bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Physics> &_physics);
  bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<OpenDynamicsEngine> &_engine);
  bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Model> &_model);
  bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Link> &_link);
  bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Joint> &_joint);
  bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Plugin> &_plugin);
  bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Visual> &_visual);
  bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Collision> &_collision);
  bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Inertial> &_inertial);
  bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Sensor> &_sensor);
  bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Material> &_mat);
  bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Geometry> &_geom);
  bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<CameraSensor> &_camera);
  bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<RaySensor> &_ray);

  bool saveXml(TiXmlElement *_parent, const gazebo::math::Vector3 &_vec);
  bool saveXml(TiXmlElement *_parent, const gazebo::math::Quatern &_rot);
  //bool saveXml(TiXmlElement *_parent, const gazebo::math::Pose3d &_pose);

  //bool saveXml(TiXmlElement *_parent, const ParamT<gazebo::math::Pose3d,true> &_pose);
  //bool saveXml(TiXmlElement *_parent, const ParamT<gazebo::math::Pose3d,false> &_pose);

  bool saveXml(TiXmlElement *_parent, boost::shared_ptr<Surface> &_surface);
  bool saveXml(TiXmlElement *_parent, boost::shared_ptr<ODEFriction> &_friction);
  bool saveXml(TiXmlElement *_parent, boost::shared_ptr<ODEContact> &_contact);


  /// \brief Helper function to get plugins that are children of the passed
  ///        xml node
  bool getPlugins(TiXmlElement *pluginXml, std::map<std::string, 
                  boost::shared_ptr<Plugin> > &plugins);
}

#endif
