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
#ifndef URDF2GAZEBO_HH
#define URDF2GAZEBO_HH

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <vector>
#include <string>

#include <sstream>
#include <map>
#include <vector>

#include <urdf_model/model.h>
#include <urdf_model/link.h>

#include <tinyxml.h>

#include "ode/mass.h"
#include "ode/rotation.h"

#include "math/Pose.hh"

/// \ingroup gazebo_parser
/// \brief namespace for URDF to SDF parser
namespace urdf2gazebo
{
  /// \addtogroup gazebo_parser
  /// \{
  class GazeboExtension
  {
    public:
      // HACK: add for now, since we are reducing fixed joints and removing links
      std::string original_reference;
      gazebo::math::Pose reduction_transform;

      // visual
      std::string material;

      // body, default off
      bool setStaticFlag;
      bool gravity;
      bool is_damping_factor;
      double damping_factor;
      bool is_maxVel;
      double maxVel;
      bool is_minDepth;
      double minDepth;
      bool self_collide;

      // geom, contact dynamics
      bool is_mu1, is_mu2, is_kp, is_kd, is_generate_texture_coord;
      double mu1, mu2, kp, kd;
      std::string fdir1;
      bool generate_texture_coord;
      bool is_laser_retro;
      double laser_retro;

      // joint, joint limit dynamics
      bool is_stop_cfm, is_stop_erp, is_initial_joint_position, is_fudge_factor;
      double stop_cfm, stop_erp, initial_joint_position, fudge_factor;
      bool provideFeedback;

      // blobs into body or robot
      std::vector<TiXmlElement*> blobs;

      GazeboExtension()
      {
        //initialize
        material.clear();
        setStaticFlag = false;
        gravity = false;
        is_damping_factor = false;
        is_maxVel = false;
        is_minDepth = false;
        is_mu1 = false;
        fdir1.clear();
        is_mu2 = false;
        is_kp = false;
        is_kd = false;
        is_generate_texture_coord = false;
        self_collide = false;
        is_laser_retro = false;
        is_stop_cfm = false;
        is_stop_erp = false;
        is_initial_joint_position = false;
        is_fudge_factor = false;
        provideFeedback = false;
        blobs.clear();
      };

      GazeboExtension(const GazeboExtension &ge)
      {
        //initialize
        material = ge.material;
        setStaticFlag = ge.setStaticFlag;
        gravity = ge.gravity;
        is_damping_factor = ge.is_damping_factor;
        is_maxVel = ge.is_maxVel;
        is_minDepth = ge.is_minDepth;
        is_mu1 = ge.is_mu1;
        is_mu2 = ge.is_mu2;
        is_kp = ge.is_kp;
        is_kd = ge.is_kd;
        is_generate_texture_coord = ge.is_generate_texture_coord;
        self_collide = ge.self_collide;
        is_laser_retro = ge.is_laser_retro;
        is_stop_cfm = ge.is_stop_cfm;
        is_stop_erp = ge.is_stop_erp;
        is_initial_joint_position = ge.is_initial_joint_position;
        is_fudge_factor = ge.is_fudge_factor;
        provideFeedback = ge.provideFeedback;
        original_reference = ge.original_reference;
        reduction_transform = ge.reduction_transform;
        blobs = ge.blobs;
      };
  };

  class URDF2Gazebo
  {
    public:
      URDF2Gazebo();
      ~URDF2Gazebo();

      /// parser xml for vector 3
      urdf::Vector3 parseVector3(TiXmlNode* key, double scale = 1.0);

      /// lump visuals when reducing fixed joints
      void lumpVisual(boost::shared_ptr<urdf::Link> link, std::string group_name, boost::shared_ptr<urdf::Visual> visual);

      /// lump collision when reducing fixed joints
      void lumpCollision(boost::shared_ptr<urdf::Link> link, std::string group_name, boost::shared_ptr<urdf::Collision> collision);

      /// convert values to string
      std::string values2str(unsigned int count, const double *values, double (*conv)(double));

      /// convert Vector3 to string
      std::string vector32str(const urdf::Vector3 vector, double (*conv)(double));

      /// append key value pair to the end of the xml element
      void addKeyValue(TiXmlElement *elem, const std::string& key, const std::string &value);

      /// append transform (pose) to the end of the xml element
      void addTransform(TiXmlElement *elem, const::gazebo::math::Pose& transform);

      /// print mass for link for debugging
      void printMass(boost::shared_ptr<urdf::Link> link);

      /// print mass for link for debugging
      void printMass(std::string link_name, dMass mass);

      /// get value from <key value="..."/> pair and return it as string
      std::string getKeyValueAsString(TiXmlElement* elem);

      /// deal with extensions (things that do not belong in urdf but should be mapped into sdf
      void parseGazeboExtension(TiXmlDocument &urdf_xml);

      /// create collision blocks from urdf collisions
      void createCollisions(TiXmlElement* elem, boost::shared_ptr<const urdf::Link> link);

      /// create visual blocks from urdf visuals
      void createVisuals(TiXmlElement* elem, boost::shared_ptr<const urdf::Link> link);

      /// insert extensions into collision geoms
      void insertGazeboExtensionCollision(TiXmlElement *elem,std::string link_name);

      /// insert extensions into visuals
      void insertGazeboExtensionVisual(TiXmlElement *elem,std::string link_name);

      /// insert extensions into links
      void insertGazeboExtensionLink(TiXmlElement *elem,std::string link_name);

      /// insert extensions into joints
      void insertGazeboExtensionJoint(TiXmlElement *elem,std::string joint_name);

      /// insert extensions into model
      void insertGazeboExtensionRobot(TiXmlElement *elem);

      /// list extensions for debugging
      void listGazeboExtensions();

      /// list extensions for debugging
      void listGazeboExtensions(std::string reference);

      /// apply appropriate updates to urdf extensions when doing fixed joint reduction
      void reduceGazeboExtensionToParent(boost::shared_ptr<urdf::Link> link);

      /// apply appropriate frame updates in urdf extensions when doing fixed joint reduction
      void updateGazeboExtensionFrameReplace(GazeboExtension* ge, boost::shared_ptr<urdf::Link> link, std::string new_link_name);

      /// apply appropriate updates to extensions when doing fixed joint reduction
      void updateGazeboExtensionBlobsReductionTransform(GazeboExtension* ge);

      urdf::Pose  transformToParentFrame(urdf::Pose transform_in_link_frame, urdf::Pose parent_to_link_transform);
      gazebo::math::Pose  transformToParentFrame(gazebo::math::Pose transform_in_link_frame, urdf::Pose parent_to_link_transform);
      gazebo::math::Pose  transformToParentFrame(gazebo::math::Pose transform_in_link_frame, gazebo::math::Pose parent_to_link_transform);
      gazebo::math::Pose  inverseTransformToParentFrame(gazebo::math::Pose transform_in_link_frame, urdf::Pose parent_to_link_transform);
      gazebo::math::Pose  copyPose(urdf::Pose pose);
      urdf::Pose  copyPose(gazebo::math::Pose pose);

      void createGeometry(TiXmlElement* elem, boost::shared_ptr<urdf::Geometry> geometry);
      
      std::string getGeometryBoundingBox(boost::shared_ptr<urdf::Geometry> geometry, double *sizeVals);

      /// reduce fixed joints by lumping inertial, visual, collision elements of the child link
      /// into the parent link
      void reduceFixedJoints(TiXmlElement *root, boost::shared_ptr<urdf::Link> link);

      /// print collision groups for debugging purposes
      void printCollisionGroups(boost::shared_ptr<urdf::Link> link);

      /// do the heavy lifting of conversion
      void initLink(TiXmlElement *root, boost::shared_ptr<const urdf::Link> link, const gazebo::math::Pose &transform);
      void createLink(TiXmlElement *root, boost::shared_ptr<const urdf::Link> link,
                      gazebo::math::Pose &currentTransform);

      void createInertial(TiXmlElement *elem, boost::shared_ptr<const urdf::Link> link);

      void createCollision(TiXmlElement* elem, boost::shared_ptr<const urdf::Link> link, boost::shared_ptr<urdf::Collision> collision,
                           std::string original_reference = std::string(""));

      void createVisual(TiXmlElement *elem, boost::shared_ptr<const urdf::Link> link, boost::shared_ptr<urdf::Visual> visual,
                        std::string original_reference = std::string(""));

      void createJoint(TiXmlElement *root, boost::shared_ptr<const urdf::Link> link, gazebo::math::Pose &currentTransform);

      TiXmlDocument initModelString(std::string urdf_str);
      TiXmlDocument initModelDoc(TiXmlDocument* _xmlDoc);
      TiXmlDocument  initModelFile(std::string filename);
      TiXmlDocument initModelString(std::string urdf_str, bool _enforce_limits, 
                   urdf::Vector3 _initial_xyz, urdf::Vector3 _initial_rpy,std::string _model_name = std::string(),
                   std::string _robot_namespace = std::string(), bool _xml_declaration = false);

      void walkChildAddNamespace(TiXmlNode* robot_xml);

      std::map<std::string, std::vector<GazeboExtension*> > gazebo_extensions_;

      private: bool enforce_limits, xml_declaration;
      private: bool reduce_fixed_joints;
      private: std::string robot_namespace;
      private: std::string model_name;
      private: urdf::Vector3 initial_xyz, initial_rpy;

  };
  /// \}
}

#endif
