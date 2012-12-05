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
#ifndef URDF2GAZEBO_HH
#define URDF2GAZEBO_HH

#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <tinyxml.h>

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <map>

#include "ode/mass.h"
#include "ode/rotation.h"

#include "math/Pose.hh"
#include "common/Console.hh"

/// \ingroup gazebo_parser
/// \brief namespace for URDF to SDF parser
namespace urdf2gazebo
{
  /// \addtogroup gazebo_parser
  /// \{

  typedef boost::shared_ptr<urdf::Collision> CollisionPtr;
  typedef boost::shared_ptr<urdf::Visual> VisualPtr;

  class GazeboExtension
  {
    public:
      // for reducing fixed joints and removing links
      std::string old_link_name;
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
      bool is_mu1, is_mu2, is_kp, is_kd;
      double mu1, mu2, kp, kd;
      std::string fdir1;
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
        material.clear();
        setStaticFlag = false;
        gravity = true;
        is_damping_factor = false;
        is_maxVel = false;
        is_minDepth = false;
        fdir1.clear();
        is_mu1 = false;
        is_mu2 = false;
        is_kp = false;
        is_kd = false;
        self_collide = false;
        is_laser_retro = false;
        is_stop_cfm = false;
        is_stop_erp = false;
        is_initial_joint_position = false;
        is_fudge_factor = false;
        provideFeedback = false;
        blobs.clear();

        damping_factor = 0;
        maxVel = 0;
        minDepth = 0;
        mu1 = 0;
        mu2 = 0;
        kp = 100000000;
        kd = 1;
        laser_retro = 101;
        stop_cfm = 0;
        stop_erp = 0.1;
        initial_joint_position = 0;
        fudge_factor = 1;
      };

      GazeboExtension(const GazeboExtension &ge)
      {
        material = ge.material;
        setStaticFlag = ge.setStaticFlag;
        gravity = ge.gravity;
        is_damping_factor = ge.is_damping_factor;
        is_maxVel = ge.is_maxVel;
        is_minDepth = ge.is_minDepth;
        fdir1 = ge.fdir1;
        is_mu1 = ge.is_mu1;
        is_mu2 = ge.is_mu2;
        is_kp = ge.is_kp;
        is_kd = ge.is_kd;
        self_collide = ge.self_collide;
        is_laser_retro = ge.is_laser_retro;
        is_stop_cfm = ge.is_stop_cfm;
        is_stop_erp = ge.is_stop_erp;
        is_initial_joint_position = ge.is_initial_joint_position;
        is_fudge_factor = ge.is_fudge_factor;
        provideFeedback = ge.provideFeedback;
        old_link_name = ge.old_link_name;
        reduction_transform = ge.reduction_transform;
        blobs = ge.blobs;

        damping_factor = ge.damping_factor;
        maxVel = ge.maxVel;
        minDepth = ge.minDepth;
        mu1 = ge.mu1;
        mu2 = ge.mu2;
        kp = ge.kp;
        kd = ge.kd;
        laser_retro = ge.laser_retro;
        stop_cfm = ge.stop_cfm;
        stop_erp = ge.stop_erp;
        initial_joint_position = ge.initial_joint_position;
        fudge_factor = ge.fudge_factor;
      };
  };

  class URDF2Gazebo
  {
    public:
      URDF2Gazebo();
      ~URDF2Gazebo();

      /// parser xml for vector 3
      urdf::Vector3 parseVector3(TiXmlNode* key, double scale = 1.0);

      /// convert values to string
      std::string values2str(unsigned int count, const double *values);

      /// convert Vector3 to string
      std::string vector32str(const urdf::Vector3 vector);

      /// append key value pair to the end of the xml element
      void addKeyValue(TiXmlElement *elem, const std::string& key,
                       const std::string &value);

      /// append transform (pose) to the end of the xml element
      void addTransform(TiXmlElement *elem,
                        const::gazebo::math::Pose& transform);

      /// print mass for link for debugging
      void printMass(boost::shared_ptr<urdf::Link> link);

      /// print mass for link for debugging
      void printMass(std::string link_name, dMass mass);

      /// get value from <key value="..."/> pair and return it as string
      std::string getKeyValueAsString(TiXmlElement* elem);

      /// things that do not belong in urdf but should be mapped into sdf
      /// @todo: do this using sdf definitions, not hard coded stuff
      void parseGazeboExtension(TiXmlDocument &urdf_xml);

      /// insert extensions into collision geoms
      void insertGazeboExtensionCollision(TiXmlElement *elem,
                                          std::string link_name);

      /// insert extensions into visuals
      void insertGazeboExtensionVisual(TiXmlElement *elem,
                                       std::string link_name);

      /// insert extensions into links
      void insertGazeboExtensionLink(TiXmlElement *elem,
                                     std::string link_name);

      /// insert extensions into joints
      void insertGazeboExtensionJoint(TiXmlElement *elem,
                                      std::string joint_name);

      /// insert extensions into model
      void insertGazeboExtensionRobot(TiXmlElement *elem);

      /// list extensions for debugging
      void listGazeboExtensions();

      /// list extensions for debugging
      void listGazeboExtensions(std::string reference);

      /// reduce fixed joints by lumping inertial, visual and
      // collision elements of the child link into the parent link
      void reduceFixedJoints(TiXmlElement *root,
                             boost::shared_ptr<urdf::Link> link);

      /// reduce fixed joints:  lump inertial to parent link
      void reduceInertialToParent(boost::shared_ptr<urdf::Link> link);

      /// reduce fixed joints:  lump visuals to parent link
      void reduceVisualsToParent(boost::shared_ptr<urdf::Link> link);

      /// reduce fixed joints:  lump collisions to parent link
      void reduceCollisionsToParent(boost::shared_ptr<urdf::Link> link);

      /// reduce fixed joints:  lump joints to parent link
      void reduceJointsToParent(boost::shared_ptr<urdf::Link> link);

      /// reduce fixed joints:  lump visuals when reducing fixed joints
      void reduceVisualToParent(boost::shared_ptr<urdf::Link> link,
                                std::string group_name,
                                boost::shared_ptr<urdf::Visual> visual);

      /// reduce fixed joints:  lump collision when reducing fixed joints
      void reduceCollisionToParent(boost::shared_ptr<urdf::Link> link,
                           std::string group_name,
                           boost::shared_ptr<urdf::Collision> collision);

      /// reduced fixed joints:  apply appropriate updates to urdf
      ///   extensions when doing fixed joint reduction
      void reduceGazeboExtensionToParent(boost::shared_ptr<urdf::Link> link);

      /// reduced fixed joints:  apply appropriate frame updates
      ///   in urdf extensions when doing fixed joint reduction
      void reduceGazeboExtensionFrameReplace(GazeboExtension* ge,
             boost::shared_ptr<urdf::Link> link);

      /// reduced fixed joints:  apply appropriate frame updates in plugins
      ///   inside urdf extensions when doing fixed joint reduction
      void reduceGazeboExtensionPluginFrameReplace(
        std::vector<TiXmlElement*>::iterator blob_it,
        boost::shared_ptr<urdf::Link> link, std::string plugin_name,
        std::string element_name, gazebo::math::Pose reduction_transform);

      /// reduced fixed joints:  apply appropriate frame updates in projector
      ///  inside urdf extensions when doing fixed joint reduction
      void reduceGazeboExtensionProjectorFrameReplace(
        std::vector<TiXmlElement*>::iterator blob_it,
        boost::shared_ptr<urdf::Link> link);

      /// reduced fixed joints:  apply appropriate frame updates in gripper
      ///   inside urdf extensions when doing fixed joint reduction
      void reduceGazeboExtensionGripperFrameReplace(
        std::vector<TiXmlElement*>::iterator blob_it,
         boost::shared_ptr<urdf::Link> link);

      /// reduced fixed joints:  apply appropriate frame updates in joint
      ///   inside urdf extensions when doing fixed joint reduction
      void reduceGazeboExtensionJointFrameReplace(
        std::vector<TiXmlElement*>::iterator blob_it,
         boost::shared_ptr<urdf::Link> link);

      /// reduced fixed joints:  apply appropriate frame updates in urdf
      ///   extensions when doing fixed joint reduction
      void reduceGazeboExtensionContactSensorFrameReplace(
        std::vector<TiXmlElement*>::iterator blob_it,
         boost::shared_ptr<urdf::Link> link);

      /// reduced fixed joints:  apply transform reduction to extensions
      ///   when doing fixed joint reduction
      void reduceGazeboExtensionsTransformReduction(GazeboExtension* ge);

      /// reduced fixed joints:  apply transform reduction for ray sensors
      ///   in extensions when doing fixed joint reduction
      void reduceGazeboExtensionSensorTransformReduction(
        std::vector<TiXmlElement*>::iterator blob_it,
        gazebo::math::Pose reduction_transform);

      /// reduced fixed joints:  apply transform reduction for projectors in
      ///   extensions when doing fixed joint reduction
      void reduceGazeboExtensionProjectorTransformReduction(
        std::vector<TiXmlElement*>::iterator blob_it,
        gazebo::math::Pose reduction_transform);

      /// reduced fixed joints: transform to parent frame
      urdf::Pose  transformToParentFrame(
        urdf::Pose transform_in_link_frame,
        urdf::Pose parent_to_link_transform);
      /// reduced fixed joints: transform to parent frame
      gazebo::math::Pose  transformToParentFrame(
        gazebo::math::Pose transform_in_link_frame,
        urdf::Pose parent_to_link_transform);
      /// reduced fixed joints: transform to parent frame
      gazebo::math::Pose  transformToParentFrame(
        gazebo::math::Pose transform_in_link_frame,
        gazebo::math::Pose parent_to_link_transform);
      /// reduced fixed joints: transform to parent frame
      gazebo::math::Pose  inverseTransformToParentFrame(
        gazebo::math::Pose transform_in_link_frame,
        urdf::Pose parent_to_link_transform);
      /// reduced fixed joints: utility to copy between urdf::Pose and
      ///   math::Pose
      gazebo::math::Pose  copyPose(urdf::Pose pose);
      /// reduced fixed joints: utility to copy between urdf::Pose and
      ///   math::Pose
      urdf::Pose  copyPose(gazebo::math::Pose pose);

      std::string getGeometryBoundingBox(
        boost::shared_ptr<urdf::Geometry> geometry, double *sizeVals);


      /// print collision groups for debugging purposes
      void printCollisionGroups(boost::shared_ptr<urdf::Link> link);

      /// create SDF from URDF link
      void createSDF(TiXmlElement *root,
        boost::shared_ptr<const urdf::Link> link,
        const gazebo::math::Pose &transform);

      /// create SDF Link block based on URDF
      void createLink(TiXmlElement *root,
        boost::shared_ptr<const urdf::Link> link,
        gazebo::math::Pose &currentTransform);

      /// create collision blocks from urdf collisions
      void createCollisions(TiXmlElement* elem,
        boost::shared_ptr<const urdf::Link> link);

      /// create visual blocks from urdf visuals
      void createVisuals(TiXmlElement* elem,
        boost::shared_ptr<const urdf::Link> link);

      /// create SDF Inertial block based on URDF
      void createInertial(TiXmlElement *elem,
        boost::shared_ptr<const urdf::Link> link);

      /// create SDF Collision block based on URDF
      void createCollision(TiXmlElement* elem,
        boost::shared_ptr<const urdf::Link> link,
        boost::shared_ptr<urdf::Collision> collision,
        std::string old_link_name = std::string(""));

      /// create SDF Visual block based on URDF
      void createVisual(TiXmlElement *elem,
        boost::shared_ptr<const urdf::Link> link,
        boost::shared_ptr<urdf::Visual> visual,
        std::string old_link_name = std::string(""));

      /// create SDF Joint block based on URDF
      void createJoint(TiXmlElement *root,
        boost::shared_ptr<const urdf::Link> link,
        gazebo::math::Pose &currentTransform);

      /// create SDF geometry block based on URDF
      void createGeometry(TiXmlElement* elem,
        boost::shared_ptr<urdf::Geometry> geometry);

      TiXmlDocument initModelString(std::string urdf_str);
      TiXmlDocument initModelDoc(TiXmlDocument* _xmlDoc);
      TiXmlDocument initModelFile(std::string filename);
      TiXmlDocument initModelString(std::string urdf_str, bool _enforce_limits);

      std::map<std::string, std::vector<GazeboExtension*> > gazebo_extensions_;

      private: bool enforce_limits;
      private: bool reduce_fixed_joints;
  };
  /// \}
}

#endif
