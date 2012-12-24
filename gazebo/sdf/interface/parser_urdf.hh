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

#include "gazebo/math/Pose.hh"
#include "gazebo/common/Console.hh"

/// \ingroup gazebo_parser
/// \brief namespace for URDF to SDF parser
namespace urdf2gazebo
{
  /// \addtogroup gazebo_parser
  /// \{

  typedef boost::shared_ptr<urdf::Collision> CollisionPtr;
  typedef boost::shared_ptr<urdf::Visual> VisualPtr;
  typedef boost::shared_ptr<urdf::Link> LinkPtr;
  typedef boost::shared_ptr<const urdf::Link> ConstLinkPtr;

  class GazeboExtension
  {
    public:
      // for reducing fixed joints and removing links
      std::string oldLinkName;
      gazebo::math::Pose reductionTransform;

      // visual
      std::string material;

      // body, default off
      bool setStaticFlag;
      bool gravity;
      bool isDampingFactor;
      double dampingFactor;
      bool isMaxVel;
      double maxVel;
      bool isMinDepth;
      double minDepth;
      bool selfCollide;

      // geom, contact dynamics
      bool isMu1, isMu2, isKp, isKd;
      double mu1, mu2, kp, kd;
      std::string fdir1;
      bool isLaserRetro;
      double laserRetro;

      // joint, joint limit dynamics
      bool isStopCfm, isStopErp, isInitialJointPosition, isFudge_factor;
      double stopCfm, stopErp, initialJointPosition, fudge_factor;
      bool provideFeedback;

      // blobs into body or robot
      std::vector<TiXmlElement*> blobs;

      GazeboExtension()
      {
        material.clear();
        setStaticFlag = false;
        gravity = true;
        isDampingFactor = false;
        isMaxVel = false;
        isMinDepth = false;
        fdir1.clear();
        isMu1 = false;
        isMu2 = false;
        isKp = false;
        isKd = false;
        selfCollide = false;
        isLaserRetro = false;
        isStopCfm = false;
        isStopErp = false;
        isInitialJointPosition = false;
        isFudge_factor = false;
        provideFeedback = false;
        blobs.clear();

        dampingFactor = 0;
        maxVel = 0;
        minDepth = 0;
        mu1 = 0;
        mu2 = 0;
        kp = 100000000;
        kd = 1;
        laserRetro = 101;
        stopCfm = 0;
        stopErp = 0.1;
        initialJointPosition = 0;
        fudge_factor = 1;
      };

      GazeboExtension(const GazeboExtension &ge)
      {
        material = ge.material;
        setStaticFlag = ge.setStaticFlag;
        gravity = ge.gravity;
        isDampingFactor = ge.isDampingFactor;
        isMaxVel = ge.isMaxVel;
        isMinDepth = ge.isMinDepth;
        fdir1 = ge.fdir1;
        isMu1 = ge.isMu1;
        isMu2 = ge.isMu2;
        isKp = ge.isKp;
        isKd = ge.isKd;
        selfCollide = ge.selfCollide;
        isLaserRetro = ge.isLaserRetro;
        isStopCfm = ge.isStopCfm;
        isStopErp = ge.isStopErp;
        isInitialJointPosition = ge.isInitialJointPosition;
        isFudge_factor = ge.isFudge_factor;
        provideFeedback = ge.provideFeedback;
        oldLinkName = ge.oldLinkName;
        reductionTransform = ge.reductionTransform;
        blobs = ge.blobs;

        dampingFactor = ge.dampingFactor;
        maxVel = ge.maxVel;
        minDepth = ge.minDepth;
        mu1 = ge.mu1;
        mu2 = ge.mu2;
        kp = ge.kp;
        kd = ge.kd;
        laserRetro = ge.laserRetro;
        stopCfm = ge.stopCfm;
        stopErp = ge.stopErp;
        initialJointPosition = ge.initialJointPosition;
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
      void printMass(LinkPtr link);

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
      void reduceFixedJoints(TiXmlElement *root, LinkPtr link);

      /// reduce fixed joints:  lump inertial to parent link
      void reduceInertialToParent(LinkPtr link);

      /// reduce fixed joints:  lump visuals to parent link
      void reduceVisualsToParent(LinkPtr link);

      /// reduce fixed joints:  lump collisions to parent link
      void reduceCollisionsToParent(LinkPtr link);

      /// reduce fixed joints:  lump joints to parent link
      void reduceJointsToParent(LinkPtr link);

      /// reduce fixed joints:  lump visuals when reducing fixed joints
      void reduceVisualToParent(LinkPtr link,
                                std::string group_name,
                                VisualPtr visual);

      /// reduce fixed joints:  lump collision when reducing fixed joints
      void reduceCollisionToParent(LinkPtr link,
                           std::string group_name,
                           CollisionPtr collision);

      /// reduced fixed joints:  apply appropriate updates to urdf
      ///   extensions when doing fixed joint reduction
      void reduceGazeboExtensionToParent(LinkPtr link);

      /// reduced fixed joints:  apply appropriate frame updates
      ///   in urdf extensions when doing fixed joint reduction
      void reduceGazeboExtensionFrameReplace(GazeboExtension* ge,
                                             LinkPtr link);

      /// reduced fixed joints:  apply appropriate frame updates in plugins
      ///   inside urdf extensions when doing fixed joint reduction
      void reduceGazeboExtensionPluginFrameReplace(
        std::vector<TiXmlElement*>::iterator blob_it,
        LinkPtr link, std::string plugin_name,
        std::string element_name, gazebo::math::Pose _reductionTransform);

      /// reduced fixed joints:  apply appropriate frame updates in projector
      ///  inside urdf extensions when doing fixed joint reduction
      void reduceGazeboExtensionProjectorFrameReplace(
        std::vector<TiXmlElement*>::iterator blob_it,
        LinkPtr link);

      /// reduced fixed joints:  apply appropriate frame updates in gripper
      ///   inside urdf extensions when doing fixed joint reduction
      void reduceGazeboExtensionGripperFrameReplace(
        std::vector<TiXmlElement*>::iterator blob_it,
         LinkPtr link);

      /// reduced fixed joints:  apply appropriate frame updates in joint
      ///   inside urdf extensions when doing fixed joint reduction
      void reduceGazeboExtensionJointFrameReplace(
        std::vector<TiXmlElement*>::iterator blob_it,
         LinkPtr link);

      /// reduced fixed joints:  apply appropriate frame updates in urdf
      ///   extensions when doing fixed joint reduction
      void reduceGazeboExtensionContactSensorFrameReplace(
        std::vector<TiXmlElement*>::iterator blob_it,
         LinkPtr link);

      /// reduced fixed joints:  apply transform reduction to extensions
      ///   when doing fixed joint reduction
      void reduceGazeboExtensionsTransformReduction(GazeboExtension* ge);

      /// reduced fixed joints:  apply transform reduction for ray sensors
      ///   in extensions when doing fixed joint reduction
      void reduceGazeboExtensionSensorTransformReduction(
        std::vector<TiXmlElement*>::iterator blob_it,
        gazebo::math::Pose _reductionTransform);

      /// reduced fixed joints:  apply transform reduction for projectors in
      ///   extensions when doing fixed joint reduction
      void reduceGazeboExtensionProjectorTransformReduction(
        std::vector<TiXmlElement*>::iterator blob_it,
        gazebo::math::Pose _reductionTransform);

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
      void printCollisionGroups(LinkPtr link);

      /// create SDF from URDF link
      void createSDF(TiXmlElement *root,
        ConstLinkPtr link,
        const gazebo::math::Pose &transform);

      /// create SDF Link block based on URDF
      void createLink(TiXmlElement *root,
        ConstLinkPtr link,
        gazebo::math::Pose &currentTransform);

      /// create collision blocks from urdf collisions
      void createCollisions(TiXmlElement* elem,
        ConstLinkPtr link);

      /// create visual blocks from urdf visuals
      void createVisuals(TiXmlElement* elem,
        ConstLinkPtr link);

      /// create SDF Inertial block based on URDF
      void createInertial(TiXmlElement *elem,
        ConstLinkPtr link);

      /// create SDF Collision block based on URDF
      void createCollision(TiXmlElement* elem,
        ConstLinkPtr link,
        CollisionPtr collision,
        std::string _oldLinkName = std::string(""));

      /// create SDF Visual block based on URDF
      void createVisual(TiXmlElement *elem,
        ConstLinkPtr link,
        VisualPtr visual,
        std::string _oldLinkName = std::string(""));

      /// create SDF Joint block based on URDF
      void createJoint(TiXmlElement *root,
        ConstLinkPtr link,
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
