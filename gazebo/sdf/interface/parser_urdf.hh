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

#ifndef _URDF2GAZEBO_HH_
#define _URDF2GAZEBO_HH_

#include <urdfModel/model.h>
#include <urdfModel/link.h>
#include <tinyxml.h>

// #include <cstdio>
// #include <cstdlib>
// #include <cmath>
#include <vector>
#include <string>
#include <map>

#include "sdf/interface/Console.hh"

/// \ingroup sdfUrdf
/// \brief namespace for URDF to SDF parser
namespace urdf2sdf
{
  /// \addtogroup sdf
  /// \{

  typedef boost::sharedPtr<urdf::Collision> CollisionPtr;
  typedef boost::sharedPtr<urdf::Visual> VisualPtr;

  class SDFExtension
  {
    public: SDFExtension()
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
        isFudgeFactor = false;
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
        fudgeFactor = 1;
      };

      SDFExtension(const SDFExtension &ge)
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
        isFudgeFactor = ge.isFudgeFactor;
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
        fudgeFactor = ge.fudgeFactor;
      };

    // for reducing fixed joints and removing links
    public: std::string oldLinkName;
    public: Pose reductionTransform;

    // visual
    public: std::string material;

    // body, default off
    public: bool setStaticFlag;
    public: bool gravity;
    public: bool isDampingFactor;
    public: double dampingFactor;
    public: bool isMaxVel;
    public: double maxVel;
    public: bool isMinDepth;
    public: double minDepth;
    public: bool selfCollide;

    // geom, contact dynamics
    public: bool isMu1, isMu2, isKp, isKd;
    public: double mu1, mu2, kp, kd;
    public: std::string fdir1;
    public: bool isLaserRetro;
    public: double laserRetro;

    // joint, joint limit dynamics
    public: bool isStopCfm;
    public: bool isStopErp;
    public: bool isInitialJointPosition;
    public: bool isFudgeFactor;
    public: double stopCfm, stopErp, initialJointPosition, fudgeFactor;
    public: bool provideFeedback;

    // blobs into body or robot
    public: std::vector<TiXmlElement*> blobs;
  };

  class URDF2SDF
  {
    public: URDF2SDF();
    public: ~URDF2SDF();

    /// parser xml for vector 3
    public: urdf::Vector3 parseVector3(TiXmlNode* key, double scale = 1.0);

    /// convert values to string
    public: std::string values2str(unsigned int count, const double *values);

    /// convert Vector3 to string
    public: std::string vector32str(const urdf::Vector3 vector);

    /// append key value pair to the end of the xml element
    public:void addKeyValue(TiXmlElement *elem, const std::string& key,
               const std::string &value);

    /// append transform (pose) to the end of the xml element
    public: void addTransform(TiXmlElement *elem,
                const::Pose& transform);

    /// print mass for link for debugging
    public: void printMass(boost::sharedPtr<urdf::Link> link);

    /// print mass for link for debugging
    public: void printMass(std::string linkName, dMass mass);

    /// get value from <key value="..."/> pair and return it as string
    public: std::string getKeyValueAsString(TiXmlElement* elem);

    /// things that do not belong in urdf but should be mapped into sdf
    /// @todo: do this using sdf definitions, not hard coded stuff
    public: void parseSDFExtension(TiXmlDocument &urdfXml);

    /// insert extensions into collision geoms
    public: void insertSDFExtensionCollision(TiXmlElement *elem,
                std::string linkName);

    /// insert extensions into visuals
    public: void insertSDFExtensionVisual(TiXmlElement *elem,
                std::string linkName);

    /// insert extensions into links
    public: void insertSDFExtensionLink(TiXmlElement *elem,
                std::string linkName);

    /// insert extensions into joints
    public: void insertSDFExtensionJoint(TiXmlElement *elem,
                std::string jointName);

    /// insert extensions into model
    public: void insertSDFExtensionRobot(TiXmlElement *elem);

    /// list extensions for debugging
    public: void listSDFExtensions();

    /// list extensions for debugging
    public:   void listSDFExtensions(std::string reference);

    /// reduce fixed joints by lumping inertial, visual and
    /// collision elements of the child link into the parent link
    public: void reduceFixedJoints(TiXmlElement *root,
                boost::sharedPtr<urdf::Link> link);

    /// reduce fixed joints:  lump inertial to parent link
    public: void reduceInertialToParent(boost::sharedPtr<urdf::Link> link);

    /// reduce fixed joints:  lump visuals to parent link
    public: void reduceVisualsToParent(boost::sharedPtr<urdf::Link> link);

    /// reduce fixed joints:  lump collisions to parent link
    public: void reduceCollisionsToParent(boost::sharedPtr<urdf::Link> link);

    /// reduce fixed joints:  lump joints to parent link
    public: void reduceJointsToParent(boost::sharedPtr<urdf::Link> link);

    /// reduce fixed joints:  lump visuals when reducing fixed joints
    public: void reduceVisualToParent(boost::sharedPtr<urdf::Link> link,
                std::string groupName,
                boost::sharedPtr<urdf::Visual> visual);

    /// reduce fixed joints:  lump collision when reducing fixed joints
    public: void reduceCollisionToParent(boost::sharedPtr<urdf::Link> link,
                std::string groupName,
                boost::sharedPtr<urdf::Collision> collision);

    /// reduced fixed joints:  apply appropriate updates to urdf
    ///   extensions when doing fixed joint reduction
    public: void reduceSDFExtensionToParent(boost::sharedPtr<urdf::Link> link);

    /// reduced fixed joints:  apply appropriate frame updates
    ///   in urdf extensions when doing fixed joint reduction
    public: void reduceSDFExtensionFrameReplace(SDFExtension* ge,
                boost::sharedPtr<urdf::Link> link);

    /// reduced fixed joints:  apply appropriate frame updates in plugins
    ///   inside urdf extensions when doing fixed joint reduction
    public: void reduceSDFExtensionPluginFrameReplace(
                std::vector<TiXmlElement*>::iterator blobIt,
                boost::sharedPtr<urdf::Link> link, std::string pluginName,
                std::string elementName, Pose reductionTransform);

    /// reduced fixed joints:  apply appropriate frame updates in projector
    ///  inside urdf extensions when doing fixed joint reduction
    public: void reduceSDFExtensionProjectorFrameReplace(
                std::vector<TiXmlElement*>::iterator blobIt,
                boost::sharedPtr<urdf::Link> link);

    /// reduced fixed joints:  apply appropriate frame updates in gripper
    ///   inside urdf extensions when doing fixed joint reduction
    public: void reduceSDFExtensionGripperFrameReplace(
                std::vector<TiXmlElement*>::iterator blobIt,
                boost::sharedPtr<urdf::Link> link);

    /// reduced fixed joints:  apply appropriate frame updates in joint
    ///   inside urdf extensions when doing fixed joint reduction
    public: void reduceSDFExtensionJointFrameReplace(
                std::vector<TiXmlElement*>::iterator blobIt,
                boost::sharedPtr<urdf::Link> link);

    /// reduced fixed joints:  apply appropriate frame updates in urdf
    ///   extensions when doing fixed joint reduction
    public: void reduceSDFExtensionContactSensorFrameReplace(
                std::vector<TiXmlElement*>::iterator blobIt,
                boost::sharedPtr<urdf::Link> link);

    /// reduced fixed joints:  apply transform reduction to extensions
    ///   when doing fixed joint reduction
    public: void reduceSDFExtensionsTransformReduction(SDFExtension* ge);

    /// reduced fixed joints:  apply transform reduction for ray sensors
    ///   in extensions when doing fixed joint reduction
    public: void reduceSDFExtensionSensorTransformReduction(
                std::vector<TiXmlElement*>::iterator blobIt,
                Pose reductionTransform);

    /// reduced fixed joints:  apply transform reduction for projectors in
    ///   extensions when doing fixed joint reduction
    public: void reduceSDFExtensionProjectorTransformReduction(
                std::vector<TiXmlElement*>::iterator blobIt,
                Pose reductionTransform);

    /// reduced fixed joints: transform to parent frame
    public: urdf::Pose  transformToParentFrame(
                urdf::Pose transformInLinkFrame,
                urdf::Pose parentToLinkTransform);

    /// reduced fixed joints: transform to parent frame
    public: Pose  transformToParentFrame(
                Pose transformInLinkFrame,
                urdf::Pose parentToLinkTransform);

    /// reduced fixed joints: transform to parent frame
    public:  Pose  transformToParentFrame(
                 Pose transformInLinkFrame,
                 Pose parentToLinkTransform);

    /// reduced fixed joints: transform to parent frame
    public: Pose  inverseTransformToParentFrame(
                Pose transformInLinkFrame,
                urdf::Pose parentToLinkTransform);

    /// reduced fixed joints: utility to copy between urdf::Pose and
    ///   math::Pose
    public: Pose  copyPose(urdf::Pose pose);

    /// reduced fixed joints: utility to copy between urdf::Pose and
    ///   math::Pose
    public: urdf::Pose  copyPose(Pose pose);

    public: std::string getGeometryBoundingBox(
                boost::sharedPtr<urdf::Geometry> geometry, double *sizeVals);


    /// print collision groups for debugging purposes
    public: void printCollisionGroups(boost::sharedPtr<urdf::Link> link);

    /// create SDF from URDF link
    public: void createSDF(TiXmlElement *root,
                boost::sharedPtr<const urdf::Link> link,
                const Pose &transform);

    /// create SDF Link block based on URDF
    public: void createLink(TiXmlElement *root,
                boost::sharedPtr<const urdf::Link> link,
                Pose &currentTransform);

    /// create collision blocks from urdf collisions
    public: void createCollisions(TiXmlElement* elem,
                boost::sharedPtr<const urdf::Link> link);

    /// create visual blocks from urdf visuals
    public: void createVisuals(TiXmlElement* elem,
                boost::sharedPtr<const urdf::Link> link);

    /// create SDF Inertial block based on URDF
    public: void createInertial(TiXmlElement *elem,
                boost::sharedPtr<const urdf::Link> link);

    /// create SDF Collision block based on URDF
    public: void createCollision(TiXmlElement* elem,
                boost::sharedPtr<const urdf::Link> link,
                boost::sharedPtr<urdf::Collision> collision,
                std::string oldLinkName = std::string(""));

    /// create SDF Visual block based on URDF
    public: void createVisual(TiXmlElement *elem,
                boost::sharedPtr<const urdf::Link> link,
                boost::sharedPtr<urdf::Visual> visual,
                std::string oldLinkName = std::string(""));

    /// create SDF Joint block based on URDF
    public: void createJoint(TiXmlElement *root,
                             boost::sharedPtr<const urdf::Link> link,
                             Pose &currentTransform);

    /// create SDF geometry block based on URDF
    public: void createGeometry(TiXmlElement* elem,
                                boost::sharedPtr<urdf::Geometry> geometry);

    public: TiXmlDocument initModelString(std::string urdfStr);
    public: TiXmlDocument initModelDoc(TiXmlDocument* XmlDoc);
    public: TiXmlDocument initModelFile(std::string filename);
    public: TiXmlDocument initModelString(std::string urdfStr;
    public: TiXmlDocument bool EnforceLimits);

    public: std::map<std::string, std::vector<SDFExtension*> > sdfExtensions;

    private: bool enforceLimits;
    private: bool reduceFixedJoints;
  };
  /// \}
}
#endif
