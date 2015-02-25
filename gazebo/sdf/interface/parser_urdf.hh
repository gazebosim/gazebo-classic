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

  typedef boost::shared_ptr<urdf::Collision> UrdfCollisionPtr;
  typedef boost::shared_ptr<urdf::Visual> UrdfVisualPtr;
  typedef boost::shared_ptr<urdf::Link> UrdfLinkPtr;
  typedef boost::shared_ptr<const urdf::Link> ConstUrdfLinkPtr;

  /// \class A class for holding gazebo extension elements in urdf
  class GazeboExtension
  {
    private: GazeboExtension()
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
      cfmDamping = false;
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

    private: GazeboExtension(const GazeboExtension &ge)
             : material(ge.material), fdir1(ge.fdir1),
             oldLinkName(ge.oldLinkName),
             reductionTransform(ge.reductionTransform),
             blobs(ge.blobs)
    {
      setStaticFlag = ge.setStaticFlag;
      gravity = ge.gravity;
      isDampingFactor = ge.isDampingFactor;
      isMaxVel = ge.isMaxVel;
      isMinDepth = ge.isMinDepth;
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
      cfmDamping = ge.cfmDamping;
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

    // visual
    private: std::string material;

    private: std::string fdir1;

    // for reducing fixed joints and removing links
    private: std::string oldLinkName;
    private: gazebo::math::Pose reductionTransform;

    // blobs into body or robot
    private: std::vector<TiXmlElement*> blobs;

    // body, default off
    private: bool setStaticFlag;
    private: bool gravity;
    private: bool isDampingFactor;
    private: double dampingFactor;
    private: bool isMaxVel;
    private: double maxVel;
    private: bool isMinDepth;
    private: double minDepth;
    private: bool selfCollide;

    // geom, contact dynamics
    private: bool isMu1, isMu2, isKp, isKd;
    private: double mu1, mu2, kp, kd;
    private: bool isLaserRetro;
    private: double laserRetro;

    // joint, joint limit dynamics
    private: bool isStopCfm, isStopErp, isInitialJointPosition, isFudgeFactor;
    private: double stopCfm, stopErp, initialJointPosition, fudgeFactor;
    private: bool provideFeedback;
    private: bool cfmDamping;

    friend class URDF2Gazebo;
  };

  class URDF2Gazebo
  {
    /// \brief constructor
    public: URDF2Gazebo();

    /// \brief destructor
    public: ~URDF2Gazebo();

    /// \brief convert urdf xml document string to sdf xml document
    /// \param[in] _xmlDoc a tinyxml document containing the urdf model
    /// \return a tinyxml document containing sdf of the model
    public: TiXmlDocument InitModelDoc(TiXmlDocument* _xmlDoc);

    /// \brief convert urdf file to sdf xml document
    /// \param[in] _urdfStr a string containing filename of the urdf model
    /// \return a tinyxml document containing sdf of the model
    public: TiXmlDocument InitModelFile(const std::string &_filename);

    /// \brief convert urdf string to sdf xml document, with option to enforce
    /// limits.
    /// \param[in] _urdfStr a string containing model urdf
    /// \param[in] _enforceLimits option to enforce joint limits
    /// \return a tinyxml document containing sdf of the model
    public: TiXmlDocument InitModelString(const std::string &_urdfStr,
                                          bool _enforceLimits = true);

    /// \brief parser xml string into urdf::Vector3
    /// \param[in] _key XML key where vector3 value might be
    /// \param[in] _scale scalar scale for the vector3
    /// \return a urdf::Vector3
    private: urdf::Vector3 ParseVector3(TiXmlNode* _key, double _scale = 1.0);

    /// \brief convert values to string
    /// \param[in] _count number of values in _values array
    /// \param[in] _values array of double values
    /// \return a string
    private: std::string Values2str(unsigned int _count, const double *_values);

    /// \brief convert Vector3 to string
    /// \param[in] _vector a urdf::Vector3
    /// \return a string
    private: std::string Vector32Str(const urdf::Vector3 _vector);

    /// \brief append key value pair to the end of the xml element
    /// \param[in] _elem pointer to xml element
    /// \param[in] _key string containing key to add to xml element
    /// \param[in] _value string containing value for the key added
    private: void AddKeyValue(TiXmlElement *_elem, const std::string &_key,
                     const std::string &_value);

    /// append transform (pose) to the end of the xml element
    private: void AddTransform(TiXmlElement *_elem,
                      const::gazebo::math::Pose& _transform);

    /// print mass for link for debugging
    private: void PrintMass(UrdfLinkPtr _link);

    /// print mass for link for debugging
    private: void PrintMass(const std::string &_linkName, dMass _mass);

    /// get value from <key value="..."/> pair and return it as string
    private: std::string GetKeyValueAsString(TiXmlElement* _elem);

    /// things that do not belong in urdf but should be mapped into sdf
    /// @todo: do this using sdf definitions, not hard coded stuff
    private: void ParseGazeboExtension(TiXmlDocument &_urdfXml);

    /// insert extensions into collision geoms
    private: void InsertGazeboExtensionCollision(TiXmlElement *_elem,
                                        const std::string &_linkName);

    /// insert extensions into visuals
    private: void InsertGazeboExtensionVisual(TiXmlElement *_elem,
                                     const std::string &_linkName);

    /// insert extensions into links
    private: void InsertGazeboExtensionLink(TiXmlElement *_elem,
                                   const std::string &_linkName);

    /// insert extensions into joints
    private: void InsertGazeboExtensionJoint(TiXmlElement *_elem,
                                    const std::string &_jointName);

    /// insert extensions into model
    private: void InsertGazeboExtensionRobot(TiXmlElement *_elem);

    /// list extensions for debugging
    private: void ListGazeboExtensions();

    /// list extensions for debugging
    private: void ListGazeboExtensions(const std::string &_reference);

    /// reduce fixed joints by lumping inertial, visual and
    // collision elements of the child link into the parent link
    private: void ReduceFixedJoints(TiXmlElement *_root, UrdfLinkPtr _link);

    /// reduce fixed joints:  lump inertial to parent link
    private: void ReduceInertialToParent(UrdfLinkPtr _link);

    /// reduce fixed joints:  lump visuals to parent link
    private: void ReduceVisualsToParent(UrdfLinkPtr _link);

    /// reduce fixed joints:  lump collisions to parent link
    private: void ReduceCollisionsToParent(UrdfLinkPtr _link);

    /// reduce fixed joints:  lump joints to parent link
    private: void ReduceJointsToParent(UrdfLinkPtr _link);

    /// reduce fixed joints:  lump visuals when reducing fixed joints
    private: void ReduceVisualToParent(UrdfLinkPtr _link,
                              const std::string &_groupName,
                              UrdfVisualPtr _visual);

    /// reduce fixed joints:  lump collision when reducing fixed joints
    private: void ReduceCollisionToParent(UrdfLinkPtr _link,
                         const std::string &_groupName,
                         UrdfCollisionPtr _collision);

    /// \brief reduced fixed joints:  apply appropriate updates to urdf
    ///   extensions when doing fixed joint reduction
    ///
    /// Take the link's existing list of gazebo extensions, transfer them
    /// into parent link.  Along the way, update local transforms by adding
    /// the additional transform to parent.  Also, look through all
    /// referenced link names with plugins and update references to current
    /// link to the parent link. (ReduceGazeboExtensionFrameReplace())
    ///
    /// \param[in] _link pointer to urdf link, its extensions will be reduced
    private: void ReduceGazeboExtensionToParent(UrdfLinkPtr _link);

    /// reduced fixed joints:  apply appropriate frame updates
    ///   in urdf extensions when doing fixed joint reduction
    private: void ReduceGazeboExtensionFrameReplace(GazeboExtension* _ge,
                                           UrdfLinkPtr _link);

    /// reduced fixed joints:  apply appropriate frame updates in plugins
    ///   inside urdf extensions when doing fixed joint reduction
    private: void ReduceGazeboExtensionPluginFrameReplace(
      std::vector<TiXmlElement*>::iterator _blobIt,
      UrdfLinkPtr _link, const std::string &_pluginName,
      const std::string &_elementName, gazebo::math::Pose _reductionTransform);

    /// reduced fixed joints:  apply appropriate frame updates in projector
    ///  inside urdf extensions when doing fixed joint reduction
    private: void ReduceGazeboExtensionProjectorFrameReplace(
      std::vector<TiXmlElement*>::iterator _blobIt,
      UrdfLinkPtr _link);

    /// reduced fixed joints:  apply appropriate frame updates in gripper
    ///   inside urdf extensions when doing fixed joint reduction
    private: void ReduceGazeboExtensionGripperFrameReplace(
      std::vector<TiXmlElement*>::iterator _blobIt,
       UrdfLinkPtr _link);

    /// reduced fixed joints:  apply appropriate frame updates in joint
    ///   inside urdf extensions when doing fixed joint reduction
    private: void ReduceGazeboExtensionJointFrameReplace(
      std::vector<TiXmlElement*>::iterator _blobIt,
       UrdfLinkPtr _link);

    /// reduced fixed joints:  apply appropriate frame updates in urdf
    ///   extensions when doing fixed joint reduction
    private: void ReduceGazeboExtensionContactSensorFrameReplace(
      std::vector<TiXmlElement*>::iterator _blobIt,
       UrdfLinkPtr _link);

    /// reduced fixed joints:  apply transform reduction to extensions
    ///   when doing fixed joint reduction
    private: void ReduceGazeboExtensionsTransform(GazeboExtension* _ge);

    /// reduced fixed joints:  apply transform reduction for ray sensors
    ///   in extensions when doing fixed joint reduction
    private: void ReduceGazeboExtensionSensorTransformReduction(
      std::vector<TiXmlElement*>::iterator _blobIt,
      gazebo::math::Pose _reductionTransform);

    /// reduced fixed joints:  apply transform reduction for projectors in
    ///   extensions when doing fixed joint reduction
    private: void ReduceGazeboExtensionProjectorTransformReduction(
      std::vector<TiXmlElement*>::iterator _blobIt,
      gazebo::math::Pose _reductionTransform);

    /// reduced fixed joints: transform to parent frame
    private: urdf::Pose  TransformToParentFrame(
      urdf::Pose _transformInLinkFrame,
      urdf::Pose _parentToLinkTransform);
    /// reduced fixed joints: transform to parent frame
    private: gazebo::math::Pose  TransformToParentFrame(
      gazebo::math::Pose _transformInLinkFrame,
      urdf::Pose _parentToLinkTransform);
    /// reduced fixed joints: transform to parent frame
    private: gazebo::math::Pose  TransformToParentFrame(
      gazebo::math::Pose _transformInLinkFrame,
      gazebo::math::Pose _parentToLinkTransform);
    /// reduced fixed joints: transform to parent frame
    private: gazebo::math::Pose  inverseTransformToParentFrame(
      gazebo::math::Pose _transformInLinkFrame,
      urdf::Pose _parentToLinkTransform);
    /// reduced fixed joints: utility to copy between urdf::Pose and
    ///   math::Pose
    private: gazebo::math::Pose  CopyPose(urdf::Pose _pose);
    /// reduced fixed joints: utility to copy between urdf::Pose and
    ///   math::Pose
    private: urdf::Pose  CopyPose(gazebo::math::Pose _pose);

    private: std::string GetGeometryBoundingBox(
      boost::shared_ptr<urdf::Geometry> _geometry, double *_sizeVals);


    /// print collision groups for debugging purposes
    private: void PrintCollisionGroups(UrdfLinkPtr _link);

    /// create SDF from URDF link
    private: void CreateSDF(TiXmlElement *_root,
      ConstUrdfLinkPtr _link,
      const gazebo::math::Pose &_transform);

    /// create SDF Link block based on URDF
    private: void CreateLink(TiXmlElement *_root,
      ConstUrdfLinkPtr _link,
      gazebo::math::Pose &_currentTransform);

    /// create collision blocks from urdf collisions
    private: void CreateCollisions(TiXmlElement* _elem,
      ConstUrdfLinkPtr _link);

    /// create visual blocks from urdf visuals
    private: void CreateVisuals(TiXmlElement* _elem,
      ConstUrdfLinkPtr _link);

    /// create SDF Inertial block based on URDF
    private: void CreateInertial(TiXmlElement *_elem,
      ConstUrdfLinkPtr _link);

    /// create SDF Collision block based on URDF
    private: void CreateCollision(TiXmlElement* _elem,
      ConstUrdfLinkPtr _link,
      UrdfCollisionPtr _collision,
      const std::string &_oldLinkName = std::string(""));

    /// create SDF Visual block based on URDF
    private: void CreateVisual(TiXmlElement *_elem,
      ConstUrdfLinkPtr _link,
      UrdfVisualPtr _visual,
      const std::string &_oldLinkName = std::string(""));

    /// create SDF Joint block based on URDF
    private: void CreateJoint(TiXmlElement *_root,
      ConstUrdfLinkPtr _link,
      gazebo::math::Pose &_currentTransform);

    /// create SDF geometry block based on URDF
    private: void CreateGeometry(TiXmlElement* _elem,
      boost::shared_ptr<urdf::Geometry> _geometry);

    private: std::map<std::string, std::vector<GazeboExtension*> > extensions;

    private: bool enforceLimits;
    private: bool reduceFixedJoints;
  };
  /// \}
}

#endif
