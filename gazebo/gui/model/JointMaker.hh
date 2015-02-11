/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#ifndef _JOINTMAKER_HH_
#define _JOINTMAKER_HH_

#include <string>
#include <vector>
#include <boost/unordered/unordered_map.hpp>

#include <sdf/sdf.hh>

#include "gazebo/common/MouseEvent.hh"
#include "gazebo/common/KeyEvent.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace Ogre
{
  class BillboardSet;
}

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{
  namespace gui
  {
    class JointData;
    class JointInspector;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class JointMaker JointMaker.hh
    /// \brief Joint visualization
    class GAZEBO_VISIBLE JointMaker : public QObject
    {
      Q_OBJECT

      /// \enum Joint types
      /// \brief Unique identifiers for joint types that can be created.
      public: enum JointType
      {
        /// \brief none
        JOINT_NONE,
        /// \brief Fixed joint
        JOINT_FIXED,
        /// \brief Slider joint
        JOINT_SLIDER,
        /// \brief Hinge joint
        JOINT_HINGE,
        /// \brief Hinge2 joint
        JOINT_HINGE2,
        /// \brief Screw joint
        JOINT_SCREW,
        /// \brief Universal joint
        JOINT_UNIVERSAL,
        /// \brief Revolute joint
        JOINT_BALL
      };

      /// \brief Constructor
      public: JointMaker();

      /// \brief Destructor
      public: virtual ~JointMaker();

      /// \brief Reset the joint maker;
      public: void Reset();

      /// \brief Enable the mouse and key event handlers for the joint maker
      public: void EnableEventHandlers();

      /// \brief Disable the mouse and key event handlers for the joint maker
      public: void DisableEventHandlers();

      /// \brief Add a joint
      /// \param[in] _type Type of joint to be added in string.
      public: void AddJoint(const std::string &_type);

      /// \brief Add a joint
      /// \param[in] _type Type of joint to be added
      public: void AddJoint(JointType _type);

      /// \brief Create a joint with parent and child.
      /// \param[in] _parent Parent of the joint.
      /// \param[in] _child Child of the joint.
      /// \return joint data.
      public: JointData *CreateJoint(rendering::VisualPtr _parent,
          rendering::VisualPtr _child);

      /// \brief Helper method to create hotspot visual for mouse interaction.
      /// \param[in] _joint Joint data used for creating the hotspot
      public: void CreateHotSpot(JointData *_joint);

      /// \brief Update callback on PreRender.
      public: void Update();

      /// \brief Remove joint by name
      /// \param[in] _jointName Name of joint to be removed.
      public: void RemoveJoint(const std::string &_jointName);

      /// \brief Remove all joints connected to part
      /// \param[in] _partName Name of joint to be removed.
      public: void RemoveJointsByPart(const std::string &_partName);

      /// \brief Generate SDF for all joints.
      public: void GenerateSDF();

      /// \brief Generate SDF for all joints.
      public: sdf::ElementPtr GetSDF() const;

      /// \brief Get the axis count for joint type.
      /// \param[in] _type Type of joint.
      public: static int GetJointAxisCount(JointMaker::JointType _type);

      /// \brief Get the joint type in string.
      /// \param[in] _type Type of joint.
      /// \return Joint type in string.
      public: static std::string GetTypeAsString(JointMaker::JointType _type);

      /// \brief Get state
      /// \return State of JointType if joint creation is in process, otherwise
      /// JOINT_NONE
      public: JointMaker::JointType GetState() const;

      /// \brief Stop the process of adding joint to the model.
      public: void Stop();

      /// \brief Get the number of joints added.
      /// return Number of joints.
      public: unsigned int GetJointCount();

      /// \brief Create a joint from SDF. This is mainly used when editing
      /// existing models.
      /// \param[_in] _jointElement SDF element to load.
      /// \_modelName Name of the model that contains this joint.
      public: void CreateJointFromSDF(sdf::ElementPtr _jointElem,
          const std::string &_modelName = "");

      /// \brief Add a scoped link name. Nested model's link names are scoped
      /// but the parent and child field in the joint SDF element may not be.
      /// So keep track of scoped link names in order to generate the correct
      /// SDF before spawning the model.
      /// \param[in] _name Scoped link name.
      public: void AddScopedLinkName(const std::string &_name);

      /// \brief Mouse event filter callback when mouse button is pressed.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMousePress(const common::MouseEvent &_event);

      /// \brief Mouse event filter callback when mouse button is released.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMouseRelease(const common::MouseEvent &_event);

      /// \brief Mouse event filter callback when mouse is moved.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMouseMove(const common::MouseEvent &_event);

      /// \brief Mouse event filter callback when mouse is double clicked.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMouseDoubleClick(const common::MouseEvent &_event);

      /// \brief Key event filter callback when key is pressed.
      /// \param[in] _event The key event.
      /// \return True if the event was handled
      private: bool OnKeyPress(const common::KeyEvent &_event);

      /// \brief Get the centroid of the part visual in world coordinates.
      /// \param[in] _visual Visual of the part.
      /// \return Centroid in world coordinates;
      private: math::Vector3 GetPartWorldCentroid(
          const rendering::VisualPtr _visual);

      /// \brief Open joint inspector.
      /// \param[in] _name Name of joint.
      private: void OpenInspector(const std::string &_name);

      /// \brief Convert a joint type string to enum.
      /// \param[in] _type Joint type in string.
      /// \return Joint type enum.
      private: JointType ConvertJointType(const std::string &_type);

      /// \brief Get the scoped name of a link.
      /// \param[in] _name Unscoped link name.
      /// \return Scoped link name.
      private :std::string GetScopedLinkName(const std::string &_name);

      /// \brief Qt signal when the joint creation process has ended.
      Q_SIGNALS: void JointAdded();

      /// \brief Qt Callback to open joint inspector
      private slots: void OnOpenInspector();

      /// \brief Constant vector containing [UnitX, UnitY, UnitZ].
      private: std::vector<math::Vector3> UnitVectors;

      /// \brief Type of joint to create
      private: JointMaker::JointType jointType;

      /// \brief Visual that is currently hovered over by the mouse
      private: rendering::VisualPtr hoverVis;

      /// \brief Visual that is previously hovered over by the mouse
      private: rendering::VisualPtr prevHoverVis;

      /// \brief Currently selected visual
      private: rendering::VisualPtr selectedVis;

      /// \brief Joint visual that is currently being inspected.
      private: rendering::VisualPtr inspectVis;

      /// \brief All joints created by joint maker.
      private: boost::unordered_map<std::string, JointData *> joints;

      /// \brief Joint currently being created.
      private: JointData *mouseJoint;

      /// \brief All the event connections.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Flag set to true when a joint has been connected.
      private: bool newJointCreated;

      /// \brief A map of joint type to its corresponding material.
      private: boost::unordered_map<JointMaker::JointType, std::string>
          jointMaterials;

      /// \brief The SDF element pointer to the model that contains the joints.
      private: sdf::ElementPtr modelSDF;

      /// \brief Counter for the number of joints in the model.
      private: int jointCounter;

      /// \brief Qt action for opening the joint inspector.
      private: QAction *inspectAct;

      /// \brief Mutex to protect the list of joints
      private: boost::recursive_mutex *updateMutex;

      /// \brief Selected joint.
      private: rendering::VisualPtr selectedJoint;

      /// \brief A list of scoped link names.
      private: std::vector<std::string> scopedLinkedNames;
    };
    /// \}


    /// \class JointData JointData.hh
    /// \brief Helper class to store joint data
    class GAZEBO_VISIBLE JointData : public QObject
    {
      Q_OBJECT

      /// \brief Name of the joint.
      public: std::string name;

      /// \brief Visual of the dynamic line
      public: rendering::VisualPtr visual;

      /// \brief Joint visual.
      public: rendering::JointVisualPtr jointVisual;

      /// \brieft Visual of the hotspot
      public: rendering::VisualPtr hotspot;

      /// \brief Parent visual the joint is connected to.
      public: rendering::VisualPtr parent;

      /// \brief Child visual the joint is connected to.
      public: rendering::VisualPtr child;

      /// \internal
      /// \brief Parent visual pose used to determine if updates are needed.
      public: math::Pose parentPose;

      /// \internal
      /// \brief Child visual pose used to determine if updates are needed.
      public: math::Pose childPose;

      /// \internal
      /// \brief Child visual scale used to determine if updates are needed.
      public: math::Vector3 childScale;

      /// \brief Visual line used to represent joint connecting parent and child
      public: rendering::DynamicLines *line;

      /// \brief Visual handle used to represent joint parent
      public: Ogre::BillboardSet *handles;

      /// \brief Type of joint.
      public: JointMaker::JointType type;

      /// \brief Joint axis direction.
      public: math::Vector3 axis[2];

      /// \brief Joint lower limit.
      public: double lowerLimit[2];

      /// \brief Joint upper limit.
      public: double upperLimit[2];

      /// \brief Use parent model frame flag.
      public: bool useParentModelFrame[2];

      /// \brief Joint pose.
      public: math::Pose pose;

      /// \brief True if the joint visual needs update.
      public: bool dirty;

      /// \brief Msg containing joint data.
      public: msgs::JointPtr jointMsg;

      /// \brief Inspector for configuring joint properties.
      public: JointInspector *inspector;

      /// \brief Qt Callback when joint inspector configurations are to be
      /// applied.
      private slots: void OnApply();
    };
    /// \}
  }
}
#endif
