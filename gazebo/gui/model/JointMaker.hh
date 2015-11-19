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

#ifndef _GAZEBO_JOINTMAKER_HH_
#define _GAZEBO_JOINTMAKER_HH_

#include <map>
#include <string>
#include <vector>

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
    class JointCreationDialog;
    class JointData;
    class JointInspector;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class JointMaker JointMaker.hh
    /// \brief Joint visualization
    class GZ_GUI_VISIBLE JointMaker : public QObject
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
        /// \brief Ball joint
        JOINT_BALL,
        /// \brief Gearbox joint
        JOINT_GEARBOX
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

      /// \brief Remove joint by name.
      /// \param[in] _jointName Name of joint to be removed, or an empty string
      /// to remove the new joint under creation.
      public: void RemoveJoint(const std::string &_jointName);

      /// \brief Remove all joints connected to link.
      /// \param[in] _linkName Name of the link.
      public: void RemoveJointsByLink(const std::string &_linkName);

      /// \brief Get a vector containing data for all joints connected to
      /// the given link.
      /// \param[in] _linkName Name of the link.
      /// \return Vector with joint data.
      public: std::vector<JointData *> GetJointDataByLink(
          const std::string &_linkName) const;

      /// \brief Generate SDF for all joints.
      public: void GenerateSDF();

      /// \brief Generate SDF for all joints.
      public: sdf::ElementPtr GetSDF() const;

      /// \brief Get the axis count for joint type.
      /// \param[in] _type Type of joint.
      public: static unsigned int GetJointAxisCount(
          JointMaker::JointType _type);

      /// \brief Get the joint type in string.
      /// \param[in] _type Type of joint.
      /// \return Joint type in string.
      public: static std::string GetTypeAsString(JointMaker::JointType _type);

      /// \brief Convert a joint type string to enum.
      /// \param[in] _type Joint type in string.
      /// \return Joint type enum.
      public: static JointType ConvertJointType(const std::string &_type);

      /// \brief Get the material for the joint type.
      /// \param[in] _type Type of joint.
      /// \return Name of material.
      public: static std::string GetJointMaterial(const std::string &_type);

      /// \brief Get state
      /// \return Current state of the joint maker. If mouse is enabled to
      /// create a new joint, it returns the type of joint. Otherwise, it
      /// returns JOINT_NONE.
      public: JointMaker::JointType GetState() const;

      /// \brief Stop the process of adding joint to the model.
      public: void Stop();

      /// \brief Get the number of joints added.
      /// return Number of joints.
      public: unsigned int GetJointCount();

      /// \brief Create a joint from SDF. This is mainly used when editing
      /// existing models.
      /// \param[in] _jointElement SDF element to load.
      /// \param[in] _modelName Name of the model that contains this joint.
      public: void CreateJointFromSDF(sdf::ElementPtr _jointElem,
          const std::string &_modelName = "");

      /// \brief Add a scoped link name. Nested model's link names are scoped
      /// but the parent and child field in the joint SDF element may not be.
      /// So keep track of scoped link names in order to generate the correct
      /// SDF before spawning the model.
      /// \param[in] _name Scoped link name.
      public: void AddScopedLinkName(const std::string &_name);

      /// \brief Qt Callback to show / hide joint visuals.
      /// \param[in] _show True to show joints, false to hide them.
      public slots: void ShowJoints(bool _show);

      /// \brief Set the select state of a joint.
      /// \param[in] _name Name of the joint.
      /// \param[in] _selected True to select the joint.
      public: void SetSelected(const std::string &_name, const bool selected);

      /// \brief Set the select state of a joint visual.
      /// \param[in] _jointVis Pointer to the joint visual.
      /// \param[in] _selected True to select the joint.
      public: void SetSelected(rendering::VisualPtr _jointVis,
          const bool selected);

      /// \brief Get the list of links.
      /// \return The list of links, with the link scoped name and leaf name.
      public: std::map<std::string, std::string> LinkList() const;

      /// \brief A new type for the joint being created has been chosen.
      /// To be used by other classes.
      /// \param[in] _typeInt Integer corresponding to joint type enum.
      public slots: void SetType(const int _typeInt);

      /// \brief A new axis for the joint being created has been chosen.
      /// To be used by other classes.
      /// \param[in] _axis Axis which was changed, either "axis1" or "axis2".
      /// \param[in] _value New value for the axis
      public slots: void SetAxis(const QString &_axis,
          const ignition::math::Vector3d &_value);

      public: void SetJointPose(const ignition::math::Pose3d &_pose);

      /// \brief A new parent link for the joint being created has been chosen.
      /// To be used by other classes.
      /// \sa SetParentLink(rendering::VisualPtr _parentLink)
      /// \param[in] _name Link name, either the leaf or scoped.
      public: void SetParentLink(const std::string &_name);

      /// \brief A new child link for the joint being created has been chosen.
      /// To be used by other classes.
      /// \sa SetChildLink(rendering::VisualPtr _childLink)
      /// \param[in] _name Link name, either the leaf or scoped.
      public: void SetChildLink(const std::string &_name);

      /// \brief Set the relative pose
      public: void SetLinksRelativePose(
          const ignition::math::Pose3d &_pose, bool _reset);

      public: void AlignLinks(
          const bool _childToParent, const std::string &_axis,
          const std::string &_mode);

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

      /// \brief Get the centroid of the link visual in world coordinates.
      /// \param[in] _visual Visual of the link.
      /// \return Centroid in world coordinates;
      private: math::Vector3 GetLinkWorldCentroid(
          const rendering::VisualPtr &_visual);

      /// \brief Open joint inspector.
      /// \param[in] _name Name of joint.
      private: void OpenInspector(const std::string &_name);

      /// \brief Get the scoped name of a link.
      /// \param[in] _name Unscoped link name.
      /// \return Scoped link name.
      private: std::string GetScopedLinkName(const std::string &_name);

      /// \brief Show a joint's context menu
      /// \param[in] _joint Name of joint the context menu is associated with.
      private: void ShowContextMenu(const std::string &_joint);

      /// \brief Deselect all currently selected joint visuals.
      private: void DeselectAll();

      /// \brief Callback when an entity is selected.
      /// \param[in] _name Name of entity.
      /// \param[in] _mode Select mode
      private: void OnSetSelectedEntity(const std::string &_name,
          const std::string &_mode);

      /// \brief Callback when a joint is selected.
      /// \param[in] _name Name of joint.
      /// \param[in] _selected True if the joint is selected, false if
      /// deselected.
      private: void OnSetSelectedJoint(const std::string &_name,
          const bool _selected);

      /// \brief Add a link to the list.
      /// \param[in] _linkName Scoped link name.
      private: void OnLinkInserted(const std::string &_linkName);

      /// \brief Remove a link from the list.
      /// \param[in] _linkId Unique link identifying name.
      private: void OnLinkRemoved(const std::string &_linkName);

      /// \brief Create a joint line.
      /// \param[in] _name Name to give the visual that contains the joint line.
      /// \param[in] _parent Parent of the joint.
      /// \return joint data.
      private: JointData *CreateJointLine(const std::string &_name,
          rendering::VisualPtr _parent);

      /// \brief Get a link's visual from the link's name.
      /// \param[in] _name Link name, scoped or not.
      /// \return Pointer to link visual.
      private: rendering::VisualPtr LinkVisualFromName(
          const std::string &_name);

      /// \brief Set a new parent link for the joint being created.
      /// \sa SetParentLink(const std::string &_name);
      /// \param[in] _parentLink Pointer to the link visual.
      /// \return True if successfully set new parent.
      private: bool SetParentLink(rendering::VisualPtr _parentLink);

      /// \brief Set a new child link for the joint being created.
      /// \sa SetChildLink(const std::string &_name);
      /// \param[in] _childLink Pointer to the link visual.
      /// \return True if successfully set new child.
      private: bool SetChildLink(rendering::VisualPtr _childLink);

      /// \brief Highlight link visuals which have been moved while creating
      /// a new joint.
      /// \param[in] _vis Visual to be highlighted.
      /// \param[in] _moved Whether it moved or not.
      private: void SetVisualMoved(rendering::VisualPtr _vis, bool _moved);

      /// \brief Qt signal when the joint creation process has ended.
      Q_SIGNALS: void JointAdded();

      /// \brief Qt signal to notify that a link has been inserted.
      /// \param[in] _linkId Link's unique name.
      Q_SIGNALS: void EmitLinkInserted(const std::string &_linkId);

      /// \brief Qt signal to notify that a link has been removed.
      /// \param[in] _linkId Link's unique name.
      Q_SIGNALS: void EmitLinkRemoved(const std::string &_linkId);

      /// \brief Qt Callback to open joint inspector
      private slots: void OnOpenInspector();

      /// \brief Qt callback when a delete signal has been emitted. This is
      /// currently triggered by the context menu via right click.
      private slots: void OnDelete();

      /// \brief Type of joint to create
      private: JointMaker::JointType jointType;

      /// \brief Visual that is currently hovered over by the mouse
      private: rendering::VisualPtr hoverVis;

      /// \brief Name of joint that is currently being inspected.
      private: std::string inspectName;

      /// \brief All joints created by joint maker.
      private: std::map<std::string, JointData *> joints;

      /// \brief Joint currently being created.
      private: JointData *newJoint;

      /// \brief All the event connections.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief The SDF element pointer to the model that contains the joints.
      private: sdf::ElementPtr modelSDF;

      /// \brief Counter for the number of joints in the model.
      private: int jointCounter;

      /// \brief Qt action for opening the joint inspector.
      private: QAction *inspectAct;

      /// \brief Mutex to protect the list of joints
      private: boost::recursive_mutex *updateMutex;

      /// \brief A list of selected link visuals.
      private: std::vector<rendering::VisualPtr> selectedJoints;

      /// \brief A list of scoped link names.
      private: std::vector<std::string> scopedLinkedNames;

      /// \brief A map of joint type to its string value.
      public: static std::map<JointMaker::JointType, std::string> jointTypes;

      /// \brief Constant vector containing [UnitX, UnitY, UnitZ].
      public: static std::vector<ignition::math::Vector3d> unitVectors;

      /// \brief A map of joint type to its corresponding material.
      public: static std::map<JointMaker::JointType, std::string>
          jointMaterials;

      /// \brief List of all links currently in the editor. The first string is
      /// the link's fully scoped name and the second is the leaf name.
      private: std::map<std::string, std::string> linkList;

      /// \brief Dialog for creating a new joint.
      private: JointCreationDialog *jointCreationDialog;

      /// \brief Visual currently selected to be the child link.
      private: ignition::math::Pose3d parentLinkOriginalPose;

      /// \brief Visual currently selected to be the child link.
      private: ignition::math::Pose3d childLinkOriginalPose;
    };
    /// \}


    /// \class JointData JointData.hh
    /// \brief Helper class to store joint data
    class GZ_GUI_VISIBLE JointData : public QObject
    {
      Q_OBJECT

      /// \brief Open the joint inspector.
      public: void OpenInspector();

      /// \brief Update this joint data. Avoid calling this directly, instead,
      /// set dirty to true and this will be called on PreRender.
      public: void Update();

      /// \brief Update the joint message based on the other fields.
      public: void UpdateMsg();

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

      /// \brief True if the joint needs update.
      public: bool dirty;

      /// \brief Msg containing joint data.
      public: msgs::JointPtr jointMsg;

      /// \brief Inspector for configuring joint properties.
      public: JointInspector *inspector;

      /// \brief Qt Callback when joint inspector is to be opened.
      private slots: void OnOpenInspector();

      /// \brief Qt Callback when joint inspector configurations are to be
      /// applied.
      private slots: void OnApply();
    };
    /// \}
  }
}
#endif
