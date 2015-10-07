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

#ifndef _GAZEBO_JOINT_INSPECTOR_HH_
#define _GAZEBO_JOINT_INSPECTOR_HH_

#include <string>
#include <vector>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/util/system.hh"


namespace gazebo
{
  namespace gui
  {
    class JointMaker;
    class ConfigWidget;
    class ConfigChildWidget;

    /// \class JointInspector gui/JointInspector.hh
    /// \brief A class to inspect and modify joints.
    class GZ_GUI_VISIBLE JointInspector : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _jointMaker Pointer to joint maker.
      /// \param[in] _parent Parent QWidget.
      public: JointInspector(JointMaker *_jointMaker, QWidget *_parent = 0);

      /// \brief Destructor
      public: ~JointInspector();

      /// \brief Update the joint config widget with a joint msg.
      /// \param[in] _jointMsg Joint message.
      public: void Update(ConstJointPtr _jointMsg);

      /// \brief Get the msg containing all joint data.
      /// \return Joint msg.
      public: msgs::Joint *GetData() const;

      /// \brief Set the pose of the joint.
      /// \param[in] _pose Pose to set the joint to.
      public: void SetPose(const math::Pose &_pose);

      /// \brief Open the inspector.
      public: void Open();

      /// \brief Set the unique id for the joint this inspector is attached to.
      /// The ID might be generated after the inspector.
      /// \param[in] _id Unique id.
      public: void SetJointId(const std::string &_id);

      /// \brief Qt event emiited when the mouse enters this widget.
      /// \param[in] _event Qt event.
      protected: virtual void enterEvent(QEvent *_event);

      /// \brief Qt callback when an enum value has changed.
      /// \param[in] _name of widget in the config widget that emitted the
      /// signal.
      /// \param[in] _value New value in string.
      private slots: void OnEnumChanged(const QString &_name,
          const QString &_value);

      /// \brief Qt callback when a pose value has changed.
      /// \param[in] _name of widget in the config widget that emitted the
      /// signal.
      /// \param[in] _value New value.
      private slots: void OnPoseChanged(const QString &_name,
          const ignition::math::Pose3d &_pose);

      /// \brief Qt callback when a Vector3d value has changed.
      /// \param[in] _name of widget in the config widget that emitted the
      /// signal.
      /// \param[in] _value New value.
      private slots: void OnVector3dChanged(const QString &_name,
          const ignition::math::Vector3d &_vec);

      /// \brief Qt callback when a string value has changed.
      /// \param[in] _name of widget in the config widget that emitted the
      /// signal.
      /// \param[in] _value New value.
      private slots: void OnStringChanged(const QString &_name,
          const std::string &_str);

      /// \brief Callback when the joint type has changed.
      /// \param[in] _value New joint type.
      private: void OnJointTypeChanged(const QString &_value);

      /// \brief Callback when the joint parent or child link has changed.
      /// \param[in] _linkName New link's name.
      private: void OnLinksChanged(const QString &_linkName = "");

      /// \brief Callback when the swap button is pressed.
      private slots: void OnSwap();

      /// \brief Add a link to the parent and child lists.
      /// \param[in] _linkName Scoped link name.
      private slots: void OnLinkInserted(const std::string &_linkName);

      /// \brief Remove a link from the parent and child lists.
      /// \param[in] _linkName Link name.
      private slots: void OnLinkRemoved(const std::string &_linkName);

      /// \brief Qt signal emitted to indicate that changes should be applied.
      Q_SIGNALS: void Applied();

      /// \brief Qt callback when the Remove button is pressed.
      private slots: void OnRemove();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Ok button is pressed.
      private slots: void OnOK();

      /// \brief Enable ok button if all values in the dialog are valid.
      private: void CheckValid();

      /// \brief
      private: void RestoreOriginalData();

      /// \brief Config widget for configuring joint properties.
      private: ConfigWidget *configWidget;

      /// \brief Widget for the joint name.
      private: ConfigChildWidget *nameWidget;

      /// \brief Custom widget for the parent link to be used instead of the one
      /// generated by parsing the joint message.
      private: ConfigChildWidget *parentLinkWidget;

      /// \brief Custom widget for the child link to be used instead of the one
      /// generated by parsing the joint message.
      private: ConfigChildWidget *childLinkWidget;

      /// \brief Ok button.
      private: QPushButton *okButton;

      /// \brief A list of gui editor events connected to this.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Pointer to the joint maker.
      private: JointMaker *jointMaker;

      /// \brief Style sheet for link widgets when there's a warning.
      private: QString warningStyleSheet;

      /// \brief Normal style sheet for link widgets.
      private: QString normalStyleSheet;

      /// \brief Unique ID which identifies this joint in the joint maker.
      private: std::string jointId;

      /// \brief Flag that indicates whether current joint name is valid.
      private: bool validJointName;

      /// \brief Flag that indicates whether current links are valid.
      private: bool validLinks;

      private: msgs::Joint originalDataMsg;
    };
    /// \}
  }
}

#endif
