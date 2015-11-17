/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_JOINT_CREATION_DIALOG_HH_
#define _GAZEBO_JOINT_CREATION_DIALOG_HH_

#include <map>
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
    class JointCreationDialogPrivate;

    /// \class JointCreationDialog gui/JointCreationDialog.hh
    /// \brief A class to inspect and modify joints.
    class GZ_GUI_VISIBLE JointCreationDialog : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _jointMaker Pointer to the joint maker.
      /// \param[in] _parent Parent QWidget.
      public: JointCreationDialog(JointMaker *_jointMaker,
          QWidget *_parent = 0);

      /// \brief Destructor
      public: ~JointCreationDialog();

      /// \brief Open the dialog.
      /// \param[in] _type Joint type which will be selected when dialog opens.
      public: void Open(JointMaker::JointType _type);

      /// \brief Update the relative pose widget.
      /// \param[in] _pose New pose.
      public: void UpdateRelativePose(const ignition::math::Pose3d &_pose);

      // Documentation inherited
      protected: virtual void enterEvent(QEvent *_event);

      /// \brief Qt callback when a link is chosen on the dialog.
      private slots: void OnLinkFromDialog();

      /// \brief Qt callback when the axis is changed on the dialog.
      /// \param[in] _name Name of the axis widget in the config widget that
      /// emitted the signal.
      /// \param[in] _pose New pose.
      private slots: void OnVector3dFromDialog(const QString &_name,
          const ignition::math::Vector3d &_pose);

      /// \brief Qt callback when the pose is changed on the dialog.
      /// \param[in] _name Name of the pose widget in the config widget that
      /// emitted the signal.
      /// \param[in] _pose New pose.
      private slots: void OnPoseFromDialog(const QString &_name,
          const ignition::math::Pose3d &_pose);

      /// \brief Qt callback when an enum value has changed on the dialog.
      /// \param[in] _name Name of the enum widget in the config widget that
      /// emitted the signal.
      /// \param[in] _value New value in string.
      private slots: void OnEnumChanged(const QString &_name,
          const QString &_value);

      /// \brief Qt callback when the Reset poses button is pressed.
      private slots: void OnResetPoses();

      /// \brief Qt callback when the axis combo box is changed.
      private slots: void OnAlign(const int _int);
      private slots: void SetType(const int _typeInt);

      /// \brief Qt callback when the Swap links button is pressed.
      private slots: void OnSwap();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Create button is pressed.
      private slots: void OnCreate();

      /// \brief Event callback when the parent link is chosen in the 3D scene.
      /// \param[in] _linkName Name of new parent link.
      public: void SetParent(const std::string &_linkName);

      /// \brief Event callback when the child link is chosen in the 3D scene.
      /// \param[in] _linkName Name of new child link.
      public: void SetChild(const std::string &_linkName);

      /// \brief Check if the current parent and child link selection is valid.
      private: void CheckLinksValid();

      /// \brief Enable ok button if all values in the dialog are valid.
      /// \return True if all values are valid.
      private: bool CheckValid();

      /// \internal
      /// \brief Handles choosing the parent link, whether it is chosen from the
      /// dialog or the 3D scene.
      /// \param[in] _linkName Name of new parent link.
      private: void OnParentImpl(const QString &_linkName);

      /// \internal
      /// \brief Handles choosing the child link, whether it is chosen from the
      /// dialog or the 3D scene.
      /// \param[in] _linkName Name of new child link.
      private: void OnChildImpl(const QString &_linkName);

      /// \internal
      /// \brief Pointer to private data.
      private: JointCreationDialogPrivate *dataPtr;
    };
    /// \}
  }
}

#endif
