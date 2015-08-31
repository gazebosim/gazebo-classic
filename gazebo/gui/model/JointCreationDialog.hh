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

    /// \class JointCreationDialog gui/JointCreationDialog.hh
    /// \brief A class to inspect and modify joints.
    class GZ_GUI_VISIBLE JointCreationDialog : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: JointCreationDialog(JointMaker *_jointMaker,
          QWidget *_parent = 0);

      /// \brief Destructor
      public: ~JointCreationDialog() = default;

      public: void Open(JointMaker::JointType _type);

      public: void UpdateRelativePose(const ignition::math::Pose3d &_pose);

      /// \brief Qt event emiited when the mouse enters this widget.
      /// \param[in] _event Qt event.
      protected: virtual void enterEvent(QEvent *_event);

      private slots: void OnTypeFromDialog(int _type);
      private slots: void OnLinkFromDialog();
      private slots: void OnPoseFromDialog(const QString &_name,
          const ignition::math::Pose3d &_pose);
      private slots: void OnParentFrom3D(const std::string &_linkName);
      private slots: void OnChildFrom3D(const std::string &_linkName);
      private slots: void OnSwap();
      private: void OnParentImpl(const QString &_linkName);
      private: void OnChildImpl(const QString &_linkName);

      /// \brief Qt callback when an enum value has changed.
      /// \param[in] _name of widget in the config widget that emitted the
      /// signal.
      /// \param[in] _value New value in string.
      private slots: void OnEnumChanged(const QString &_name,
          const QString &_value);

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Ok button is pressed.
      private slots: void OnCreate();

      private: void CheckLinksValid();

      /// \brief Config widget for configuring joint properties.
      private: ConfigWidget *configWidget;

      /// \brief Widget for the parent link.
      private: ConfigChildWidget *parentLinkWidget;

      /// \brief Widget for the child link.
      private: ConfigChildWidget *childLinkWidget;

      /// \brief Config widget for configuring joint properties.
      private: JointMaker *jointMaker;

      private: QButtonGroup *typeButtons;

      /// \brief A list of gui editor events connected to this palette.
      private: std::vector<event::ConnectionPtr> connections;


      private: QPushButton *createButton;
      private: QToolButton *swapButton;

      /// \brief Style sheet for link widgets when there's a warning.
      private: QString warningStyleSheet;

      /// \brief Normal style sheet for link widgets.
      private: QString normalStyleSheet;
    };
    /// \}
  }
}

#endif
