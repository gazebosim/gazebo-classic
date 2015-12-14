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
#ifndef _GAZEBO_JOINT_CREATION_DIALOG_PRIVATE_HH_
#define _GAZEBO_JOINT_CREATION_DIALOG_PRIVATE_HH_

#include <vector>

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the JointCreationDialog class
    class JointCreationDialogPrivate
    {
      /// \brief Config widget for configuring joint properties.
      public: ConfigWidget *configWidget;

      /// \brief Widget for the parent link.
      public: ConfigChildWidget *parentLinkWidget;

      /// \brief Widget for the child link.
      public: ConfigChildWidget *childLinkWidget;

      /// \brief Pointer to the joint maker.
      public: JointMaker *jointMaker;

      /// \brief Group of buttons for joint types.
      public: QButtonGroup *typeButtons;

      /// \brief Axis presets combo box.
      public: QComboBox *axis1PresetsCombo;

      /// \brief Axis presets combo box.
      public: QComboBox *axis2PresetsCombo;

      /// \brief A list of gui editor events connected to this palette.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Button to create joint.
      public: QPushButton *createButton;

      /// \brief Button to swap parent and child links.
      public: QToolButton *swapButton;

      /// \brief Vector containing the 3 button groups.
      public: std::vector<QButtonGroup *> alignGroups;

      /// \brief Combo box to select the alignment target.
      public: QComboBox *alignCombo;

      /// \brief Check box to toggle reverse X alignment.
      public: QCheckBox *reverseXBox;

      /// \brief Check box to toggle reverse Y alignment.
      public: QCheckBox *reverseYBox;

      /// \brief Check box to toggle reverse Z alignment.
      public: QCheckBox *reverseZBox;

      /// \brief Label for joints without axes.
      public: QLabel *axis0Widget;

      /// \brief Widget for axis 1.
      public: ConfigChildWidget *axis1Widget;

      /// \brief Widget for axis 2.
      public: ConfigChildWidget *axis2Widget;

      /// \brief Icon displayed at the parent link widget.
      public: QLabel *parentIcon;

      /// \brief Text with instructions on how to select links.
      public: QLabel *selectionsText;

      /// \brief Flag to indicate whether the current links are different from
      /// each other.
      public: bool validLinks;

      /// \brief Flag to indicate whether axis 1 is not zero.
      public: bool validAxis1;

      /// \brief Flag to indicate whether axis 2 is not zero.
      public: bool validAxis2;

      /// \brief Flag to indicate whether there's alignment pending.
      public: bool alignPending;
    };
  }
}
#endif
