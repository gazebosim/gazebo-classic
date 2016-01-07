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
#ifndef _GAZEBO_APPLY_WRENCH_DIALOG_PRIVATE_HH_
#define _GAZEBO_APPLY_WRENCH_DIALOG_PRIVATE_HH_

#include <string>
#include <vector>
#include <map>
#include <mutex>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/ApplyWrenchDialog.hh"

namespace gazebo
{
  namespace gui
  {
    /// \class ApplyWrenchDialogPrivate ApplyWrenchDialogPrivate.hh
    /// \brief Private data for the ApplyWrenchDialog class
    class ApplyWrenchDialogPrivate
    {
      /// \brief Node for communication.
      public: transport::NodePtr node;

      /// \brief Name of the model this is connected to.
      public: std::string modelName;

      /// \brief Name of the link currently targeted.
      public: std::string linkName;

      /// \brief Label holding the model name.
      public: QLabel *modelLabel;

      /// \brief Dropdown holding all link names.
      public: QComboBox *linksComboBox;

      /// \brief Radio button for CoM.
      public: QRadioButton *comRadio;

      /// \brief Radio button for force position.
      public: QRadioButton *forcePosRadio;

      /// \brief Spin for force position X.
      public: QDoubleSpinBox *forcePosXSpin;

      /// \brief Spin for force position Y.
      public: QDoubleSpinBox *forcePosYSpin;

      /// \brief Spin for force position Z.
      public: QDoubleSpinBox *forcePosZSpin;

      /// \brief Spin for force magnitude.
      public: QDoubleSpinBox *forceMagSpin;

      /// \brief Spin for force X.
      public: QDoubleSpinBox *forceXSpin;

      /// \brief Spin for force Y.
      public: QDoubleSpinBox *forceYSpin;

      /// \brief Spin for force Z.
      public: QDoubleSpinBox *forceZSpin;

      /// \brief Spin for torque magnitude.
      public: QDoubleSpinBox *torqueMagSpin;

      /// \brief Spin for torque X.
      public: QDoubleSpinBox *torqueXSpin;

      /// \brief Spin for torque Y.
      public: QDoubleSpinBox *torqueYSpin;

      /// \brief Spin for torque Z.
      public: QDoubleSpinBox *torqueZSpin;

      /// \brief CoM coordinates in link frame.
      public: math::Vector3 comVector;

      /// \brief Force position coordinates in link frame.
      public: math::Vector3 forcePosVector;

      /// \brief Force vector.
      public: math::Vector3 forceVector;

      /// \brief Torque vector.
      public: math::Vector3 torqueVector;

      /// \brief Publish user command messages for the server to place in the
      /// undo queue.
      public: transport::PublisherPtr userCmdPub;

      /// \brief Visual of the targeted link.
      public: rendering::VisualPtr linkVisual;

      /// \brief A list of events connected to this.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Map link name to link CoM vector.
      public: std::map<std::string, math::Vector3> linkToCOMMap;

      /// \brief Interactive visual which represents the wrench to be applied.
      public: rendering::ApplyWrenchVisualPtr applyWrenchVisual;

      /// \brief Indicate whether mouse is dragging on top the
      /// rotation tool or not.
      public: bool draggingTool;

      /// \brief World pose of the rotation tool the moment dragging
      /// started.
      public: math::Pose dragStartPose;

      /// \brief State of the manipulation tool, here only using "rot_y"
      /// and "rot_z".
      public: std::string manipState;

      /// \brief Current mode, either force, torque or none.
      public: ApplyWrenchDialog::Mode mode;

      /// \brief Mutex to protect variables.
      public: std::mutex mutex;

      /// \brief Pointer to the main window.
      public: MainWindow *mainWindow;
    };
  }
}
#endif
