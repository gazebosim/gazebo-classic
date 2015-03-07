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

#include "gazebo/gui/qt.h"

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

      /// Name of the model this is connected to.
      public: std::string modelName;

      /// Name of the link currently targeted.
      public: std::string linkName;

      /// Label holding the model name.
      public: QLabel *modelLabel;

      /// Dropdown holding all link names.
      public: QComboBox *linksComboBox;

      /// Radio button for CoM.
      public: QRadioButton *comRadio;

      /// Radio button for force position.
      public: QRadioButton *forcePosRadio;

      /// Spin for force position X.
      public: QDoubleSpinBox *forcePosXSpin;

      /// Spin for force position Y.
      public: QDoubleSpinBox *forcePosYSpin;

      /// Spin for force position Z.
      public: QDoubleSpinBox *forcePosZSpin;

      /// Spin for force magnitude.
      public: QDoubleSpinBox *forceMagSpin;

      /// Spin for force X.
      public: QDoubleSpinBox *forceXSpin;

      /// Spin for force Y.
      public: QDoubleSpinBox *forceYSpin;

      /// Spin for force Z.
      public: QDoubleSpinBox *forceZSpin;

      /// Spin for torque magnitude.
      public: QDoubleSpinBox *torqueMagSpin;

      /// Spin for torque X.
      public: QDoubleSpinBox *torqueXSpin;

      /// Spin for torque Y.
      public: QDoubleSpinBox *torqueYSpin;

      /// Spin for torque Z.
      public: QDoubleSpinBox *torqueZSpin;

      /// CoM coordinates in link frame.
      public: math::Vector3 comVector;

      /// Forse position coordinates in link frame.
      public: math::Vector3 forcePosVector;

      /// Force vector.
      public: math::Vector3 forceVector;

      /// Torque vector.
      public: math::Vector3 torqueVector;

      /// \brief Publishes the wrench message.
      public: transport::PublisherPtr wrenchPub;

      /// Visual of the targeted link.
      public: rendering::VisualPtr linkVisual;

      /// Interactive visual which represents the wrench applied.
      public: rendering::ApplyWrenchVisualPtr applyWrenchVisual;

      /// Indicate whether mousepress is dragging on top the rotation tool or
      /// not.
      public: bool draggingTool;

      /// World pose of the rotation tool the moment dragging started.
      public: math::Pose dragStartPose;

      /// State of the manipulation tool, here only using "rot_y" and "tor_z"
      public: std::string manipState;

      /// Current mode, either "force", "torque" or "none".
      public: std::string mode;

      /// Message to request for entity info.
      public: msgs::Request *requestMsg;

      /// Publishes the request message.
      public: transport::PublisherPtr requestPub;

      /// Subscribes to response messages.
      public: transport::SubscriberPtr responseSub;

      /// Pointer to the main window.
      public: MainWindow *mainWindow;

      /// \brief A list of events connected to this.
      public: std::vector<event::ConnectionPtr> connections;
    };
  }
}
#endif
