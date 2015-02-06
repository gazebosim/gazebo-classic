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
#ifndef _APPLY_WRENCH_DIALOG_PRIVATE_HH_
#define _APPLY_WRENCH_DIALOG_PRIVATE_HH_

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

      /// TODO
      public: std::string linkName;

      /// TODO
      public: QLabel *messageLabel;

      /// TODO
      public: QWidget *pointCollapsibleWidget;

      /// TODO
      public: QWidget *forceCollapsibleWidget;

      /// TODO
      public: QWidget *torqueCollapsibleWidget;

      /// TODO
      public: QDoubleSpinBox *pointXSpin;

      /// TODO
      public: QDoubleSpinBox *pointYSpin;

      /// TODO
      public: QDoubleSpinBox *pointZSpin;

      /// TODO
      public: QDoubleSpinBox *forceMagSpin;

      /// TODO
      public: QDoubleSpinBox *forceXSpin;

      /// TODO
      public: QDoubleSpinBox *forceYSpin;

      /// TODO
      public: QDoubleSpinBox *forceZSpin;

      /// TODO
      public: QDoubleSpinBox *torqueMagSpin;

      /// TODO
      public: QDoubleSpinBox *torqueXSpin;

      /// TODO
      public: QDoubleSpinBox *torqueYSpin;

      /// TODO
      public: QDoubleSpinBox *torqueZSpin;

      /// TODO
      public: math::Vector3 forceVector;

      /// TODO
      public: math::Vector3 torqueVector;

      /// \brief Publishes the wrench message.
      public: transport::PublisherPtr wrenchPub;

      /// TODO
      public: rendering::VisualPtr linkVisual;

      /// TODO
      public: rendering::ApplyWrenchVisualPtr applyWrenchVisual;

      /// TODO
      public: rendering::ApplyWrenchVisual::WrenchModes wrenchMode;

      /// TODO
      public: math::Vector2i dragStart;

      /// TODO
      public: math::Vector3 dragStartNormal;

      /// TODO
      public: math::Pose dragStartPose;

      /// TODO
      public: std::string manipState;
    };
  }
}
#endif
