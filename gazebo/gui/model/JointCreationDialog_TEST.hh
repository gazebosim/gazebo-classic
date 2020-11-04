/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_GUI_JOINT_CREATION_DIALOG_TEST_HH_
#define _GAZEBO_GUI_JOINT_CREATION_DIALOG_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the joint creation dialog.
class JointCreationDialog_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Constructor
  public: JointCreationDialog_TEST() = default;

  /// \brief Test joint type widget.
  private slots: void Type();

  /// \brief Test joint child and parent links widget.
  private slots: void Links();

  /// \brief Test joint axis widget.
  private slots: void Axis();

  /// \brief Test child and parent align widget.
  private slots: void Align();

  /// \brief Test the relative pose widget.
  private slots: void RelativePose();

  /// \brief Test hitting the Cancel button.
  private slots: void Cancel();
};

#endif
