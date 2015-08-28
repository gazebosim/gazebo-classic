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

#ifndef _CONFIGWIDGET_TEST_HH_
#define _CONFIGWIDGET_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the config widget.
class ConfigWidget_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Test creating config widgets from empty messages.
  private slots: void EmptyMsgWidget();

  /// \brief Test creating a config widget from a joint message.
  private slots: void JointMsgWidget();

  /// \brief Test creating a config widget from a visual message.
  private slots: void VisualMsgWidget();

  /// \brief Test setting visibility of a field in config widget.
  private slots: void ConfigWidgetVisible();

  /// \brief Test setting a field to be read-only in config widget.
  private slots: void ConfigWidgetReadOnly();

  /// \brief Test creating and updating a config widget without parsing
  /// messages.
  private slots: void CreatedExternally();

  /// \brief Test receiving a signal from child pose widget.
  private slots: void ChildPoseSignal();

  /// \brief Slot that receives pose signals from child widgets.
  /// \param[in] _name Scoped name of child widget which sent signal.
  /// \param[in] _pose New pose value.
  private slots: void OnPoseValueChanged(const QString &_name,
      const ignition::math::Pose3d &_pose);

  /// \brief Check that pose has been received.
  private: bool g_poseSignalReceived = false;
};

#endif
