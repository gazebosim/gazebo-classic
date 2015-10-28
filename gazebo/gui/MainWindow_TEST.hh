/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _MAINWINDOW_TEST_HH_
#define _MAINWINDOW_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the main window class.
class MainWindow_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Test the step action state when simulation is running and paused.
  private slots: void StepState();

  /// \brief Test that Actions created in MainWindow get destroyed.
  private slots: void ActionCreationDestruction();

  /// \brief Test scene destruction on shutdown
  private slots: void SceneDestruction();

  /// \brief Test user camera entity selection
  private slots: void Selection();

  /// \brief Test user camera frames per second
  private slots: void UserCameraFPS();

  /// \brief Test copying and pasting a model and a light
  private slots: void CopyPaste();

  /// \brief Test that trigger of the view wireframe action creates an
  /// appropriate request.
  private slots: void Wireframe();

  /// \brief Test creating a main window with non-default world
  private slots: void NonDefaultWorld();

  /// \brief Test moving the user camera via a joystick message.
  private slots: void UserCameraJoystick();

  /// \brief Test Set user camera pose via SDF
  private slots: void SetUserCameraPoseSDF();

  /// \brief Test that menus are created in the main window menu bar.
  private slots: void MenuBar();

  /// \brief Test different window modes.
  private slots: void WindowModes();
};

#endif
