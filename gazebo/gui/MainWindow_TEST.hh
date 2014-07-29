/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

/// \brief A test class for the DataLogger widget.
class MainWindow_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Test copying and pasting a model
  private slots: void CopyPasteModel();

  /// \brief Test copying and pasting a light
  private slots: void CopyPasteLight();

  /// \brief Test that trigger of the view wireframe action creates an
  /// appropriate request.
  private slots: void Wireframe();

  /// \brief Test creating a main window with non-default world
  private slots: void NonDefaultWorld();
};

#endif
