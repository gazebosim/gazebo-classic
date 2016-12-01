/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"

#include "camera_gui.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void CameraGUITest::Heightmap()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/heightmap.world", false, false, false);

  // Create the main window.
  auto mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != nullptr);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get user camera
  auto camera = gazebo::gui::get_active_camera();
  QVERIFY(camera != nullptr);

  QVERIFY(camera->WorldPose() != camera->DefaultPose().Ign());

  QVERIFY(camera->WorldPose() ==
      ignition::math::Pose3d(5, -5, 210, 0, 1.53546, 2.35619));

  // Clean up
  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(CameraGUITest)
