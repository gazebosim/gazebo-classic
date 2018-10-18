/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in axispliance with the License.
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

#include <gtest/gtest.h>
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/WrenchVisual.hh"
#include "gazebo/test/ServerFixture.hh"
#include "test/integration/force_torque_visualization.hh"

using namespace gazebo;

/////////////////////////////////////////////////
void ForceTorqueVisualizationTest::WrenchVisual()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  // Load world with force torque sensors
  this->Load("test/worlds/force_torque_test.world", false, false, false);

  // Create the main window.
  auto mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != nullptr);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Get the user camera and scene. We need to check the client scene
  // since this is a GUI visualization.
  auto cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != nullptr);
  auto scene = cam->GetScene();
  QVERIFY(scene != nullptr);

  // Make sure sensor visualizations is enabled
  QVERIFY(scene->EnableVisualizations());

  this->ProcessEventsAndDraw(mainWindow);

  int sleep = 0;
  int maxSleep = 30;
  std::string visName("model_1::joint_01::force_torque_GUIONLY_wrench_vis");

  while (!scene->GetVisual(visName) && sleep < maxSleep)
  {
    this->ProcessEventsAndDraw(mainWindow);
    common::Time::MSleep(100);
    sleep++;
  }

  // Check that wrench visual have been added to the scene
  auto forceTorqueVis = scene->GetVisual(visName);
  QVERIFY(forceTorqueVis != nullptr);

  auto wrenchVis =
      std::dynamic_pointer_cast<rendering::WrenchVisual>(forceTorqueVis);
  QVERIFY(wrenchVis != nullptr);

  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(ForceTorqueVisualizationTest)
