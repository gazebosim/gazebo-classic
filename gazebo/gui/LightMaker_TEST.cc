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

#include "gazebo/common/MouseEvent.hh"

#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"

#include "gazebo/gui/LightMaker.hh"
#include "gazebo/gui/LightMaker_TEST.hh"

/////////////////////////////////////////////////
void LightMaker_TEST::PointLight()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Get scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  QVERIFY(scene != NULL);

  // Check there's no light in the scene yet
  gazebo::rendering::LightPtr light = scene->GetLight("__default__");
  QVERIFY(light == NULL);
  light = scene->GetLight("user_point_light_0");
  QVERIFY(light == NULL);

  // Create a pointLight maker
  gazebo::gui::PointLightMaker *pointLightMaker =
      new gazebo::gui::PointLightMaker();
  QVERIFY(pointLightMaker != NULL);

  // Start the maker to make a light
  pointLightMaker->Start();

  // Check there's a light in the scene -- this is the preview
  light = scene->GetLight("__default__");
  QVERIFY(light != NULL);

  // Check that the light appeared in the center of the screen
  ignition::math::Vector3d startPos = pointLightMaker->EntityPosition();
  QVERIFY(startPos == ignition::math::Vector3d::UnitZ);
  QVERIFY(light->GetPosition() == startPos);

  // Mouse move
  gazebo::common::MouseEvent mouseEvent;
  mouseEvent.SetType(gazebo::common::MouseEvent::MOVE);
  pointLightMaker->OnMouseMove(mouseEvent);

  // Check that entity moved
  ignition::math::Vector3d pos = pointLightMaker->EntityPosition();
  QVERIFY(pos != startPos);
  QVERIFY(light->GetPosition() == pos);

  // Mouse release
  mouseEvent.SetType(gazebo::common::MouseEvent::RELEASE);
  mouseEvent.SetButton(gazebo::common::MouseEvent::LEFT);
  mouseEvent.SetDragging(false);
  mouseEvent.SetPressPos(0, 0);
  mouseEvent.SetPos(0, 0);
  pointLightMaker->OnMouseRelease(mouseEvent);

  // Check there's no light in the scene -- the preview is gone
  light = scene->GetLight("__default__");
  QVERIFY(light == NULL);
  light = scene->GetLight("user_point_light");
  QVERIFY(light == NULL);

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check there's a light in the scene -- this is the final pointLight
  light = scene->GetLight("user_point_light_0");
  QVERIFY(light != NULL);

  // Terminate
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void LightMaker_TEST::CopyLight()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/spotlight.world");

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Get scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  QVERIFY(scene != NULL);

  // Check there's a light but no clone in the scene yet
  gazebo::rendering::LightPtr light = scene->GetLight("spotlight");
  QVERIFY(light != NULL);
  light = scene->GetLight("spotlight_clone_tmp");
  QVERIFY(light == NULL);
  light = scene->GetLight("spotlight_clone");
  QVERIFY(light == NULL);

  // Create a generic light maker
  gazebo::gui::LightMaker *lightMaker = new gazebo::gui::LightMaker();
  QVERIFY(lightMaker != NULL);

  // Start the maker to copy the light
  lightMaker->InitFromLight("spotlight");
  lightMaker->Start();

  // Check there's a light in the scene -- this is the preview
  light = scene->GetLight("spotlight_clone_tmp");
  QVERIFY(light != NULL);

  // Check that the light appeared in the center of the screen
  ignition::math::Vector3d startPos = lightMaker->EntityPosition();
  QVERIFY(startPos == ignition::math::Vector3d::UnitZ);
  QVERIFY(light->GetPosition() == startPos);

  // Mouse move
  gazebo::common::MouseEvent mouseEvent;
  mouseEvent.SetType(gazebo::common::MouseEvent::MOVE);
  lightMaker->OnMouseMove(mouseEvent);

  // Check that entity moved
  ignition::math::Vector3d pos = lightMaker->EntityPosition();
  QVERIFY(pos != startPos);
  QVERIFY(light->GetPosition() == pos);

  // Mouse release
  mouseEvent.SetType(gazebo::common::MouseEvent::RELEASE);
  mouseEvent.SetButton(gazebo::common::MouseEvent::LEFT);
  mouseEvent.SetDragging(false);
  mouseEvent.SetPressPos(0, 0);
  mouseEvent.SetPos(0, 0);
  lightMaker->OnMouseRelease(mouseEvent);

  // Check there's no light in the scene -- the preview is gone
  light = scene->GetLight("spotlight_clone_tmp");
  QVERIFY(light == NULL);

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check there's a light in the scene -- this is the final light
  light = scene->GetLight("spotlight_clone");
  QVERIFY(light != NULL);

  // Terminate
  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(LightMaker_TEST)
