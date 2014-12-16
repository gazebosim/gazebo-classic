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
/*#include "gazebo/math/Helpers.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/TimePanel.hh"
#include "gazebo/gui/GLWidget.hh"*/
#include <boost/filesystem.hpp>
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/InsertModelWidget.hh"
#include "gazebo/gui/InsertModelWidget_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void InsertModelWidget_TEST::ReadPermissions()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, true);

  gazebo::gui::MainWindow mainWindow;

  mainWindow.Load();
  mainWindow.Init();

  // wait a bit for the event to fire (?)
  gazebo::gui::InsertModelWidget *insertModelWidget =
      mainWindow.findChild<gazebo::gui::InsertModelWidget *>("insertModel");

  // Create files in /tmp and set permissions accordingly

  boost::filesystem::path testDir("/tmp/InsertModelWidget_TEST");
  if (!boost::filesystem::exists(testDir))
  {
    boost::filesystem::create_directories(testDir);
  }

  // Case 1: add a restricted access folder to GAZEBO_MODEL_PATHS

  boost::filesystem::path case1 = testDir / "case1";
  if (!boost::filesystem::exists(case1))
  {
    boost::filesystem::create_directories(case1);
  }
  boost::filesystem::permissions(case1, boost::filesystem::no_perms);

  // Try to add the folder to the model path
  gazebo::common::SystemPaths::Instance()->AddModelPathsUpdate(case1.string());

  // Check if it is in InsertModelWidget
  QVERIFY(insertModelWidget->LocalPathInFileWidget(case1.string()));

  // GAZEBO_MODEL_PATHS contains a restricted access folder

  // GAZEBO_MODEL_PATHS contains parent of a restricted access file


  mainWindow.close();

  // Delete all test files
  boost::filesystem::remove_all(testDir);

  /*std::string modelName = "cylinder";

  gazebo::rendering::Events::createScene("default");

  // trigger selection to initialize wirebox's vertex buffer creation first.
  // Otherwise test segfaults later when selecting a model due to making
  // this call outside the rendering thread.
  gazebo::event::Events::setSelectedEntity(modelName, "normal");

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelName);
  QVERIFY(modelVis != NULL);

  // Select the model
  gazebo::event::Events::setSelectedEntity(modelName, "normal");

  // Wait until the model is selected
  int sleep = 0;
  int maxSleep = 100;
  while (!modelVis->GetHighlighted() && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(30);
    sleep++;
  }
  QVERIFY(modelVis->GetHighlighted());

  // Get GLWidget
  gazebo::gui::GLWidget *glWidget =
      mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != NULL);

  // Copy the model
  QTest::keyClick(glWidget, Qt::Key_C, Qt::ControlModifier);
  QTest::qWait(500);

  // Move to center of the screen
  QPoint moveTo(glWidget->width()/2, glWidget->height()/2);
  QTest::mouseMove(glWidget, moveTo);
  QTest::qWait(500);

  // Paste the model
  QTest::keyClick(glWidget, Qt::Key_V, Qt::ControlModifier);
  QTest::qWait(500);

  // Release and spawn the model
  QTest::mouseClick(glWidget, Qt::LeftButton, Qt::NoModifier, moveTo);
  QTest::qWait(500);

  QCoreApplication::processEvents();

  // Verify there is a clone of the model
  gazebo::rendering::VisualPtr modelVisClone;
  sleep = 0;
  maxSleep = 100;
  while (!modelVisClone && sleep < maxSleep)
  {
    modelVisClone = scene->GetVisual(modelName + "_clone");
    QTest::qWait(30);
    sleep++;
  }
  QVERIFY(modelVisClone);

  cam->Fini();
  delete mainWindow;*/
}

// Generate a main function for the test
QTEST_MAIN(InsertModelWidget_TEST)
