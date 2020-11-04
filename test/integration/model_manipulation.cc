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

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "model_manipulation.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void ModelManipulationTest::StopProcessingPoseMsgs()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  // Load a world which has moving models
  this->Load("test/worlds/revolute_joint_test.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Get scene
  auto scene = gazebo::gui::get_active_camera()->GetScene();
  QVERIFY(scene != NULL);

  // Check we're in select mode
  QVERIFY(gazebo::gui::g_arrowAct != NULL);
  QVERIFY(gazebo::gui::g_arrowAct->isChecked());

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Get a moving link and check its pose is changing over time
  auto linkVis = scene->GetVisual("pendulum_0deg::upper_link");
  QVERIFY(linkVis != NULL);
  auto pose = linkVis->GetWorldPose();

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  QVERIFY(pose != linkVis->GetWorldPose());
  pose = linkVis->GetWorldPose();

  // Select the parent model and check its pose still changes over time
  gazebo::event::Events::setSelectedEntity("pendulum_0deg", "normal");

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  QVERIFY(pose != linkVis->GetWorldPose());
  pose = linkVis->GetWorldPose();

  // Change to translate mode and check the model stops moving
  QVERIFY(gazebo::gui::g_translateAct != NULL);
  QVERIFY(!gazebo::gui::g_translateAct->isChecked());
  gazebo::gui::g_translateAct->trigger();

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Sequence was: select -> change mode
  QVERIFY(pose == linkVis->GetWorldPose());

  // Deselect model and check it starts moving again even though we're still in
  // translate mode
  gazebo::event::Events::setSelectedEntity("", "move");

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  QVERIFY(gazebo::gui::g_translateAct->isChecked());
  QVERIFY(pose != linkVis->GetWorldPose());
  pose = linkVis->GetWorldPose();

  // Select model again and check it stops again
  gazebo::event::Events::setSelectedEntity("pendulum_0deg", "move");

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Sequence was: change mode -> select
  QVERIFY(pose == linkVis->GetWorldPose());

  // Change to select mode and check model moves again
  QVERIFY(!gazebo::gui::g_arrowAct->isChecked());
  gazebo::gui::g_arrowAct->trigger();

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  QVERIFY(gazebo::gui::g_arrowAct->isChecked());
  QVERIFY(pose != linkVis->GetWorldPose());

  delete mainWindow;
  mainWindow = NULL;
}

// Generate a main function for the test
QTEST_MAIN(ModelManipulationTest)
