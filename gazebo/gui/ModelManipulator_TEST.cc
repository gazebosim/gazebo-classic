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

#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/ModelManipulator.hh"

#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/gui/ModelManipulator_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void ModelManipulator_TEST::Attach()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  gazebo::rendering::VisualPtr vis1;
  vis1.reset(new gazebo::rendering::Visual("vis1", scene->WorldVisual()));
  vis1->Load();

  gazebo::rendering::VisualPtr vis2;
  vis2.reset(new gazebo::rendering::Visual("vis2", scene->WorldVisual()));
  vis2->Load();

  // initialize the model manipulator
  gazebo::gui::ModelManipulator::Instance()->Init();

  // attach to visual and verify the visual has one extra child
  QCOMPARE(vis1->GetChildCount(), 0u);
  gazebo::gui::ModelManipulator::Instance()->SetAttachedVisual(vis1);
  QCOMPARE(vis1->GetChildCount(), 1u);

  // detach and verify the child is gone
  gazebo::gui::ModelManipulator::Instance()->Detach();
  QCOMPARE(vis1->GetChildCount(), 0u);

  // attach again to different visual
  QCOMPARE(vis2->GetChildCount(), 0u);
  gazebo::gui::ModelManipulator::Instance()->SetAttachedVisual(vis2);
  QCOMPARE(vis2->GetChildCount(), 1u);

  // attach to another visual without explicitly detaching
  gazebo::gui::ModelManipulator::Instance()->SetAttachedVisual(vis1);
  QCOMPARE(vis1->GetChildCount(), 1u);
  QCOMPARE(vis2->GetChildCount(), 0u);

  // remove vis1 while model manipulator is attached.
  scene->RemoveVisual(vis1);
  vis1.reset();
  QVERIFY(scene->GetVisual("vis1") == NULL);

  // verify we can still attach to vis2
  gazebo::gui::ModelManipulator::Instance()->SetAttachedVisual(vis2);
  QCOMPARE(vis2->GetChildCount(), 1u);

  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

// Generate a main function for the test
QTEST_MAIN(ModelManipulator_TEST)
