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
  QVERIFY(mainWindow != nullptr);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != nullptr);

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
  QVERIFY(scene->GetVisual("vis1") == nullptr);

  // verify we can still attach to vis2
  gazebo::gui::ModelManipulator::Instance()->SetAttachedVisual(vis2);
  QCOMPARE(vis2->GetChildCount(), 1u);

  mainWindow->close();
  delete mainWindow;
  mainWindow = nullptr;
}

/////////////////////////////////////////////////
void ModelManipulator_TEST::Transparency()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != nullptr);
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
  QVERIFY(scene != nullptr);

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  gazebo::rendering::VisualPtr vis1 = scene->GetVisual("box");
  QVERIFY(vis1 != nullptr);

  double vis1Transp = 0.2;
  vis1->SetTransparency(vis1Transp);
  QVERIFY(ignition::math::equal(
      static_cast<double>(vis1->GetTransparency()), vis1Transp, 1e-5));

  gazebo::gui::ModelManipulator::Instance()->Init();

  // Time to translate vis1.
  gazebo::common::MouseEvent mouseEvent;

  mouseEvent.SetType(gazebo::common::MouseEvent::PRESS);
  mouseEvent.SetButton(gazebo::common::MouseEvent::LEFT);
  mouseEvent.SetDragging(true);
  mouseEvent.SetPressPos(0, 0);
  mouseEvent.SetPos(0, 0);

  // To set mouseStart.
  gazebo::gui::ModelManipulator::Instance()->OnMousePressEvent(mouseEvent);

  // Set mode.
  gazebo::gui::ModelManipulator::Instance()->SetManipulationMode("translate");

  // mouse moved.
  mouseEvent.SetPressPos(10, 10);
  mouseEvent.SetPos(10, 10);

  // On mouse move event.
  gazebo::gui::ModelManipulator::Instance()->SetAttachedVisual(vis1);
  gazebo::gui::ModelManipulator::Instance()->OnMouseMoveEvent(mouseEvent);

  // Verify Transparency  while the visual is being moved.
  QVERIFY(ignition::math::equal(
    static_cast<double>(vis1->GetTransparency()),
    (1.0 - vis1Transp) * 0.5, 1e-5));

  mouseEvent.SetType(gazebo::common::MouseEvent::RELEASE);
  mouseEvent.SetButton(gazebo::common::MouseEvent::NO_BUTTON);

  // Mouse release, translation done.
  gazebo::gui::ModelManipulator::Instance()->OnMouseReleaseEvent(mouseEvent);

  // Test transparency.
  QVERIFY(ignition::math::equal(
    static_cast<double>(vis1->GetTransparency()), vis1Transp, 1e-5));

  mainWindow->close();
  delete mainWindow;
  mainWindow = nullptr;
}

// Generate a main function for the test
QTEST_MAIN(ModelManipulator_TEST)
