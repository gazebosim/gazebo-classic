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

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/ModelCreator.hh"
#include "gazebo/gui/model/ModelCreator_TEST.hh"

using namespace gazebo;

/////////////////////////////////////////////////
void ModelCreator_TEST::SaveState()
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

  // Start never saved
  gui::ModelCreator *modelCreator = new gui::ModelCreator();
  QCOMPARE(modelCreator->GetCurrentSaveState(), gui::ModelCreator::NEVER_SAVED);

  // Inserting a link and it still is never saved
  modelCreator->AddShape(gui::ModelCreator::LINK_CYLINDER);
  gazebo::rendering::VisualPtr cylinder =
      scene->GetVisual("ModelPreview_0::link_0");
  QVERIFY(cylinder != NULL);
  QCOMPARE(modelCreator->GetCurrentSaveState(),
      gui::ModelCreator::NEVER_SAVED);

  // Save all changes
  modelCreator->SaveModelFiles();
  QCOMPARE(modelCreator->GetCurrentSaveState(),
      gui::ModelCreator::ALL_SAVED);

  // Insert another link to have unsaved changes
  modelCreator->AddShape(gui::ModelCreator::LINK_BOX);
  QCOMPARE(modelCreator->GetCurrentSaveState(),
      gui::ModelCreator::UNSAVED_CHANGES);

  // Save all changes
  modelCreator->SaveModelFiles();
  QCOMPARE(modelCreator->GetCurrentSaveState(),
      gui::ModelCreator::ALL_SAVED);

  // Move a link to have unsaved changes
  cylinder->SetWorldPose(math::Pose(1, 2, 3, 4, 5, 6));
  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }
  QCOMPARE(modelCreator->GetCurrentSaveState(),
      gui::ModelCreator::UNSAVED_CHANGES);

  // Save all changes
  modelCreator->SaveModelFiles();
  QCOMPARE(modelCreator->GetCurrentSaveState(),
      gui::ModelCreator::ALL_SAVED);

  // Remove a link to have unsaved changes
  modelCreator->RemoveLink(cylinder->GetName());
  QCOMPARE(modelCreator->GetCurrentSaveState(),
      gui::ModelCreator::UNSAVED_CHANGES);

  // Save all changes
  modelCreator->SaveModelFiles();
  QCOMPARE(modelCreator->GetCurrentSaveState(),
      gui::ModelCreator::ALL_SAVED);

  delete modelCreator;
  modelCreator = NULL;
  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void ModelCreator_TEST::Selection()
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

  // Start never saved
  gui::ModelCreator *modelCreator = new gui::ModelCreator();
  QVERIFY(modelCreator);

  // Inserting a few links
  modelCreator->AddShape(gui::ModelCreator::LINK_CYLINDER);
  gazebo::rendering::VisualPtr cylinder =
      scene->GetVisual("ModelPreview_0::link_0");
  QVERIFY(cylinder != NULL);

  modelCreator->AddShape(gui::ModelCreator::LINK_BOX);
  gazebo::rendering::VisualPtr box =
      scene->GetVisual("ModelPreview_0::link_1");
  QVERIFY(box != NULL);

  modelCreator->AddShape(gui::ModelCreator::LINK_SPHERE);
  gazebo::rendering::VisualPtr sphere =
      scene->GetVisual("ModelPreview_0::link_2");
  QVERIFY(sphere != NULL);

  // verify initial selected state
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(!box->GetHighlighted());
  QVERIFY(!sphere->GetHighlighted());

  // select the shapes and verify that they are selected
  modelCreator->SetSelected(cylinder, true);
  QVERIFY(cylinder->GetHighlighted());

  modelCreator->SetSelected(box, true);
  QVERIFY(box->GetHighlighted());

  modelCreator->SetSelected(sphere, true);
  QVERIFY(sphere->GetHighlighted());

  // deselect and verify
  modelCreator->SetSelected(cylinder, false);
  QVERIFY(!cylinder->GetHighlighted());

  modelCreator->SetSelected(box, false);
  QVERIFY(!box->GetHighlighted());

  modelCreator->SetSelected(sphere, false);
  QVERIFY(!sphere->GetHighlighted());

  // select one and verify all
  modelCreator->SetSelected(cylinder, true);
  QVERIFY(cylinder->GetHighlighted());
  QVERIFY(!box->GetHighlighted());
  QVERIFY(!sphere->GetHighlighted());

  delete modelCreator;
  modelCreator = NULL;
  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

// Generate a main function for the test
QTEST_MAIN(ModelCreator_TEST)
