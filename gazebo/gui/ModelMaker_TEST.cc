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

#include "gazebo/msgs/msgs.hh"

#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"

#include "gazebo/gui/ModelMaker.hh"
#include "gazebo/gui/ModelMaker_TEST.hh"

/////////////////////////////////////////////////
void ModelMaker_TEST::SimpleShape()
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

  // Check there's no box in the left panel yet
  bool hasBox = mainWindow->HasEntityName("unit_box_0");
  QVERIFY(!hasBox);

  // Get scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  QVERIFY(scene != NULL);

  // Check there's no box in the scene yet
  gazebo::rendering::VisualPtr vis = scene->GetVisual("unit_box_0");
  QVERIFY(vis == NULL);

  // Create a model maker
  gazebo::gui::ModelMaker *modelMaker = new gazebo::gui::ModelMaker();
  QVERIFY(modelMaker != NULL);

  // Start the maker to make a box
  modelMaker->InitSimpleShape(gazebo::gui::ModelMaker::SimpleShapes::BOX);
  modelMaker->Start();

  // Check there's still no box in the left panel
  hasBox = mainWindow->HasEntityName("unit_box_0");
  QVERIFY(!hasBox);

  // Check there's a box in the scene -- this is the preview
  vis = scene->GetVisual("unit_box_0");
  QVERIFY(vis != NULL);

  // Check that the box appeared in the center of the screen
  ignition::math::Vector3d startPos = modelMaker->EntityPosition();
  QVERIFY(startPos == ignition::math::Vector3d(0, 0, 0.5));
  QVERIFY(vis->GetWorldPose().pos == startPos);

  // Mouse move
  gazebo::common::MouseEvent mouseEvent;
  mouseEvent.SetType(gazebo::common::MouseEvent::MOVE);
  modelMaker->OnMouseMove(mouseEvent);

  // Check that entity moved
  ignition::math::Vector3d pos = modelMaker->EntityPosition();
  QVERIFY(pos != startPos);
  QVERIFY(vis->GetWorldPose().pos == pos);

  // Mouse release
  mouseEvent.SetType(gazebo::common::MouseEvent::RELEASE);
  mouseEvent.SetButton(gazebo::common::MouseEvent::LEFT);
  mouseEvent.SetDragging(false);
  mouseEvent.SetPressPos(0, 0);
  mouseEvent.SetPos(0, 0);
  modelMaker->OnMouseRelease(mouseEvent);

  // Check there's no box in the scene -- the preview is gone
  vis = scene->GetVisual("unit_box_0");
  QVERIFY(vis == NULL);

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check there's a box in the scene -- this is the final model
  vis = scene->GetVisual("unit_box_0");
  QVERIFY(vis != NULL);

  // Check the box is in the left panel
  hasBox = mainWindow->HasEntityName("unit_box_0");
  QVERIFY(hasBox);

  // Terminate
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void ModelMaker_TEST::FromFile()
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

  // Check there's no box in the left panel yet
  bool hasBox = mainWindow->HasEntityName("box");
  QVERIFY(!hasBox);

  // Get scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  QVERIFY(scene != NULL);

  // Check there's no box in the scene yet
  gazebo::rendering::VisualPtr vis = scene->GetVisual("box");
  QVERIFY(vis == NULL);

  // Create a model maker
  gazebo::gui::ModelMaker *modelMaker = new gazebo::gui::ModelMaker();
  QVERIFY(modelMaker != NULL);

  // Model data
  boost::filesystem::path path;
  path = path / TEST_PATH / "models" / "box.sdf";

  // Start the maker to make a box
  modelMaker->InitFromFile(path.string());
  modelMaker->Start();

  // Check there's still no box in the left panel
  hasBox = mainWindow->HasEntityName("box");
  QVERIFY(!hasBox);

  // Check there's a box in the scene -- this is the preview
  vis = scene->GetVisual("box");
  QVERIFY(vis != NULL);

  // Check that the box appeared in the center of the screen
  ignition::math::Vector3d startPos = modelMaker->EntityPosition();
  QVERIFY(startPos == ignition::math::Vector3d(0, 0, 0.5));
  QVERIFY(vis->GetWorldPose().pos == startPos);

  // Mouse move
  gazebo::common::MouseEvent mouseEvent;
  mouseEvent.SetType(gazebo::common::MouseEvent::MOVE);
  modelMaker->OnMouseMove(mouseEvent);

  // Check that entity moved
  ignition::math::Vector3d pos = modelMaker->EntityPosition();
  QVERIFY(pos != startPos);
  QVERIFY(vis->GetWorldPose().pos == pos);

  // Mouse release
  mouseEvent.SetType(gazebo::common::MouseEvent::RELEASE);
  mouseEvent.SetButton(gazebo::common::MouseEvent::LEFT);
  mouseEvent.SetDragging(false);
  mouseEvent.SetPressPos(0, 0);
  mouseEvent.SetPos(0, 0);
  modelMaker->OnMouseRelease(mouseEvent);

  // Check there's no box in the scene -- the preview is gone
  vis = scene->GetVisual("box");
  QVERIFY(vis == NULL);

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check there's a box in the scene -- this is the final model
  vis = scene->GetVisual("box");
  QVERIFY(vis != NULL);

  // Check the box is in the left panel
  hasBox = mainWindow->HasEntityName("box");
  QVERIFY(hasBox);

  // Terminate
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void ModelMaker_TEST::FromModel()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/box.world");

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

  // Check there's a model but not its copy
  bool hasModel = mainWindow->HasEntityName("box");
  QVERIFY(hasModel);
  hasModel = mainWindow->HasEntityName("box_clone");
  QVERIFY(!hasModel);

  // Get scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  QVERIFY(scene != NULL);

  // Check there's a model but no clone in the scene yet
  gazebo::rendering::VisualPtr vis = scene->GetVisual("box");
  QVERIFY(vis != NULL);
  vis = scene->GetVisual("box_clone_tmp");
  QVERIFY(vis == NULL);
  vis = scene->GetVisual("box_clone");
  QVERIFY(vis == NULL);

  // Create a model maker
  gazebo::gui::ModelMaker *modelMaker = new gazebo::gui::ModelMaker();
  QVERIFY(modelMaker != NULL);

  // Start the maker to copy the model
  modelMaker->InitFromModel("box");
  modelMaker->Start();

  // Check there's still no clone in the left panel
  hasModel = mainWindow->HasEntityName("box_clone_tmp");
  QVERIFY(!hasModel);
  hasModel = mainWindow->HasEntityName("box_clone");
  QVERIFY(!hasModel);

  // Check there's a clone in the scene -- this is the preview
  vis = scene->GetVisual("box_clone");
  QVERIFY(vis == NULL);
  vis = scene->GetVisual("box_clone_tmp");
  QVERIFY(vis != NULL);

  // Check that the clone appeared in the center of the screen
  ignition::math::Vector3d startPos = modelMaker->EntityPosition();
  QVERIFY(startPos == ignition::math::Vector3d(0, 0, 0.5));
  QVERIFY(vis->GetWorldPose().pos == startPos);

  // Mouse move
  gazebo::common::MouseEvent mouseEvent;
  mouseEvent.SetType(gazebo::common::MouseEvent::MOVE);
  modelMaker->OnMouseMove(mouseEvent);

  // Check that entity moved
  ignition::math::Vector3d pos = modelMaker->EntityPosition();
  QVERIFY(pos != startPos);
  QVERIFY(vis->GetWorldPose().pos == pos);

  // Mouse release
  mouseEvent.SetType(gazebo::common::MouseEvent::RELEASE);
  mouseEvent.SetButton(gazebo::common::MouseEvent::LEFT);
  mouseEvent.SetDragging(false);
  mouseEvent.SetPressPos(0, 0);
  mouseEvent.SetPos(0, 0);
  modelMaker->OnMouseRelease(mouseEvent);

  // Check there's no clone in the scene -- the preview is gone
  vis = scene->GetVisual("box_clone_tmp");
  QVERIFY(vis == NULL);

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check there's a clone in the scene -- this is the final model
  vis = scene->GetVisual("box_clone");
  QVERIFY(vis != NULL);

  // Check the clone is in the left panel
  hasModel = mainWindow->HasEntityName("box_clone");
  QVERIFY(hasModel);

  // Terminate
  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(ModelMaker_TEST)
