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

#include <memory>
#include "gazebo/common/MouseEvent.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/gui/GLWidget.hh"
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

  this->Load("worlds/empty.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Check there's no box in the left panel yet
  bool hasBox = mainWindow->HasEntityName("unit_box");
  QVERIFY(!hasBox);

  // Get scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  QVERIFY(scene != NULL);

  // Check there's no box in the scene yet
  gazebo::rendering::VisualPtr vis = scene->GetVisual("unit_box");
  QVERIFY(vis == NULL);

  // Create a model maker
  gazebo::gui::ModelMaker *modelMaker = new gazebo::gui::ModelMaker();
  QVERIFY(modelMaker != NULL);

  // Start the maker to make a box
  modelMaker->InitSimpleShape(gazebo::gui::ModelMaker::SimpleShapes::BOX);
  modelMaker->Start();

  // Check there's still no box in the left panel
  hasBox = mainWindow->HasEntityName("unit_box");
  QVERIFY(!hasBox);

  // Check there's a box in the scene -- this is the preview
  vis = scene->GetVisual("unit_box");
  QVERIFY(vis != NULL);

  // Check that the box appeared in the center of the screen
  ignition::math::Vector3d startPos = modelMaker->EntityPosition();
  QVERIFY(startPos == ignition::math::Vector3d(0, 0, 0.5));
  QVERIFY(vis->WorldPose().Pos() == startPos);

  // Mouse move
  gazebo::common::MouseEvent mouseEvent;
  mouseEvent.SetType(gazebo::common::MouseEvent::MOVE);
  modelMaker->OnMouseMove(mouseEvent);

  // Check that entity moved
  ignition::math::Vector3d pos = modelMaker->EntityPosition();
  QVERIFY(pos != startPos);
  QVERIFY(vis->WorldPose().Pos() == pos);

  // Mouse release
  mouseEvent.SetType(gazebo::common::MouseEvent::RELEASE);
  mouseEvent.SetButton(gazebo::common::MouseEvent::LEFT);
  mouseEvent.SetDragging(false);
  mouseEvent.SetPressPos(0, 0);
  mouseEvent.SetPos(0, 0);
  modelMaker->OnMouseRelease(mouseEvent);

  // Check there's no box in the scene -- the preview is gone
  vis = scene->GetVisual("unit_box");
  QVERIFY(vis == NULL);

  this->ProcessEventsAndDraw(mainWindow);

  // Check there's a box in the scene -- this is the final model
  vis = scene->GetVisual("unit_box");
  QVERIFY(vis != NULL);

  // Check the box is in the left panel
  hasBox = mainWindow->HasEntityName("unit_box");
  QVERIFY(hasBox);

  delete modelMaker;

  // Terminate
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void ModelMaker_TEST::FromFile()
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

  this->ProcessEventsAndDraw(mainWindow);

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
  QVERIFY(vis->WorldPose().Pos() == startPos);

  // Mouse move
  gazebo::common::MouseEvent mouseEvent;
  mouseEvent.SetType(gazebo::common::MouseEvent::MOVE);
  modelMaker->OnMouseMove(mouseEvent);

  // Check that entity moved
  ignition::math::Vector3d pos = modelMaker->EntityPosition();
  QVERIFY(pos != startPos);
  QVERIFY(vis->WorldPose().Pos() == pos);

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

  this->ProcessEventsAndDraw(mainWindow);

  // Check there's a box in the scene -- this is the final model
  vis = scene->GetVisual("box");
  QVERIFY(vis != NULL);

  // Check the box is in the left panel
  hasBox = mainWindow->HasEntityName("box");
  QVERIFY(hasBox);

  delete modelMaker;

  // Terminate
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void ModelMaker_TEST::FromFileWithFrameSemantics()
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

  this->ProcessEventsAndDraw(mainWindow);

  const std::string entityName = "box_with_frame_semantics";
  // Check there's no box in the left panel yet
  bool hasBox = mainWindow->HasEntityName(entityName);
  QVERIFY(!hasBox);

  // Get scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  QVERIFY(scene != NULL);

  // Check there's no box in the scene yet
  gazebo::rendering::VisualPtr vis = scene->GetVisual(entityName);
  QVERIFY(vis == NULL);

  // Create a model maker
  gazebo::gui::ModelMaker *modelMaker = new gazebo::gui::ModelMaker();
  QVERIFY(modelMaker != NULL);

  // Model data
  boost::filesystem::path path;
  path = path / TEST_PATH / "models" / (entityName + ".sdf");

  // Start the maker to make a box
  modelMaker->InitFromFile(path.string());
  modelMaker->Start();

  // Check there's still no box in the left panel
  hasBox = mainWindow->HasEntityName(entityName);
  QVERIFY(!hasBox);

  // Check there's a box in the scene -- this is the preview
  vis = scene->GetVisual(entityName);
  QVERIFY(vis != NULL);

  // Get the rendering visuals associated with sdf links and visuals
  auto link1Visual= scene->GetVisual(entityName + "::L1");
  QVERIFY(link1Visual!= NULL);
  auto link1VisVisual = scene->GetVisual(entityName + "::L1::Visual_0");
  QVERIFY(link1VisVisual!= NULL);

  auto link2Visual= scene->GetVisual(entityName + "::L2");
  QVERIFY(link1Visual!= NULL);
  auto link2VisVisual = scene->GetVisual(entityName + "::L2::Visual_0");
  QVERIFY(link2VisVisual!= NULL);

  // Check that the box appeared in the center of the screen
  ignition::math::Vector3d startPos = modelMaker->EntityPosition();
  QVERIFY(startPos == ignition::math::Vector3d(0, 0, 0.5));
  QVERIFY(vis->WorldPose().Pos() == startPos);

  // Check that the sdf link's pose
  // Note that WorldPose here returns the pose relative to the model frame. Not
  // sure if this is a bug.
  QVERIFY(link1Visual->WorldPose().Pos() ==
          ignition::math::Vector3d(0, 0, 1.5));

  QVERIFY(link2Visual->WorldPose().Pos() ==
          ignition::math::Vector3d(0, 5, 1.5));

  // Check that the sdf visual is offset
  QVERIFY(link1VisVisual->Pose().Pos() == ignition::math::Vector3d(5, 0, -1));
  QVERIFY(link2VisVisual->Pose().Pos() == ignition::math::Vector3d(5, -5, -1));

  // Mouse move
  gazebo::common::MouseEvent mouseEvent;
  mouseEvent.SetType(gazebo::common::MouseEvent::MOVE);
  modelMaker->OnMouseMove(mouseEvent);

  // Check that entity moved
  ignition::math::Vector3d pos = modelMaker->EntityPosition();
  QVERIFY(pos != startPos);
  QVERIFY(vis->WorldPose().Pos() == pos);

  // Mouse release
  mouseEvent.SetType(gazebo::common::MouseEvent::RELEASE);
  mouseEvent.SetButton(gazebo::common::MouseEvent::LEFT);
  mouseEvent.SetDragging(false);
  mouseEvent.SetPressPos(0, 0);
  mouseEvent.SetPos(0, 0);
  modelMaker->OnMouseRelease(mouseEvent);

  // Check there's no box in the scene -- the preview is gone
  vis = scene->GetVisual(entityName);
  QVERIFY(vis == NULL);

  this->ProcessEventsAndDraw(mainWindow);

  // Check there's a box in the scene -- this is the final model
  vis = scene->GetVisual(entityName);
  QVERIFY(vis != NULL);

  // Check the box is in the left panel
  hasBox = mainWindow->HasEntityName(entityName);
  QVERIFY(hasBox);

  delete modelMaker;

  // Terminate
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void ModelMaker_TEST::FromNestedModelFile()
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

  this->ProcessEventsAndDraw(mainWindow);

  // Check there's no model in the left panel yet
  bool hasModel = mainWindow->HasEntityName("model_00");
  QVERIFY(!hasModel);

  // Get scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  QVERIFY(scene != NULL);

  // Check there's no model in the scene yet
  gazebo::rendering::VisualPtr vis = scene->GetVisual("model_00");
  QVERIFY(vis == NULL);

  // Create a model maker
  gazebo::gui::ModelMaker *modelMaker = new gazebo::gui::ModelMaker();
  QVERIFY(modelMaker != NULL);

  // Model data
  boost::filesystem::path path;
  path = path / TEST_PATH / "models" / "testdb" / "deeply_nested_model" /
      "model.sdf";

  // Start the maker to make a model
  modelMaker->InitFromFile(path.string());
  modelMaker->Start();

  // Check there's still no model in the left panel
  hasModel = mainWindow->HasEntityName("model_00");
  QVERIFY(!hasModel);

  // Check there's a model in the scene -- this is the preview
  vis = scene->GetVisual("model_00");
  QVERIFY(vis != NULL);

  // check all preview visuals are loaded
  vis = scene->GetVisual("model_00::model_01");
  QVERIFY(vis != NULL);
  vis = scene->GetVisual("model_00::model_01::model_02");
  QVERIFY(vis != NULL);
  vis = scene->GetVisual("model_00::model_01::model_02::model_03");
  QVERIFY(vis != NULL);

  // Check that the model appeared in the center of the screen
  ignition::math::Vector3d startPos = modelMaker->EntityPosition();
  QVERIFY(startPos == ignition::math::Vector3d(0, 0, 0.5));
  vis = scene->GetVisual("model_00");
  QVERIFY(vis->WorldPose().Pos() == startPos);

  // Mouse move
  gazebo::common::MouseEvent mouseEvent;
  mouseEvent.SetType(gazebo::common::MouseEvent::MOVE);
  modelMaker->OnMouseMove(mouseEvent);

  // Check that entity moved
  ignition::math::Vector3d pos = modelMaker->EntityPosition();
  QVERIFY(pos != startPos);
  QVERIFY(vis->WorldPose().Pos() == pos);

  // Mouse release
  mouseEvent.SetType(gazebo::common::MouseEvent::RELEASE);
  mouseEvent.SetButton(gazebo::common::MouseEvent::LEFT);
  mouseEvent.SetDragging(false);
  mouseEvent.SetPressPos(0, 0);
  mouseEvent.SetPos(0, 0);
  modelMaker->OnMouseRelease(mouseEvent);

  // Check there's no model in the scene -- the preview is gone
  vis = scene->GetVisual("model_00");
  QVERIFY(vis == NULL);
  vis = scene->GetVisual("model_00::model_01");
  QVERIFY(vis == NULL);
  vis = scene->GetVisual("model_00::model_01::model_02");
  QVERIFY(vis == NULL);
  vis = scene->GetVisual("model_00::model_01::model_02::model_03");
  QVERIFY(vis == NULL);

  this->ProcessEventsAndDraw(mainWindow);

  // Check there's a model in the scene -- this is the final model
  vis = scene->GetVisual("model_00");
  QVERIFY(vis != NULL);
  vis = scene->GetVisual("model_00::model_01");
  QVERIFY(vis != NULL);
  vis = scene->GetVisual("model_00::model_01::model_02");
  QVERIFY(vis != NULL);
  vis = scene->GetVisual("model_00::model_01::model_02::model_03");
  QVERIFY(vis != NULL);

  // Check the model is in the left panel
  hasModel = mainWindow->HasEntityName("model_00");
  QVERIFY(hasModel);

  delete modelMaker;

  // Terminate
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void ModelMaker_TEST::FromModel()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/box.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

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
  QVERIFY(vis->WorldPose().Pos() == startPos);

  // Mouse move
  gazebo::common::MouseEvent mouseEvent;
  mouseEvent.SetType(gazebo::common::MouseEvent::MOVE);
  modelMaker->OnMouseMove(mouseEvent);

  // Check that entity moved
  ignition::math::Vector3d pos = modelMaker->EntityPosition();
  QVERIFY(pos != startPos);
  QVERIFY(vis->WorldPose().Pos() == pos);

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

  this->ProcessEventsAndDraw(mainWindow);

  // Check there's a clone in the scene -- this is the final model
  vis = scene->GetVisual("box_clone");
  QVERIFY(vis != NULL);

  // Check the clone is in the left panel
  hasModel = mainWindow->HasEntityName("box_clone");
  QVERIFY(hasModel);

  delete modelMaker;

  // Terminate
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void ModelMaker_TEST::FromNestedModel()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("test/worlds/deeply_nested_models.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Check there's a model but not its copy
  bool hasModel = mainWindow->HasEntityName("model_00");
  QVERIFY(hasModel);
  hasModel = mainWindow->HasEntityName("model_00_clone");
  QVERIFY(!hasModel);

  // Get scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  QVERIFY(scene != NULL);

  // Check there's a model but no clone in the scene yet
  gazebo::rendering::VisualPtr vis = scene->GetVisual("model_00");
  QVERIFY(vis != NULL);
  vis = scene->GetVisual("model_00_clone_tmp");
  QVERIFY(vis == NULL);
  vis = scene->GetVisual("model_00_clone");
  QVERIFY(vis == NULL);

  // check all nested model visuals are there
  vis = scene->GetVisual("model_00::model_01");
  QVERIFY(vis != NULL);
  vis = scene->GetVisual("model_00::model_01::model_02");
  QVERIFY(vis != NULL);
  vis = scene->GetVisual("model_00::model_01::model_02::model_03");
  QVERIFY(vis != NULL);

  // Create a model maker
  gazebo::gui::ModelMaker *modelMaker = new gazebo::gui::ModelMaker();
  QVERIFY(modelMaker != NULL);

  // Start the maker to copy the model
  modelMaker->InitFromModel("model_00");
  modelMaker->Start();

  // Check there's still no clone in the left panel
  hasModel = mainWindow->HasEntityName("model_00_clone_tmp");
  QVERIFY(!hasModel);
  hasModel = mainWindow->HasEntityName("model_00_clone");
  QVERIFY(!hasModel);

  // Check there's a clone in the scene -- this is the preview
  vis = scene->GetVisual("model_00_clone");
  QVERIFY(vis == NULL);
  vis = scene->GetVisual("model_00_clone_tmp");
  QVERIFY(vis != NULL);

  vis = scene->GetVisual("model_00_clone_tmp::model_01");
  QVERIFY(vis != NULL);
  vis = scene->GetVisual("model_00_clone_tmp::model_01::model_02");
  QVERIFY(vis != NULL);
  vis = scene->GetVisual("model_00_clone_tmp::model_01::model_02::model_03");
  QVERIFY(vis != NULL);

  // Check that the clone appeared in the center of the screen
  ignition::math::Vector3d startPos = modelMaker->EntityPosition();
  QVERIFY(startPos == ignition::math::Vector3d(0, 0, 0.5));
  vis = scene->GetVisual("model_00_clone_tmp");
  QVERIFY(vis->WorldPose().Pos() == startPos);

  // Mouse move
  gazebo::common::MouseEvent mouseEvent;
  mouseEvent.SetType(gazebo::common::MouseEvent::MOVE);
  modelMaker->OnMouseMove(mouseEvent);

  // Check that entity moved
  ignition::math::Vector3d pos = modelMaker->EntityPosition();
  QVERIFY(pos != startPos);
  QVERIFY(vis->WorldPose().Pos() == pos);

  // Mouse release
  mouseEvent.SetType(gazebo::common::MouseEvent::RELEASE);
  mouseEvent.SetButton(gazebo::common::MouseEvent::LEFT);
  mouseEvent.SetDragging(false);
  mouseEvent.SetPressPos(0, 0);
  mouseEvent.SetPos(0, 0);
  modelMaker->OnMouseRelease(mouseEvent);

  // Check there's no clone in the scene -- the preview is gone
  vis = scene->GetVisual("model_00_clone_tmp");
  QVERIFY(vis == nullptr);
  vis = scene->GetVisual("model_00_clone_tmp::model_01");
  QVERIFY(vis == nullptr);
  vis = scene->GetVisual("model_00_clone_tmp::model_01::model_02");
  QVERIFY(vis == nullptr);
  vis = scene->GetVisual("model_00_clone_tmp::model_01::model_02::model_03");
  QVERIFY(vis == nullptr);

  this->ProcessEventsAndDraw(mainWindow);

  // Check there's a clone in the scene -- this is the final model
  vis = scene->GetVisual("model_00_clone");
  QVERIFY(vis != NULL);
  vis = scene->GetVisual("model_00_clone::model_01");
  QVERIFY(vis != NULL);
  vis = scene->GetVisual("model_00_clone::model_01::model_02");
  QVERIFY(vis != NULL);
  vis = scene->GetVisual("model_00_clone::model_01::model_02::model_03");
  QVERIFY(vis != NULL);

  // Check the clone is in the left panel
  hasModel = mainWindow->HasEntityName("model_00_clone");
  QVERIFY(hasModel);

  delete modelMaker;

  // Terminate
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void ModelMaker_TEST::FromNestedModelFileWithFrameSemantics()
{
  using ignition::math::Pose3d;
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", true, false, false);

  // Create the main window.
  auto mainWindow = std::make_unique<gazebo::gui::MainWindow>();
  QVERIFY(mainWindow != nullptr);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow.get());

  // Check there's no model in the left panel of the main window yet
  QVERIFY(!mainWindow->HasEntityName("model_00"));

  // Get scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  QVERIFY(scene != nullptr);

  // Check there's no model in the scene yet
  {
    gazebo::rendering::VisualPtr model00Vis = scene->GetVisual("model_00");
    QVERIFY(model00Vis == nullptr);
  }

  // Create a model maker
  auto modelMaker = std::make_unique<gazebo::gui::ModelMaker>();
  QVERIFY(modelMaker != nullptr);

  // Model data
  boost::filesystem::path path;
  path = path / TEST_PATH / "models" / "testdb" /
         "deeply_nested_model_with_frame_semantics" / "model.sdf";

  // Start the maker to make a model
  modelMaker->InitFromFile(path.string());
  modelMaker->Start();

  // Check there's still no model in the left panel of the main window
  QVERIFY(!mainWindow->HasEntityName("model_00"));

  ignition::math::Vector3d startPos = modelMaker->EntityPosition();

  // Check there's a model in the scene -- this is the preview
  {
    auto model00 = scene->GetVisual("model_00");
    QVERIFY(model00 != nullptr);
    // Check that the model appeared in the center of the screen
    QVERIFY(startPos == ignition::math::Vector3d(0, 0, 0.5));
  }

  // check all preview visuals are loaded
  {
    auto model01 = scene->GetVisual("model_00::model_01");
    QVERIFY(model01 != nullptr);
    QVERIFY(Pose3d(1.25, 0, 1.5, 0, 0, 0) == model01->WorldPose());
  }
  {
    auto model02 = scene->GetVisual("model_00::model_01::model_02");
    QVERIFY(model02 != nullptr);
    QVERIFY(Pose3d(1.5, 2, 2.5, 0, 0, 0) == model02->WorldPose());
  }
  {
    auto model03 = scene->GetVisual("model_00::model_01::model_02::model_03");
    QVERIFY(model03 != nullptr);
    QVERIFY(Pose3d(1.75, 2, 6.5, 0, 0, 0) == model03->WorldPose());
  }

  // Mouse move
  gazebo::common::MouseEvent mouseEvent;
  mouseEvent.SetType(gazebo::common::MouseEvent::MOVE);
  modelMaker->OnMouseMove(mouseEvent);

  // Check that entity moved
  Pose3d newPose;
  newPose.Pos() = modelMaker->EntityPosition();
  {
    auto model00 = scene->GetVisual("model_00");
    QVERIFY(model00 != nullptr);
    QVERIFY(newPose.Pos() != startPos);
    QVERIFY(model00->WorldPose() == newPose);
  }

  // Mouse release
  mouseEvent.SetType(gazebo::common::MouseEvent::RELEASE);
  mouseEvent.SetButton(gazebo::common::MouseEvent::LEFT);
  mouseEvent.SetDragging(false);
  mouseEvent.SetPressPos(0, 0);
  mouseEvent.SetPos(0, 0);
  modelMaker->OnMouseRelease(mouseEvent);

  // Check there's no model in the scene -- the preview is gone
  QVERIFY(nullptr == scene->GetVisual("model_00"));
  QVERIFY(nullptr == scene->GetVisual("model_00::model_01"));
  QVERIFY(nullptr == scene->GetVisual("model_00::model_01::model_02"));
  QVERIFY(nullptr ==
          scene->GetVisual("model_00::model_01::model_02::model_03"));

  this->ProcessEventsAndDraw(mainWindow.get(), 30);
  // Check the model is in the left panel
  QVERIFY(mainWindow->HasEntityName("model_00"));

  // Check there's a model in the scene -- this is the final model
  // Also check the new poses of the created visuals
  {
    auto model00 = scene->GetVisual("model_00");
    QVERIFY(model00 != nullptr);
    QVERIFY(newPose == model00->WorldPose());
  }

  // The expected poses are calculated as offset from newPose. However,
  // since newPose already includes the [0 0 0.5] position of model_00, the
  // that translation is removed from the offsets.
  {
    auto model01 = scene->GetVisual("model_00::model_01");
    QVERIFY(model01 != nullptr);
    QVERIFY(newPose * Pose3d(1.25, 0, 1.0, 0, 0, 0) == model01->WorldPose());
  }
  {
    auto model02 = scene->GetVisual("model_00::model_01::model_02");
    QVERIFY(model02 != nullptr);
    QVERIFY(newPose * Pose3d(1.5, 2, 2.0, 0, 0, 0) == model02->WorldPose());
  }
  {
    auto model03 = scene->GetVisual("model_00::model_01::model_02::model_03");
    QVERIFY(model03 != nullptr);
    QVERIFY(newPose * Pose3d(1.75, 2, 6.0, 0, 0, 0) == model03->WorldPose());
  }
  // Terminate
  mainWindow->close();
}
// Generate a main function for the test
QTEST_MAIN(ModelMaker_TEST)
