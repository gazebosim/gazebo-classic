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
void ModelCreator_TEST::NestedModel()
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

  // Create a model creator
  gui::ModelCreator *modelCreator = new gui::ModelCreator();
  QVERIFY(modelCreator != NULL);

  // Create a box model and add it to the model crerator
  double mass = 1.0;
  ignition::math::Vector3d size = ignition::math::Vector3d::One;
  msgs::Model model;
  model.set_name("box_model");
  msgs::AddBoxLink(model, mass, size);
  sdf::ElementPtr boxModelSDF = msgs::ModelToSDF(model);

  modelCreator->AddEntity(boxModelSDF);

  // Verify it has been added
  gazebo::rendering::VisualPtr boxModelVis =
      scene->GetVisual("ModelPreview_0::box_model");
  QVERIFY(boxModelVis != NULL);

  // add a nested model - two top level links, one joint, and a nested model
  std::ostringstream sdfStream;
  sdfStream << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='model_00'>"
    << "  <pose>0 0 1 0 0 0</pose>"
    << "  <link name ='link_00'>"
    << "    <pose>1 0 0 0 0 0</pose>"
    << "    <collision name ='collision_01'>"
    << "      <geometry>"
    << "        <box><size>1 1 1</size></box>"
    << "      </geometry>"
    << "    </collision>"
    << "    <visual name ='visual_01'>"
    << "      <geometry>"
    << "        <box><size>1 1 1</size></box>"
    << "      </geometry>"
    << "    </visual>"
    << "  </link>"
    << "  <link name ='link_01'>"
    << "    <pose>-1 0 0 0 0 0</pose>"
    << "    <collision name ='collision_02'>"
    << "      <geometry>"
    << "        <cylinder>"
    << "          <radius>0.5</radius>"
    << "          <length>1.0</length>"
    << "        </cylinder>"
    << "      </geometry>"
    << "    </collision>"
    << "    <visual name ='visual_02'>"
    << "      <geometry>"
    << "        <cylinder>"
    << "          <radius>0.5</radius>"
    << "          <length>1.0</length>"
    << "        </cylinder>"
    << "      </geometry>"
    << "    </visual>"
    << "  </link>"
    << "  <joint name ='joint_01' type='prismatic'>"
    << "    <pose>0 1 0 0 0 0</pose>"
    << "    <parent>link_00</parent>"
    << "    <child>link_01</child>"
    << "    <axis>"
    << "      <xyz>0 0 1</xyz>"
    << "    </axis>"
    << "  </joint>"
    << "  <model name ='model_01'>"
    << "    <pose>0 1 0 0 0 0</pose>"
    << "    <link name ='link_01'>"
    << "      <pose>1 0 0 0 0 0</pose>"
    << "      <collision name ='collision_01'>"
    << "        <geometry>"
    << "          <box><size>1 1 1</size></box>"
    << "        </geometry>"
    << "      </collision>"
    << "      <visual name ='visual_01'>"
    << "        <geometry>"
    << "          <box><size>1 1 1</size></box>"
    << "        </geometry>"
    << "      </visual>"
    << "     </link>"
    << "  </model>"
    << "</model>"
    << "</sdf>";

  sdf::ElementPtr modelSDF(new sdf::Element);
  sdf::initFile("model.sdf", modelSDF);
  sdf::readString(sdfStream.str(), modelSDF);
  modelCreator->AddEntity(modelSDF);

  // verify the model with joint has been added
  gazebo::rendering::VisualPtr modelVis =
      scene->GetVisual("ModelPreview_0::model_00");
  QVERIFY(modelVis != NULL);
  gazebo::rendering::VisualPtr link00Vis =
      scene->GetVisual("ModelPreview_0::model_00::link_00");
  QVERIFY(link00Vis != NULL);
  gazebo::rendering::VisualPtr link01Vis =
      scene->GetVisual("ModelPreview_0::model_00::link_01");
  QVERIFY(link01Vis != NULL);
  gazebo::rendering::VisualPtr model01Vis =
      scene->GetVisual("ModelPreview_0::model_00::model_01");
  QVERIFY(model01Vis != NULL);

  // remove box model and verify
  modelCreator->RemoveEntity(boxModelVis->GetName());
  boxModelVis = scene->GetVisual("ModelPreview_0::box_model");
  QVERIFY(boxModelVis == NULL);

  // remove nested model and verify
  modelCreator->RemoveEntity(modelVis->GetName());
  modelVis = scene->GetVisual("ModelPreview_0::model_00");
  QVERIFY(modelVis == NULL);
  link00Vis = scene->GetVisual("ModelPreview_0::model_00::link_00");
  QVERIFY(link00Vis == NULL);
  link01Vis = scene->GetVisual("ModelPreview_0::model_00::link_01");
  QVERIFY(link01Vis == NULL);
  model01Vis = scene->GetVisual("ModelPreview_0::model_00::model_01");
  QVERIFY(model01Vis == NULL);

  delete modelCreator;
  modelCreator = NULL;

  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

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
  modelCreator->AddShape(gui::ModelCreator::ENTITY_CYLINDER);
  gazebo::rendering::VisualPtr cylinder =
      scene->GetVisual("ModelPreview_0::ENTITY_0");
  QVERIFY(cylinder != NULL);
  QCOMPARE(modelCreator->GetCurrentSaveState(),
      gui::ModelCreator::NEVER_SAVED);

  // Save all changes
  modelCreator->SaveModelFiles();
  QCOMPARE(modelCreator->GetCurrentSaveState(),
      gui::ModelCreator::ALL_SAVED);

  // Insert another link to have unsaved changes
  modelCreator->AddShape(gui::ModelCreator::ENTITY_BOX);
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
  modelCreator->RemoveEntity(cylinder->GetName());
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
  modelCreator->AddShape(gui::ModelCreator::ENTITY_CYLINDER);
  gazebo::rendering::VisualPtr cylinder =
      scene->GetVisual("ModelPreview_0::ENTITY_0");
  QVERIFY(cylinder != NULL);

  modelCreator->AddShape(gui::ModelCreator::ENTITY_BOX);
  gazebo::rendering::VisualPtr box =
      scene->GetVisual("ModelPreview_0::ENTITY_1");
  QVERIFY(box != NULL);

  modelCreator->AddShape(gui::ModelCreator::ENTITY_SPHERE);
  gazebo::rendering::VisualPtr sphere =
      scene->GetVisual("ModelPreview_0::ENTITY_2");
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
