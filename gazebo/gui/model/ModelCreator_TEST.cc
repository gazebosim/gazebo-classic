/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/model/ModelData.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/ModelCreator.hh"
#include "gazebo/gui/model/ModelCreator_TEST.hh"
#include "sdf/sdf.hh"

using namespace gazebo;

/////////////////////////////////////////////////
std::string GetNestedModelSDFString()
{
  // nested model - two top level links, one joint, and a nested model
  std::stringstream nestedModelSdfStream;
  nestedModelSdfStream << "<sdf version='" << SDF_VERSION << "'>"
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
  return nestedModelSdfStream.str();
}

/////////////////////////////////////////////////
void ModelCreator_TEST::NestedModel()
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

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  // Create a model creator
  gui::ModelCreator *modelCreator = new gui::ModelCreator();
  QVERIFY(modelCreator != NULL);

  // Create a box model and add it to the model creator
  double mass = 1.0;
  ignition::math::Vector3d size = ignition::math::Vector3d::One;
  msgs::Model model;
  model.set_name("box_model");
  msgs::AddBoxLink(model, mass, size);
  sdf::ElementPtr boxModelSDF = msgs::ModelToSDF(model);

  modelCreator->AddEntity(boxModelSDF);

  // Verify it has been added
  gazebo::rendering::VisualPtr boxModelVis =
      scene->GetVisual("ModelPreview_0_0::box_model");
  QVERIFY(boxModelVis != NULL);

  // test loading nested model from sdf
  sdf::ElementPtr modelSDF(new sdf::Element);
  sdf::initFile("model.sdf", modelSDF);
  sdf::readString(GetNestedModelSDFString(), modelSDF);
  modelCreator->AddEntity(modelSDF);

  // verify the model with joint has been added
  gazebo::rendering::VisualPtr modelVis =
      scene->GetVisual("ModelPreview_0_0::model_00");
  QVERIFY(modelVis != NULL);
  gazebo::rendering::VisualPtr link00Vis =
      scene->GetVisual("ModelPreview_0_0::model_00::link_00");
  QVERIFY(link00Vis != NULL);
  gazebo::rendering::VisualPtr link01Vis =
      scene->GetVisual("ModelPreview_0_0::model_00::link_01");
  QVERIFY(link01Vis != NULL);
  gazebo::rendering::VisualPtr model01Vis =
      scene->GetVisual("ModelPreview_0_0::model_00::model_01");
  QVERIFY(model01Vis != NULL);

  // remove box model and verify
  modelCreator->RemoveEntity(boxModelVis->GetName());
  boxModelVis = scene->GetVisual("ModelPreview_0_0::box_model");
  QVERIFY(boxModelVis == NULL);

  // remove nested model and verify
  modelCreator->RemoveEntity(modelVis->GetName());
  modelVis = scene->GetVisual("ModelPreview_0_0::model_00");
  QVERIFY(modelVis == NULL);
  link00Vis = scene->GetVisual("ModelPreview_0_0::model_00::link_00");
  QVERIFY(link00Vis == NULL);
  link01Vis = scene->GetVisual("ModelPreview_0_0::model_00::link_01");
  QVERIFY(link01Vis == NULL);
  model01Vis = scene->GetVisual("ModelPreview_0_0::model_00::model_01");
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

  this->Load("worlds/empty.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

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
      scene->GetVisual("ModelPreview_0_0::link_0");
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

  this->ProcessEventsAndDraw(mainWindow);

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

  this->Load("worlds/empty.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

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
      scene->GetVisual("ModelPreview_0_0::link_0");
  QVERIFY(cylinder != NULL);

  modelCreator->AddShape(gui::ModelCreator::ENTITY_BOX);
  gazebo::rendering::VisualPtr box =
      scene->GetVisual("ModelPreview_0_0::link_1");
  QVERIFY(box != NULL);

  modelCreator->AddShape(gui::ModelCreator::ENTITY_SPHERE);
  gazebo::rendering::VisualPtr sphere =
      scene->GetVisual("ModelPreview_0_0::link_2");
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

/////////////////////////////////////////////////
void ModelCreator_TEST::ModelPlugin()
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
      scene->GetVisual("ModelPreview_0_0::link_0");
  QVERIFY(cylinder != NULL);

  // add model plugin
  modelCreator->OnAddModelPlugin("test_name", "test_filename",
      "<data>test</data>");
  gazebo::gui::ModelPluginData *modelPluginData =
       modelCreator->ModelPlugin("test_name");
  QVERIFY(modelPluginData != NULL);
  sdf::ElementPtr modelPluginSDF = modelPluginData->modelPluginSDF;
  QCOMPARE(modelPluginSDF->Get<std::string>("name"), std::string("test_name"));
  QCOMPARE(modelPluginSDF->Get<std::string>("filename"),
      std::string("test_filename"));
  QVERIFY(modelPluginSDF->HasElement("data"));
  QCOMPARE(modelPluginSDF->Get<std::string>("data"), std::string("test"));

  // remove the model plugin
  modelCreator->RemoveModelPlugin("test_name");
  modelPluginData = modelCreator->ModelPlugin("test_name");
  QVERIFY(modelPluginData == NULL);

  delete modelCreator;
  modelCreator = NULL;
  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void ModelCreator_TEST::NestedModelSelection()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, true);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  // Create a model creator
  gui::ModelCreator *modelCreator = new gui::ModelCreator();
  QVERIFY(modelCreator != NULL);

  // a link
  modelCreator->AddShape(gui::ModelCreator::ENTITY_CYLINDER);
  gazebo::rendering::VisualPtr cylinder =
      scene->GetVisual("ModelPreview_0_0::link_0");
  QVERIFY(cylinder != NULL);

  // Add various models and links into the editor
  // a box nested model
  double mass = 1.0;
  ignition::math::Vector3d size = ignition::math::Vector3d::One;
  msgs::Model model;
  model.set_name("box_model");
  msgs::AddBoxLink(model, mass, size);
  sdf::ElementPtr boxModelSDF = msgs::ModelToSDF(model);

  modelCreator->AddModel(boxModelSDF);

  /// Verify it has been added
  gazebo::rendering::VisualPtr boxModelVis =
      scene->GetVisual("ModelPreview_0_0::box_model");
  QVERIFY(boxModelVis != NULL);

  this->ProcessEventsAndDraw(mainWindow);

  // a more complicated a nested model loaded from from sdf
  sdf::ElementPtr modelSDF(new sdf::Element);
  sdf::initFile("model.sdf", modelSDF);
  sdf::readString(GetNestedModelSDFString(), modelSDF);
  modelCreator->AddModel(modelSDF);

  this->ProcessEventsAndDraw(mainWindow);

  // verify the model has been added
  gazebo::rendering::VisualPtr modelVis =
      scene->GetVisual("ModelPreview_0_0::model_00");
  QVERIFY(modelVis != NULL);
  gazebo::rendering::VisualPtr link00Vis =
      scene->GetVisual("ModelPreview_0_0::model_00::link_00");
  QVERIFY(link00Vis != NULL);
  gazebo::rendering::VisualPtr link01Vis =
      scene->GetVisual("ModelPreview_0_0::model_00::link_01");
  QVERIFY(link01Vis != NULL);
  gazebo::rendering::VisualPtr model01Vis =
      scene->GetVisual("ModelPreview_0_0::model_00::model_01");
  QVERIFY(model01Vis != NULL);

  this->ProcessEventsAndDraw(mainWindow);

  // verify initial selected state
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(!boxModelVis->GetHighlighted());
  QVERIFY(!modelVis->GetHighlighted());
  QVERIFY(!link00Vis->GetHighlighted());
  QVERIFY(!link01Vis->GetHighlighted());
  QVERIFY(!model01Vis->GetHighlighted());

  // test selecting links and nested models
  modelCreator->SetSelected(cylinder, true);
  QVERIFY(cylinder->GetHighlighted());
  QVERIFY(!boxModelVis->GetHighlighted());
  QVERIFY(!modelVis->GetHighlighted());
  QVERIFY(!link00Vis->GetHighlighted());
  QVERIFY(!link01Vis->GetHighlighted());
  QVERIFY(!model01Vis->GetHighlighted());

  modelCreator->SetSelected(boxModelVis, true);
  QVERIFY(cylinder->GetHighlighted());
  QVERIFY(boxModelVis->GetHighlighted());
  QVERIFY(!modelVis->GetHighlighted());
  QVERIFY(!link00Vis->GetHighlighted());
  QVERIFY(!link01Vis->GetHighlighted());
  QVERIFY(!model01Vis->GetHighlighted());

  modelCreator->SetSelected(modelVis, true);
  QVERIFY(cylinder->GetHighlighted());
  QVERIFY(boxModelVis->GetHighlighted());
  QVERIFY(modelVis->GetHighlighted());
  QVERIFY(!link00Vis->GetHighlighted());
  QVERIFY(!link01Vis->GetHighlighted());
  QVERIFY(!model01Vis->GetHighlighted());

  modelCreator->SetSelected(link00Vis, true);
  QVERIFY(cylinder->GetHighlighted());
  QVERIFY(boxModelVis->GetHighlighted());
  QVERIFY(modelVis->GetHighlighted());
  QVERIFY(link00Vis->GetHighlighted());
  QVERIFY(!link01Vis->GetHighlighted());
  QVERIFY(!model01Vis->GetHighlighted());

  modelCreator->SetSelected(link01Vis, true);
  QVERIFY(cylinder->GetHighlighted());
  QVERIFY(boxModelVis->GetHighlighted());
  QVERIFY(modelVis->GetHighlighted());
  QVERIFY(link00Vis->GetHighlighted());
  QVERIFY(link01Vis->GetHighlighted());
  QVERIFY(!model01Vis->GetHighlighted());

  modelCreator->SetSelected(model01Vis, true);
  QVERIFY(cylinder->GetHighlighted());
  QVERIFY(boxModelVis->GetHighlighted());
  QVERIFY(modelVis->GetHighlighted());
  QVERIFY(link00Vis->GetHighlighted());
  QVERIFY(link01Vis->GetHighlighted());
  QVERIFY(model01Vis->GetHighlighted());

  modelCreator->SetSelected(cylinder, false);
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(boxModelVis->GetHighlighted());
  QVERIFY(modelVis->GetHighlighted());
  QVERIFY(link00Vis->GetHighlighted());
  QVERIFY(link01Vis->GetHighlighted());
  QVERIFY(model01Vis->GetHighlighted());

  modelCreator->SetSelected(boxModelVis, false);
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(!boxModelVis->GetHighlighted());
  QVERIFY(modelVis->GetHighlighted());
  QVERIFY(link00Vis->GetHighlighted());
  QVERIFY(link01Vis->GetHighlighted());
  QVERIFY(model01Vis->GetHighlighted());

  modelCreator->SetSelected(modelVis, false);
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(!boxModelVis->GetHighlighted());
  QVERIFY(!modelVis->GetHighlighted());
  QVERIFY(link00Vis->GetHighlighted());
  QVERIFY(link01Vis->GetHighlighted());
  QVERIFY(model01Vis->GetHighlighted());

  modelCreator->SetSelected(link00Vis, false);
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(!boxModelVis->GetHighlighted());
  QVERIFY(!modelVis->GetHighlighted());
  QVERIFY(!link00Vis->GetHighlighted());
  QVERIFY(link01Vis->GetHighlighted());
  QVERIFY(model01Vis->GetHighlighted());

  modelCreator->SetSelected(link01Vis, false);
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(!boxModelVis->GetHighlighted());
  QVERIFY(!modelVis->GetHighlighted());
  QVERIFY(!link00Vis->GetHighlighted());
  QVERIFY(!link01Vis->GetHighlighted());
  QVERIFY(model01Vis->GetHighlighted());

  modelCreator->SetSelected(model01Vis, false);
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(!boxModelVis->GetHighlighted());
  QVERIFY(!modelVis->GetHighlighted());
  QVERIFY(!link00Vis->GetHighlighted());
  QVERIFY(!link01Vis->GetHighlighted());
  QVERIFY(!model01Vis->GetHighlighted());

  delete modelCreator;
  modelCreator = NULL;

  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void ModelCreator_TEST::CopyPaste()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, true);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  gui::ModelCreator *modelCreator = new gui::ModelCreator();
  QVERIFY(modelCreator);

  QVERIFY(gazebo::gui::g_copyAct != NULL);
  QVERIFY(gazebo::gui::g_pasteAct != NULL);
  QVERIFY(gui::g_editModelAct != NULL);

  // switch to editor mode
  gui::g_editModelAct->toggle();

  // Inserting a link and it still is never saved
  modelCreator->AddShape(gui::ModelCreator::ENTITY_CYLINDER);
  gazebo::rendering::VisualPtr cylinder =
      scene->GetVisual("ModelPreview_0_0::link_0");

  QVERIFY(cylinder != NULL);

  // Add various models and links into the editor
  // a box nested model
  double mass = 1.0;
  ignition::math::Vector3d size = ignition::math::Vector3d::One;
  msgs::Model model;
  model.set_name("box_model");
  msgs::AddBoxLink(model, mass, size);
  sdf::ElementPtr boxModelSDF = msgs::ModelToSDF(model);
  modelCreator->AddModel(boxModelSDF);

  /// Verify it has been added
  gazebo::rendering::VisualPtr boxModel =
      scene->GetVisual("ModelPreview_0_0::box_model");
  QVERIFY(boxModel != NULL);

  this->ProcessEventsAndDraw(mainWindow);

  // copy and paste cylinder link
  modelCreator->SetSelected(cylinder, true);
  QVERIFY(cylinder->GetHighlighted());

  gui::g_copyAct->trigger();

  // Get GLWidget
  gazebo::gui::GLWidget *glWidget =
    mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != NULL);

  // Move to center of the screen
  QPoint moveTo(glWidget->width() * 0.5, glWidget->height() * 0.5);
  QTest::mouseMove(glWidget, moveTo, 100);

  gui::g_pasteAct->trigger();
  QCoreApplication::processEvents();

  // Verify there is a clone of the cylinder link
  rendering::VisualPtr cylinderClone =
      scene->GetVisual(cylinder->GetName() + "_clone");
  QVERIFY(cylinderClone != NULL);


  // copy and paste box model
  modelCreator->SetSelected(boxModel, true);
  QVERIFY(boxModel->GetHighlighted());

  gui::g_copyAct->trigger();

  // Move to center of the screen
  QTest::mouseMove(glWidget, moveTo, 100);

  gui::g_pasteAct->trigger();
  QCoreApplication::processEvents();

  // Verify there is a clone of the box model
  rendering::VisualPtr boxModelClone =
      scene->GetVisual(boxModel->GetName() + "_clone");
  QVERIFY(boxModelClone != NULL);

  this->ProcessEventsAndDraw(mainWindow);

  delete modelCreator;
  modelCreator = NULL;

  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void ModelCreator_TEST::AddPluginElement()
{
  gui::ModelCreator *modelCreator = new gui::ModelCreator();
  {
    sdf::ElementPtr connectionElem(new sdf::Element);
    connectionElem->SetName("connection");

    sdf::ElementPtr sourceElem(new sdf::Element);
    sourceElem->SetName("source");
    sourceElem->AddValue("string", "Battery", "_none_", "source");

    connectionElem->InsertElement(sourceElem);

    sdf::ElementPtr sourcePortElem(new sdf::Element);
    sourcePortElem->SetName("source_port");
    sourcePortElem->AddValue("string", "Positive", "_none_", "sourcePort");

    connectionElem->InsertElement(sourcePortElem);

    sdf::ElementPtr targetElem(new sdf::Element);
    targetElem->SetName("target");
    targetElem->AddValue("string", "Motor", "_none_", "target");

    connectionElem->InsertElement(targetElem);

    sdf::ElementPtr targetPortElem(new sdf::Element);
    targetPortElem->SetName("target_port");
    targetPortElem->AddValue("string", "Positive", "_none_", "targetPort");

    connectionElem->InsertElement(targetPortElem);

    modelCreator->AppendPluginElement("simple_connections",
        "libSimpleConnectionsPlugin.so", connectionElem);

    sdf::ElementPtr sdfToAppend = modelCreator->GetSDFToAppend();

    QVERIFY(sdfToAppend != NULL);
    QVERIFY(sdfToAppend->HasElement("plugin"));

    sdf::ElementPtr pluginElem = sdfToAppend->GetElement("plugin");
    sdf::ParamPtr nameAttrib = pluginElem->GetAttribute("name");

    QVERIFY(nameAttrib != NULL);
    QVERIFY(nameAttrib->GetAsString() == "simple_connections");

    sdf::ParamPtr filenameAttrib = pluginElem->GetAttribute("filename");

    QVERIFY(filenameAttrib != NULL);
    QVERIFY(filenameAttrib->GetAsString() == "libSimpleConnectionsPlugin.so");

    QVERIFY(pluginElem->HasElement("connection"));
    connectionElem = pluginElem->GetElement("connection");

    QVERIFY(connectionElem->HasElement("source"));
    sourceElem = connectionElem->GetElement("source");
    QVERIFY(sourceElem->GetValue()->GetAsString() == "Battery");

    QVERIFY(connectionElem->HasElement("source_port"));
    sourcePortElem = connectionElem->GetElement("source_port");
    QVERIFY(sourcePortElem->GetValue()->GetAsString() == "Positive");

    QVERIFY(connectionElem->HasElement("target"));
    targetElem = connectionElem->GetElement("target");
    QVERIFY(targetElem->GetValue()->GetAsString() == "Motor");

    QVERIFY(connectionElem->HasElement("target_port"));
    targetPortElem = connectionElem->GetElement("target_port");
    QVERIFY(targetPortElem->GetValue()->GetAsString() == "Positive");
  }
  {
    sdf::ElementPtr connectionElem(new sdf::Element);
    connectionElem->SetName("connection");

    sdf::ElementPtr sourceElem(new sdf::Element);
    sourceElem->SetName("source");
    sourceElem->AddValue("string", "Battery", "_none_", "source");

    connectionElem->InsertElement(sourceElem);

    sdf::ElementPtr sourcePortElem(new sdf::Element);
    sourcePortElem->SetName("source_port");
    sourcePortElem->AddValue("string", "Negative", "_none_", "sourcePort");

    connectionElem->InsertElement(sourcePortElem);

    sdf::ElementPtr targetElem(new sdf::Element);
    targetElem->SetName("target");
    targetElem->AddValue("string", "Motor", "_none_", "target");

    connectionElem->InsertElement(targetElem);

    sdf::ElementPtr targetPortElem(new sdf::Element);
    targetPortElem->SetName("target_port");
    targetPortElem->AddValue("string", "Negative", "_none_", "targetPort");

    connectionElem->InsertElement(targetPortElem);

    modelCreator->AppendPluginElement("simple_connections",
        "libSimpleConnectionsPlugin.so", connectionElem);

    sdf::ElementPtr sdfToAppend = modelCreator->GetSDFToAppend();

    QVERIFY(sdfToAppend != NULL);
    QVERIFY(sdfToAppend->HasElement("plugin"));

    sdf::ElementPtr pluginElem = sdfToAppend->GetElement("plugin");
    sdf::ParamPtr nameAttrib = pluginElem->GetAttribute("name");

    QVERIFY(nameAttrib != NULL);
    QVERIFY(nameAttrib->GetAsString() == "simple_connections");

    sdf::ParamPtr filenameAttrib = pluginElem->GetAttribute("filename");

    QVERIFY(filenameAttrib != NULL);
    QVERIFY(filenameAttrib->GetAsString() == "libSimpleConnectionsPlugin.so");

    QVERIFY(pluginElem->HasElement("connection"));
    connectionElem = pluginElem->GetElement("connection");

    QVERIFY(connectionElem->HasElement("source"));
    sourceElem = connectionElem->GetElement("source");
    QVERIFY(sourceElem->GetValue()->GetAsString() == "Battery");

    QVERIFY(connectionElem->HasElement("source_port"));
    sourcePortElem = connectionElem->GetElement("source_port");
    QVERIFY(sourcePortElem->GetValue()->GetAsString() == "Positive");

    QVERIFY(connectionElem->HasElement("target"));
    targetElem = connectionElem->GetElement("target");
    QVERIFY(targetElem->GetValue()->GetAsString() == "Motor");

    QVERIFY(connectionElem->HasElement("target_port"));
    targetPortElem = connectionElem->GetElement("target_port");
    QVERIFY(targetPortElem->GetValue()->GetAsString() == "Positive");

    pluginElem = sdfToAppend->GetElement("plugin");
  }
  delete modelCreator;
}

static sdf::ElementPtr FindElementNamed(sdf::ElementPtr _parent,
    const std::string &_name)
{
  sdf::ElementPtr result = NULL;
  if (_parent)
  {
    if (_parent->GetName() == _name)
      result = _parent;
    else
      result = FindElementNamed(_parent->GetFirstElement(), _name);
  }
  return result;
}

/////////////////////////////////////////////////
void ModelCreator_TEST::RemovePluginElement()
{
  gui::ModelCreator *modelCreator = new gui::ModelCreator();
  std::ostringstream stream1, stream2, stream3, stream4;
  sdf::SDF sdf1, sdf2, sdf3, sdf4;

  stream1 << "<sdf version='1.5'>"
          << "<model name='SampleModel'>"
          << "<plugin name='simple_connections' "
          << "filename='libSimpleConnectionsPlugin.so'>"
          << "<connection>"
          << "  <source>AA_battery</source>"
          << "  <source_port>positive</source_port>"
          << "  <target>power_switch</target>"
          << "  <target_port>connector0</target_port>"
          << "</connection>"
          << "</plugin>"
          << "</model>"
          << "</sdf>";

  sdf1.SetFromString(stream1.str());
  sdf::ElementPtr batterySwitchConnection =
      FindElementNamed(sdf1.Root(), "connection");

  modelCreator->AppendPluginElement("simple_connections",
      "libSimpleConnectionsPlugin.so", batterySwitchConnection);

  stream2 << "<sdf version='1.5'>"
          << "<model name='SampleModel'>"
          << "<plugin name='simple_connections' "
          << "filename='libSimpleConnectionsPlugin.so'>"
          << "<connection>"
          << "  <source>power_switch</source>"
          << "  <source_port>connector0</source_port>"
          << "  <target>motor</target>"
          << "  <target_port>connector0</target_port>"
          << "</connection>"
          << "</plugin>"
          << "</model>"
          << "</sdf>";

  sdf2.SetFromString(stream2.str());
  sdf::ElementPtr switchMotorConnection =
      FindElementNamed(sdf2.Root(), "connection");

  modelCreator->AppendPluginElement("simple_connections",
      "libSimpleConnectionsPlugin.so", switchMotorConnection);

  stream3 << "<sdf version='1.5'>"
          << "<model name='SampleModel'>"
          << "<plugin name='simple_connections' "
          << "filename='libSimpleConnectionsPlugin.so'>"
          << "<connection>"
          << "  <source>motor</source>"
          << "  <source_port>connector2</source_port>"
          << "  <target>AA_battery</target>"
          << "  <target_port>negative</target_port>"
          << "</connection>"
          << "</plugin>"
          << "</model>"
          << "</sdf>";

  sdf3.SetFromString(stream3.str());
  sdf::ElementPtr motorBatteryConnection =
      FindElementNamed(sdf3.Root(), "connection");

  modelCreator->AppendPluginElement("simple_connections",
      "libSimpleConnectionsPlugin.so", motorBatteryConnection);

  modelCreator->RemovePluginElement("simple_connections",
      "libSimpleConnectionsPlugin.so", motorBatteryConnection);

  sdf::ElementPtr sdfToAppend = modelCreator->GetSDFToAppend();

  QVERIFY(sdfToAppend->HasElement("plugin"));
  sdf::ElementPtr pluginElem = sdfToAppend->GetElement("plugin");

  stream4 << "<sdf version='1.5'>"
          << "<model name='SampleModel'>"
          << pluginElem->ToString("")
          << "</model>"
          << "</sdf>";

  sdf4.SetFromString(stream4.str());

  pluginElem = sdf4.Root()->GetElement("model")->GetElement("plugin");

  // Verify first connection

  sdf::ElementPtr connectionElem = pluginElem->GetElement("connection");

  sdf::ElementPtr sourceElem = connectionElem->GetElement("source");
  sdf::ParamPtr sourceValue = sourceElem->GetValue();

  sdf::ElementPtr targetElem = connectionElem->GetElement("target");
  sdf::ParamPtr targetValue = targetElem->GetValue();

  QVERIFY(sourceValue && sourceValue->GetAsString() == "AA_battery");
  QVERIFY(targetValue && targetValue->GetAsString() == "power_switch");

  // Verify second connection

  connectionElem = connectionElem->GetNextElement("connection");

  sourceElem = connectionElem->GetElement("source");
  sourceValue = sourceElem->GetValue();

  targetElem = connectionElem->GetElement("target");
  targetValue = targetElem->GetValue();

  QVERIFY(sourceValue && sourceValue->GetAsString() == "power_switch");
  QVERIFY(targetValue && targetValue->GetAsString() == "motor");

  // Verify no third connection

  QVERIFY(connectionElem->GetNextElement("connection") == NULL);

  delete modelCreator;
}

// Generate a main function for the test
QTEST_MAIN(ModelCreator_TEST)
