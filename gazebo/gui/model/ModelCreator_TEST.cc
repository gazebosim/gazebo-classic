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
#include "sdf/sdf.hh"

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
