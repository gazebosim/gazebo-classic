/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <array>

#include "gazebo/common/MouseEvent.hh"
#include "gazebo/common/SystemPaths.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/InsertModelWidget.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/MouseEventHandler.hh"

#include "gazebo/gui/QTestFixture.hh"

#include "test_config.h"

class GuiSpawnedNestedURDFTest: public QTestFixture
{
  Q_OBJECT

  private slots: void SpawnNestedURDF();
};

/////////////////////////////////////////////////
void GuiSpawnedNestedURDFTest::SpawnNestedURDF()
{
  this->Load("worlds/empty.world", true, false, false);

  // Create the main window.
  auto mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != nullptr);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera, scene and GLWidget
  auto cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != nullptr);
  auto scene = cam->GetScene();
  QVERIFY(scene != nullptr);
  auto glWidget = mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != nullptr);

  // Add the test model database to the insert tab
  gazebo::common::SystemPaths::Instance()->AddModelPathsUpdate(
      PROJECT_SOURCE_PATH "/test/models/test_nested_urdf");

  // Get the insert model widget
  auto insertModelWidget = mainWindow->findChild<
      gazebo::gui::InsertModelWidget *>("insertModel");
  QVERIFY(insertModelWidget != nullptr);

  // Get the items in the list
  auto tree = insertModelWidget->findChildren<QTreeWidget *>();

  std::array modelNames{"model_urdf", "model_sdf_1_6", "model_sdf_1_7"};
  for (std::size_t i = 0; i < modelNames.size(); ++i)
  {
    std::string modelName = modelNames[i];
    QVERIFY(tree.size() == 1u);
    auto modelItem = tree[0]->findItems(QString::fromStdString(modelName),
        Qt::MatchContains | Qt::MatchRecursive);
    QVERIFY(modelItem.size() > 0);

    QPoint mousePos(glWidget->width() * 0.25 * (1 + i),
                    glWidget->height() * 0.5);

    // Trigger signal as if item was clicked
    QMetaObject::invokeMethod(tree[0], "itemClicked", Q_ARG(QTreeWidgetItem *,
          modelItem[0]), Q_ARG(int, 0));

    QTest::mouseMove(glWidget, mousePos);
    this->ProcessEventsAndDraw(mainWindow);

    std::string modelNamePrefix = "model_urdf::";
    if (modelName == "model_urdf")
      modelNamePrefix = "";

    const std::string baseLinkName = modelNamePrefix + "base_link";
    const std::string link1Name = modelNamePrefix + "link_1";

    // Check visual shows the links in the right positions
    auto modelVis = scene->GetVisual(modelName);
    QVERIFY(modelVis != nullptr);

    auto baseLinkVis = scene->GetVisual(modelName + "::" + baseLinkName);
    QVERIFY(baseLinkVis != nullptr);

    auto link1Vis = scene->GetVisual(modelName + "::" + link1Name);
    QVERIFY(link1Vis != nullptr);

    if (modelName == "model_urdf")
    {
      // There is no offset of 0.1 when spawning model_urdf directly
      QVERIFY(std::fabs(0.0 - baseLinkVis->WorldPose().Pos().Z()) < 1e-5);
      QVERIFY(std::fabs(1.0 - link1Vis->WorldPose().Pos().Z()) < 1e-5);
    }
    else
    {
      QVERIFY(std::fabs(0.1 - baseLinkVis->WorldPose().Pos().Z()) < 1e-5);
      QVERIFY(std::fabs(1.1 - link1Vis->WorldPose().Pos().Z()) < 1e-5);
    }
    // Press the mouse in the scene to finish inserting a model
    QTest::mouseRelease(glWidget, Qt::LeftButton, 0, mousePos);

    this->ProcessEventsAndDraw(mainWindow);

    baseLinkVis = scene->GetVisual(modelName + "::" + baseLinkName);
    QVERIFY(baseLinkVis != nullptr);

    link1Vis = scene->GetVisual(modelName + "::" + link1Name);
    QVERIFY(link1Vis != nullptr);
    // Check that after insertion that the link poses are still as expected
    if (modelName == "model_urdf")
    {
      // There is no offset of 0.1 when spawning model_urdf directly
      QVERIFY(std::fabs(baseLinkVis->WorldPose().Pos().Z()) < 1e-5);
      QVERIFY(std::fabs(1.0 - link1Vis->WorldPose().Pos().Z()) < 1e-5);
    }
    else
    {
      QVERIFY(std::fabs(0.1 - baseLinkVis->WorldPose().Pos().Z()) < 1e-5);
      QVERIFY(std::fabs(1.1 - link1Vis->WorldPose().Pos().Z()) < 1e-5);
    }
  }

  // Get world
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);
  world->SetPaused(false);

  this->ProcessEventsAndDraw(mainWindow, 30);
  // Check that after insertion that the link poses are still as expected
  // We have to find the links again because a new set of Visuals will have been
  // created
  for (std::size_t i = 0; i < modelNames.size(); ++i)
  {
    std::string modelName = modelNames[i];

    std::string modelNamePrefix = "model_urdf::";
    if (modelName == "model_urdf")
      modelNamePrefix = "";

    const std::string baseLinkName = modelNamePrefix + "base_link";
    const std::string link1Name = modelNamePrefix + "link_1";

    // Check visual shows the links in the right positions
    auto modelVis = scene->GetVisual(modelName);
    QVERIFY(modelVis != nullptr);

    auto baseLinkVis = scene->GetVisual(modelName + "::" + baseLinkName);
    QVERIFY(baseLinkVis != nullptr);

    auto link1Vis = scene->GetVisual(modelName + "::" + link1Name);
    QVERIFY(link1Vis != nullptr);

    QVERIFY(std::fabs(baseLinkVis->WorldPose().Pos().Z()) < 1e-5);
    QVERIFY(std::fabs(1.0 - link1Vis->WorldPose().Pos().Z()) < 1e-5);
  }

  mainWindow->close();
  delete mainWindow;
  mainWindow = nullptr;
}

/////////////////////////////////////////////////

// Generate a main function for the test
QTEST_MAIN(GuiSpawnedNestedURDFTest)
// The included file doesn't exist, but will be created by moc. Be sure to keep
// the name in sync with the .cc file name.
#include "2728_gui_spawned_nested_urdf.moc"
