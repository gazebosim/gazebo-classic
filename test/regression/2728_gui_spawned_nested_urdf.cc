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
  QVERIFY(tree.size() == 1u);
  auto modelItem = tree[0]->findItems(QString("model_sdf_1_6"),
      Qt::MatchContains | Qt::MatchRecursive);
  QVERIFY(modelItem.size() > 0);

  // Trigger signal as if item was clicked
  QMetaObject::invokeMethod(tree[0], "itemClicked", Q_ARG(QTreeWidgetItem *,
      modelItem[0]), Q_ARG(int, 0));

  QTest::mouseMove(glWidget, QPoint(-mainWindow->width() * 0.5,
                                    -mainWindow->height() * 0.5));
  this->ProcessEventsAndDraw(mainWindow);
  const std::string modelName = "model_sdf_1_6";
  const std::string baseLinkName = "model_urdf::base_link";
  const std::string link1Name = "model_urdf::link_1";

  // Check visual shows the links in the right positions
  auto modelVis = scene->GetVisual(modelName);
  QVERIFY(modelVis != nullptr);

  auto baseLinkVis = scene->GetVisual(modelName + "::" + baseLinkName);
  QVERIFY(baseLinkVis != nullptr);

  auto link1Vis = scene->GetVisual(modelName + "::" + link1Name);
  QVERIFY(link1Vis != nullptr);

  QCOMPARE(ignition::math::Pose3d(0, 0, 0.1, 0, 0, 0), baseLinkVis->WorldPose());
  QCOMPARE(ignition::math::Pose3d(0, 0, 1.1, 0, 0, 0), link1Vis->WorldPose());

  // Press the mouse in the scene to finish inserting a model
  QTest::mouseRelease(
      glWidget, Qt::LeftButton, 0,
      QPoint(-mainWindow->width() * 0.5, -mainWindow->height() * 0.5));

  this->ProcessEventsAndDraw(mainWindow);

  baseLinkVis = scene->GetVisual(modelName + "::" + baseLinkName);
  QVERIFY(baseLinkVis != nullptr);

  link1Vis = scene->GetVisual(modelName + "::" + link1Name);
  QVERIFY(link1Vis != nullptr);
  // Check that after insertion that the link poses are still as expected
  QCOMPARE(ignition::math::Pose3d(0, 0, 0.1, 0, 0, 0), baseLinkVis->WorldPose());
  QCOMPARE(ignition::math::Pose3d(0, 0, 1.1, 0, 0, 0), link1Vis->WorldPose());

  // Get world
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

  world->SetPaused(false);

  this->ProcessEventsAndDraw(mainWindow, 30);
  // Check that after insertion that the link poses are still as expected
  QVERIFY(std::fabs(baseLinkVis->WorldPose().Pos().Z()) < 1e-5);
  QVERIFY(std::fabs(1.0 - link1Vis->WorldPose().Pos().Z()) < 1e-5);

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
