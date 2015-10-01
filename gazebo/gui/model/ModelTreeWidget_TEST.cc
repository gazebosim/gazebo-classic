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
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/ModelTreeWidget.hh"
#include "gazebo/gui/model/ModelTreeWidget_TEST.hh"

using namespace gazebo;

/////////////////////////////////////////////////
void ModelTreeWidget_TEST::AddRemoveNestedModels()
{
  gui::ModelTreeWidget *modelTree = new gui::ModelTreeWidget();
  QVERIFY(modelTree);

  // Get nested model item
  QList<QTreeWidget *> trees = modelTree->findChildren<QTreeWidget *>();
  QCOMPARE(trees.size(), 1);
  QCOMPARE(trees[0]->topLevelItemCount(), 4);

  QTreeWidgetItem *nestedModelsItem = trees[0]->topLevelItem(1);
  QVERIFY(nestedModelsItem->text(0) == "Models");

  // Check number of nested models
  QCOMPARE(nestedModelsItem->childCount(), 0);

  // Insert a nested model and check number again
  gazebo::gui::model::Events::nestedModelInserted("nestedModel1");
  QCOMPARE(nestedModelsItem->childCount(), 1);
  QVERIFY(nestedModelsItem->child(0)->data(0, Qt::UserRole) == "nestedModel1");

  // Insert another nested model and check number again
  gazebo::gui::model::Events::nestedModelInserted("nestedModel2");
  QCOMPARE(nestedModelsItem->childCount(), 2);
  QVERIFY(nestedModelsItem->child(0)->data(0, Qt::UserRole) == "nestedModel1");
  QVERIFY(nestedModelsItem->child(1)->data(0, Qt::UserRole) == "nestedModel2");

  delete modelTree;
  modelTree = NULL;
}

/////////////////////////////////////////////////
void ModelTreeWidget_TEST::LoadNestedModel()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("test/worlds/deeply_nested_models.world");

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

  // Get the top level model
  gazebo::rendering::VisualPtr modelVis = scene->GetVisual("model_00");
  QVERIFY(modelVis != NULL);

  // Edit the top level model
  gui::g_editModelAct->trigger();
  gui::Events::editModel("model_00");

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Get model tree
  gazebo::gui::ModelTreeWidget *modelTree =
      mainWindow->findChild<gazebo::gui::ModelTreeWidget *>();
  QVERIFY(modelTree != NULL);

  // Get tree
  QTreeWidget *tree = modelTree->findChild<QTreeWidget *>();
  QCOMPARE(tree->topLevelItemCount(), 4);

  // Check nested models are in correct hierarchy
  QTreeWidgetItem *nestedModelsItem = tree->topLevelItem(1);
  QVERIFY(nestedModelsItem->text(0) == "Models");

  // 1st nested model
  QCOMPARE(nestedModelsItem->childCount(), 1);
  QTreeWidgetItem *model_1 = nestedModelsItem->child(0);
  QVERIFY(model_1->data(0, Qt::UserRole) ==
      "ModelPreview_1::model_01");

  // 2nd nested model
  QCOMPARE(model_1->childCount(), 1);
  QTreeWidgetItem *model_2 = model_1->child(0);
  QVERIFY(model_2->data(0, Qt::UserRole) ==
      "ModelPreview_1::model_01::model_02");

  // 3rd nested model
  QCOMPARE(model_2->childCount(), 1);
  QTreeWidgetItem *model_3 = model_2->child(0);
  QVERIFY(model_3->data(0, Qt::UserRole) ==
      "ModelPreview_1::model_01::model_02::model_03");

  // Check only the top level link is in the list
  QTreeWidgetItem *linksItem = tree->topLevelItem(2);
  QVERIFY(linksItem->text(0) == "Links");
  QCOMPARE(linksItem->childCount(), 1);
  QVERIFY(linksItem->child(0)->data(0, Qt::UserRole) ==
      "ModelPreview_1::link_00");

  // Check all joints are in the list
  QTreeWidgetItem *jointsItem = tree->topLevelItem(3);
  QVERIFY(jointsItem->text(0) == "Joints");
  QCOMPARE(jointsItem->childCount(), 3);

  QVERIFY(jointsItem->child(0)->data(0, Qt::UserRole) ==
      "ModelPreview_1::joint_00_UNIQUE_ID_");
  QVERIFY(jointsItem->child(1)->data(0, Qt::UserRole) ==
      "ModelPreview_1::joint_01_UNIQUE_ID_");
  QVERIFY(jointsItem->child(2)->data(0, Qt::UserRole) ==
      "ModelPreview_1::joint_02_UNIQUE_ID_");

  delete modelTree;
  modelTree = NULL;
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void ModelTreeWidget_TEST::AddRemoveLinks()
{
  gui::ModelTreeWidget *modelTree = new gui::ModelTreeWidget();
  QVERIFY(modelTree);

  // Get link item
  QList<QTreeWidget *> trees = modelTree->findChildren<QTreeWidget *>();
  QCOMPARE(trees.size(), 1);
  QCOMPARE(trees[0]->topLevelItemCount(), 4);

  QTreeWidgetItem *linksItem = trees[0]->topLevelItem(2);
  QVERIFY(linksItem->text(0) == "Links");

  // Check number of links
  QCOMPARE(linksItem->childCount(), 0);

  // Insert a link and check number again
  gazebo::gui::model::Events::linkInserted("link1");
  QCOMPARE(linksItem->childCount(), 1);
  QVERIFY(linksItem->child(0)->data(0, Qt::UserRole) == "link1");

  // Insert another link and check number again
  gazebo::gui::model::Events::linkInserted("link2");
  QCOMPARE(linksItem->childCount(), 2);
  QVERIFY(linksItem->child(0)->data(0, Qt::UserRole) == "link1");
  QVERIFY(linksItem->child(1)->data(0, Qt::UserRole) == "link2");

  // Remove a link and check number again
  gazebo::gui::model::Events::linkRemoved("link1");
  QCOMPARE(linksItem->childCount(), 1);
  QVERIFY(linksItem->child(0)->data(0, Qt::UserRole) == "link2");

  // Try to remove inexistent link
  gazebo::gui::model::Events::linkRemoved("link3");
  QCOMPARE(linksItem->childCount(), 1);
  QVERIFY(linksItem->child(0)->data(0, Qt::UserRole) == "link2");

  delete modelTree;
  modelTree = NULL;
}

/////////////////////////////////////////////////
void ModelTreeWidget_TEST::AddRemoveJoints()
{
  gui::ModelTreeWidget *modelTree = new gui::ModelTreeWidget();
  QVERIFY(modelTree);

  // Get joint item
  QList<QTreeWidget *> trees = modelTree->findChildren<QTreeWidget *>();
  QCOMPARE(trees.size(), 1);
  QCOMPARE(trees[0]->topLevelItemCount(), 4);

  QTreeWidgetItem *jointsItem = trees[0]->topLevelItem(3);
  QVERIFY(jointsItem->text(0) == "Joints");

  // Check number of joints
  QCOMPARE(jointsItem->childCount(), 0);

  // Insert a joint and check number again
  gazebo::gui::model::Events::jointInserted("joint1Id", "joint1Name", "type",
      "parent", "child");
  QCOMPARE(jointsItem->childCount(), 1);
  QVERIFY(jointsItem->child(0)->data(0, Qt::UserRole) == "joint1Id");

  // Insert another joint and check number again
  gazebo::gui::model::Events::jointInserted("joint2Id", "joint2Name", "type",
      "parent", "child");
  QCOMPARE(jointsItem->childCount(), 2);
  QVERIFY(jointsItem->child(0)->data(0, Qt::UserRole) == "joint1Id");
  QVERIFY(jointsItem->child(1)->data(0, Qt::UserRole) == "joint2Id");

  // Remove a joint and check number again
  gazebo::gui::model::Events::jointRemoved("joint1Id");
  QCOMPARE(jointsItem->childCount(), 1);
  QVERIFY(jointsItem->child(0)->data(0, Qt::UserRole) == "joint2Id");

  // Try to remove inexistent joint
  gazebo::gui::model::Events::jointRemoved("joint3Id");
  QCOMPARE(jointsItem->childCount(), 1);
  QVERIFY(jointsItem->child(0)->data(0, Qt::UserRole) == "joint2Id");

  delete modelTree;
  modelTree = NULL;
}

/////////////////////////////////////////////////
void ModelTreeWidget_TEST::AddRemoveModelPlugins()
{
  gui::ModelTreeWidget *modelTree = new gui::ModelTreeWidget();
  QVERIFY(modelTree);

  // Get model plugin item
  QList<QTreeWidget *> trees = modelTree->findChildren<QTreeWidget *>();
  QCOMPARE(trees.size(), 1);
  QCOMPARE(trees[0]->topLevelItemCount(), 4);

  QTreeWidgetItem *modelPluginsItem = trees[0]->topLevelItem(0);
  QVERIFY(modelPluginsItem->text(0) == "Model Plugins");

  // Check number of model plugins
  QCOMPARE(modelPluginsItem->childCount(), 0);

  // Insert a plugin and check number again
  gazebo::gui::model::Events::modelPluginInserted("plugin1");
  QCOMPARE(modelPluginsItem->childCount(), 1);
  QVERIFY(modelPluginsItem->child(0)->data(0, Qt::UserRole) == "plugin1");

  // Insert another plugin and check number again
  gazebo::gui::model::Events::modelPluginInserted("plugin2");
  QCOMPARE(modelPluginsItem->childCount(), 2);
  QVERIFY(modelPluginsItem->child(0)->data(0, Qt::UserRole) == "plugin1");
  QVERIFY(modelPluginsItem->child(1)->data(0, Qt::UserRole) == "plugin2");

  // Remove a plugin and check number again
  gazebo::gui::model::Events::modelPluginRemoved("plugin1");
  QCOMPARE(modelPluginsItem->childCount(), 1);
  QVERIFY(modelPluginsItem->child(0)->data(0, Qt::UserRole) == "plugin2");

  // Try to remove inexistent plugin
  gazebo::gui::model::Events::modelPluginRemoved("plugin3");
  QCOMPARE(modelPluginsItem->childCount(), 1);
  QVERIFY(modelPluginsItem->child(0)->data(0, Qt::UserRole) == "plugin2");

  delete modelTree;
  modelTree = NULL;
}

// Generate a main function for the test
QTEST_MAIN(ModelTreeWidget_TEST)
