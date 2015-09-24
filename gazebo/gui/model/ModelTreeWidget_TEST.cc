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

#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/ModelTreeWidget.hh"
#include "gazebo/gui/model/ModelTreeWidget_TEST.hh"

using namespace gazebo;

/////////////////////////////////////////////////
void ModelTreeWidget_TEST::AddRemoveLinks()
{
  gui::ModelTreeWidget *palette = new gui::ModelTreeWidget();
  QVERIFY(palette);

  // Get link item
  QList<QTreeWidget *> trees = palette->findChildren<QTreeWidget *>();
  QCOMPARE(trees.size(), 1);
  QCOMPARE(trees[0]->topLevelItemCount(), 3);

  QTreeWidgetItem *linksItem = trees[0]->topLevelItem(1);
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

  delete palette;
  palette = NULL;
}

/////////////////////////////////////////////////
void ModelTreeWidget_TEST::AddRemoveJoints()
{
  gui::ModelTreeWidget *palette = new gui::ModelTreeWidget();
  QVERIFY(palette);

  // Get joint item
  QList<QTreeWidget *> trees = palette->findChildren<QTreeWidget *>();
  QCOMPARE(trees.size(), 1);
  QCOMPARE(trees[0]->topLevelItemCount(), 3);

  QTreeWidgetItem *jointsItem = trees[0]->topLevelItem(2);
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

  delete palette;
  palette = NULL;
}

/////////////////////////////////////////////////
void ModelTreeWidget_TEST::AddRemoveModelPlugins()
{
  gui::ModelTreeWidget *palette = new gui::ModelTreeWidget();
  QVERIFY(palette);

  // Get model plugin item
  QList<QTreeWidget *> trees = palette->findChildren<QTreeWidget *>();
  QCOMPARE(trees.size(), 1);
  QCOMPARE(trees[0]->topLevelItemCount(), 3);

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

  delete palette;
  palette = NULL;
}

// Generate a main function for the test
QTEST_MAIN(ModelTreeWidget_TEST)
