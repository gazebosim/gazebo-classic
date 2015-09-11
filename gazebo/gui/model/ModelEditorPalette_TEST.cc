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
#include "gazebo/gui/model/ModelEditorPalette.hh"
#include "gazebo/gui/model/ModelEditorPalette_TEST.hh"

using namespace gazebo;

/////////////////////////////////////////////////
void ModelEditorPalette_TEST::AddItem()
{
  gui::ModelEditorPalette *palette = new gui::ModelEditorPalette();
  QVERIFY(palette);

  // add a custom tool button to the palette
  QToolButton *testButton = new QToolButton();
  testButton->setObjectName("my_tool_button");

  // test adding without specifying category
  palette->AddItem(testButton);

  // verify that the button is added.
  QToolButton *retButton =
      palette->findChild<QToolButton *>("my_tool_button");
  QVERIFY(retButton);
  QVERIFY(retButton == testButton);

  // add another custom tool button to the palette
  QToolButton *testButton2 = new QToolButton();
  testButton2->setObjectName("my_tool_button2");

  // test adding with category
  palette->AddItem(testButton2, "category");

  // verify that the button is added.
  QToolButton *retButton2 =
      palette->findChild<QToolButton *>("my_tool_button2");
  QVERIFY(retButton2);
  QVERIFY(retButton2 == testButton2);

  // add a button to an existing category
  QToolButton *testButton3 = new QToolButton();
  testButton3->setObjectName("my_tool_button3");

  palette->AddItem(testButton3, "category");

  // verify that the button is added.
  QToolButton *retButton3 =
      palette->findChild<QToolButton *>("my_tool_button3");
  QVERIFY(retButton3);
  QVERIFY(retButton3 == testButton3);

  // add a button with an empty category
  QToolButton *testButton4 = new QToolButton();
  testButton4->setObjectName("my_tool_button4");

  palette->AddItem(testButton4, "");

  // verify that the button is added.
  QToolButton *retButton4 =
      palette->findChild<QToolButton *>("my_tool_button4");
  QVERIFY(retButton4);
  QVERIFY(retButton4 == testButton4);

  delete palette;
  palette = NULL;
}

/////////////////////////////////////////////////
void ModelEditorPalette_TEST::AddRemoveNestedModels()
{
  gui::ModelEditorPalette *palette = new gui::ModelEditorPalette();
  QVERIFY(palette);

  // Get nested model item
  QList<QTreeWidget *> trees = palette->findChildren<QTreeWidget *>();
  QCOMPARE(trees.size(), 1);
  QCOMPARE(trees[0]->topLevelItemCount(), 4);

  QTreeWidgetItem *nestedModelsItem = trees[0]->topLevelItem(1);
  QVERIFY(nestedModelsItem->text(0) == "Nested Models");

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

  delete palette;
  palette = NULL;
}

/////////////////////////////////////////////////
void ModelEditorPalette_TEST::LoadNestedModel()
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

  // Get palette
  gazebo::gui::ModelEditorPalette *palette =
      mainWindow->findChild<gazebo::gui::ModelEditorPalette *>();
  QVERIFY(palette != NULL);

  // Get tree
  QTreeWidget *tree = palette->findChild<QTreeWidget *>();
  QCOMPARE(tree->topLevelItemCount(), 4);

  // Check nested models are in correct hierarchy
  QTreeWidgetItem *nestedModelsItem = tree->topLevelItem(1);
  QVERIFY(nestedModelsItem->text(0) == "Nested Models");

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
}


// Generate a main function for the test
QTEST_MAIN(ModelEditorPalette_TEST)
