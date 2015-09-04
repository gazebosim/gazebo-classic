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
void ModelEditorPalette_TEST::ModelPlugins()
{
  gui::ModelEditorPalette *palette = new gui::ModelEditorPalette();
  QVERIFY(palette);

  // Get model plugin item
  QList<QTreeWidget *> trees = palette->findChildren<QTreeWidget *>();
  QCOMPARE(trees.size(), 1);
  QCOMPARE(trees[0]->topLevelItemCount(), 3);

  // Check number of model plugins



  // Insert a plugin and check number again
  gazebo::gui::model::Events::modelPluginInserted("plugin1");

  delete palette;
  palette = NULL;
}

// Generate a main function for the test
QTEST_MAIN(ModelEditorPalette_TEST)
