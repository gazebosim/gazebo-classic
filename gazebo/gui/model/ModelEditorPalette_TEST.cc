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

#include "gazebo/gui/MainWindow.hh"
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

  // add another custom tool button to the palette
  QToolButton *testButton2 = new QToolButton();
  testButton2->setObjectName("my_tool_button2");

  // test adding without specifying category
  palette->AddItem(testButton2, "category");

  // verify that the button is added.
  QToolButton *retButton2 =
      palette->findChild<QToolButton *>("my_tool_button2");
  QVERIFY(retButton2);

  delete palette;
  palette = NULL;
}

// Generate a main function for the test
QTEST_MAIN(ModelEditorPalette_TEST)
