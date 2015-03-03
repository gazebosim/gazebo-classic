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

#include "gazebo/gui/RenderWidget.hh"
#include "gazebo/gui/RenderWidget_TEST.hh"

/////////////////////////////////////////////////
void RenderWidget_TEST::InsertWidget()
{
  this->Load("empty.world");

  // Create a new render widget
  gazebo::gui::RenderWidget *renderWidget = new gazebo::gui::RenderWidget;

  // Get the splitter widget
  QSplitter *splitter = renderWidget->findChild<QSplitter *>();
  QVERIFY(splitter != NULL);

  QVERIFY(splitter->count() == 1);
  QWidget *render3DWidget = splitter->widget(0);
  QVERIFY(render3DWidget->objectName().toStdString() == "render3DFrame");

  // Prepend a widget
  QWidget *testWidget = new QWidget(renderWidget);
  testWidget->setObjectName("testWidget");
  renderWidget->InsertWidget(0, testWidget);
  QVERIFY(splitter->count() == 2);
  QWidget *verifyTestWidget = splitter->widget(0);
  QVERIFY(verifyTestWidget->objectName().toStdString() == "testWidget");

  // Append a widget
  QWidget *testWidget2 = new QWidget(renderWidget);
  testWidget2->setObjectName("testWidget2");
  renderWidget->InsertWidget(splitter->count(), testWidget2);
  QVERIFY(splitter->count() == 3);
  QWidget *verifyTestWidget2 = splitter->widget(2);
  QVERIFY(verifyTestWidget2->objectName().toStdString() == "testWidget2");
}

// Generate a main function for the test
QTEST_MAIN(RenderWidget_TEST)
