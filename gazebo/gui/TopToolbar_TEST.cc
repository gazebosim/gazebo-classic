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
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/GuiEvents.hh"

#include "gazebo/gui/TopToolbar.hh"
#include "gazebo/gui/TopToolbar_TEST.hh"

/////////////////////////////////////////////////
void TopToolbar_TEST::WindowModes()
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

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Get the top toolbar
  gazebo::gui::TopToolbar *topToolbar =
      mainWindow->findChild<gazebo::gui::TopToolbar *>("topToolbar");
  QVERIFY(topToolbar != NULL);

  // Check RTSN are visible
  QVERIFY(gazebo::gui::g_arrowAct->isVisible());
  QVERIFY(gazebo::gui::g_rotateAct->isVisible());
  QVERIFY(gazebo::gui::g_translateAct->isVisible());
  QVERIFY(gazebo::gui::g_scaleAct->isVisible());
  QVERIFY(gazebo::gui::g_snapAct->isVisible());

  // Check insert shapes are visible
  QVERIFY(gazebo::gui::g_boxCreateAct->isVisible());
  QVERIFY(gazebo::gui::g_sphereCreateAct->isVisible());
  QVERIFY(gazebo::gui::g_cylinderCreateAct->isVisible());

  // Change to Model Editor mode
  gazebo::gui::Events::windowMode("ModelEditor");

  // Check RTSN are visible for
  QVERIFY(gazebo::gui::g_arrowAct->isVisible());
  QVERIFY(gazebo::gui::g_rotateAct->isVisible());
  QVERIFY(gazebo::gui::g_translateAct->isVisible());
  QVERIFY(gazebo::gui::g_scaleAct->isVisible());
  QVERIFY(gazebo::gui::g_snapAct->isVisible());

  // Check insert shapes are not visible
  QVERIFY(!gazebo::gui::g_boxCreateAct->isVisible());
  QVERIFY(!gazebo::gui::g_sphereCreateAct->isVisible());
  QVERIFY(!gazebo::gui::g_cylinderCreateAct->isVisible());

  // Change to Log Playback mode
  gazebo::gui::Events::windowMode("LogPlayback");

  // Check RTSN are not visible
  QVERIFY(!gazebo::gui::g_arrowAct->isVisible());
  QVERIFY(!gazebo::gui::g_rotateAct->isVisible());
  QVERIFY(!gazebo::gui::g_translateAct->isVisible());
  QVERIFY(!gazebo::gui::g_scaleAct->isVisible());
  QVERIFY(!gazebo::gui::g_snapAct->isVisible());

  // Check insert shapes are not visible
  QVERIFY(!gazebo::gui::g_boxCreateAct->isVisible());
  QVERIFY(!gazebo::gui::g_sphereCreateAct->isVisible());
  QVERIFY(!gazebo::gui::g_cylinderCreateAct->isVisible());

  // Clean up
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void TopToolbar_TEST::Insert()
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

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Get the top toolbar
  gazebo::gui::TopToolbar *topToolbar =
      mainWindow->findChild<gazebo::gui::TopToolbar *>("topToolbar");
  QVERIFY(topToolbar != NULL);

  // Get number of actions
  QToolBar *toolbar = topToolbar->findChild<QToolBar *>("topToolbarToolbar");
  QVERIFY(toolbar != NULL);

  int actionsCount = toolbar->actions().size();

  // Insert separator and see increase in number of actions
  QAction *separator = topToolbar->InsertSeparator("toolbarSpacerAction");
  QVERIFY(separator != NULL);
  QCOMPARE(toolbar->actions().size(), actionsCount + 1);
  actionsCount = toolbar->actions().size();

  // Fail to insert before inexistent action
  QAction *separatorFail = topToolbar->InsertSeparator("fail");
  QVERIFY(separatorFail == NULL);
  QCOMPARE(toolbar->actions().size(), actionsCount);

  // Insert widget and see increase in number of actions
  QWidget *widget = new QWidget();
  QAction *widgetAct = topToolbar->InsertWidget("toolbarSpacerAction", widget);
  QVERIFY(widgetAct != NULL);
  QCOMPARE(toolbar->actions().size(), actionsCount + 1);
  actionsCount = toolbar->actions().size();

  // Fail to insert before inexistent action
  QWidget *widgetFail = new QWidget();
  QAction *widgetActFail = topToolbar->InsertWidget("fail", widgetFail);
  QVERIFY(widgetActFail == NULL);
  QCOMPARE(toolbar->actions().size(), actionsCount);

  // Insert action and see increase in number of actions
  QAction *action = new QAction(this);
  topToolbar->InsertAction("toolbarSpacerAction", action);
  QCOMPARE(toolbar->actions().size(), actionsCount + 1);
  actionsCount = toolbar->actions().size();

  // Fail to insert before inexistent action
  QAction *actionFail = new QAction(this);
  topToolbar->InsertAction("fail", actionFail);
  QCOMPARE(toolbar->actions().size(), actionsCount);

  // Clean up
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void TopToolbar_TEST::Add()
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

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Get the top toolbar
  gazebo::gui::TopToolbar *topToolbar =
      mainWindow->findChild<gazebo::gui::TopToolbar *>("topToolbar");
  QVERIFY(topToolbar != NULL);

  // Get number of actions
  QToolBar *toolbar = topToolbar->findChild<QToolBar *>("topToolbarToolbar");
  QVERIFY(toolbar != NULL);

  int actionsCount = toolbar->actions().size();

  // Add separator and see increase in number of actions
  QAction *separator = topToolbar->AddSeparator();
  QVERIFY(separator != NULL);
  QCOMPARE(toolbar->actions().size(), actionsCount + 1);
  actionsCount = toolbar->actions().size();

  // Add widget and see increase in number of actions
  QWidget *widget = new QWidget();
  QAction *widgetAct = topToolbar->AddWidget(widget);
  QVERIFY(widgetAct != NULL);
  QCOMPARE(toolbar->actions().size(), actionsCount + 1);
  actionsCount = toolbar->actions().size();

  // Add action and see increase in number of actions
  QAction *action = new QAction(this);
  topToolbar->AddAction(action);
  QCOMPARE(toolbar->actions().size(), actionsCount + 1);
  actionsCount = toolbar->actions().size();

  // Clean up
  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(TopToolbar_TEST)
