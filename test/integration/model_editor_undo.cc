/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include "gazebo/gui/model/ModelCreator.hh"
#include "gazebo/gui/GLWidget.hh"

#include "model_editor_undo.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void ModelEditorUndoTest::LinkInsertionByMouse()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  // Create the main window.
  auto mainWindow = new gazebo::gui::MainWindow();
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

  // Get the user camera, scene and GLWidget
  auto cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  auto scene = cam->GetScene();
  QVERIFY(scene != NULL);
  auto glWidget = mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != NULL);

  // Get the cylinder button on the model editor palette
  auto cylinderButton = mainWindow->findChild<QToolButton *>(
      "modelEditorPaletteCylinderButton");
  QVERIFY(cylinderButton != NULL);
  QVERIFY(!cylinderButton->isChecked());
  QVERIFY(!cylinderButton->isVisible());

  // Enter the model editor
  QVERIFY(gazebo::gui::g_editModelAct != NULL);
  gazebo::gui::g_editModelAct->trigger();
  QVERIFY(cylinderButton->isVisible());

  // Check undo/redo are disabled
  QVERIFY(gazebo::gui::g_undoAct != NULL);
  QVERIFY(gazebo::gui::g_undoHistoryAct != NULL);
  QVERIFY(gazebo::gui::g_redoAct != NULL);
  QVERIFY(gazebo::gui::g_redoHistoryAct != NULL);
  QVERIFY(!gazebo::gui::g_undoAct->isEnabled());
  QVERIFY(!gazebo::gui::g_undoHistoryAct->isEnabled());
  QVERIFY(!gazebo::gui::g_redoAct->isEnabled());
  QVERIFY(!gazebo::gui::g_redoHistoryAct->isEnabled());

  // Press the cylinder button to start inserting a link
  cylinderButton->click();
  QVERIFY(cylinderButton->isChecked());

  // Press the mouse in the scene to finish inserting a link
  QTest::mouseRelease(glWidget, Qt::LeftButton, 0,
      QPoint(-mainWindow->width()*0.5, -mainWindow->height()*0.5));

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check the cylinder button is not pressed anymore
  QVERIFY(!cylinderButton->isChecked());

  // Check undo is enabled
  QVERIFY(gazebo::gui::g_undoAct->isEnabled());
  QVERIFY(gazebo::gui::g_undoHistoryAct->isEnabled());
  QVERIFY(!gazebo::gui::g_redoAct->isEnabled());
  QVERIFY(!gazebo::gui::g_redoHistoryAct->isEnabled());

  // Check the link has been inserted
  std::string insertedLinkDefaultName = "ModelPreview_0::link_0";
  auto link = scene->GetVisual(insertedLinkDefaultName);
  QVERIFY(link != NULL);

  // Undo
  gazebo::gui::g_undoAct->trigger();

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check redo is enabled
  QVERIFY(!gazebo::gui::g_undoAct->isEnabled());
  QVERIFY(!gazebo::gui::g_undoHistoryAct->isEnabled());
  QVERIFY(gazebo::gui::g_redoAct->isEnabled());
  QVERIFY(gazebo::gui::g_redoHistoryAct->isEnabled());

  // Check the link has been removed
  link = scene->GetVisual(insertedLinkDefaultName);
  QVERIFY(link == NULL);

  // Redo
  gazebo::gui::g_redoAct->trigger();

  // Check undo is enabled
  QVERIFY(gazebo::gui::g_undoAct->isEnabled());
  QVERIFY(gazebo::gui::g_undoHistoryAct->isEnabled());
  QVERIFY(!gazebo::gui::g_redoAct->isEnabled());
  QVERIFY(!gazebo::gui::g_redoHistoryAct->isEnabled());

  // Check the link has been inserted again
  link = scene->GetVisual(insertedLinkDefaultName);
  QVERIFY(link != NULL);

  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

// Generate a main function for the test
QTEST_MAIN(ModelEditorUndoTest)
