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
#include "gazebo/gui/HotkeyDialog.hh"
#include "gazebo/gui/HotkeyDialog_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void HotkeyDialog_TEST::HotkeyChart()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);

  // Create the main window.
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Check there is no hotkey dialog
  gazebo::gui::HotkeyDialog *hotkeyDialog =
      mainWindow->findChild<gazebo::gui::HotkeyDialog *>("hotkeyChart");
  QVERIFY(hotkeyDialog == NULL);

  // Trigger hotkey chart action
  gazebo::gui::g_hotkeyChartAct->trigger();

  // Check that dialog was created and is visible
  hotkeyDialog =
      mainWindow->findChild<gazebo::gui::HotkeyDialog *>("hotkeyChart");
  QVERIFY(hotkeyDialog != NULL);
  QVERIFY(hotkeyDialog->isVisible());

  // Press Esc to close it
  QTest::keyClick(hotkeyDialog, Qt::Key_Escape);

  // Check the dialog is hidden
  QVERIFY(!hotkeyDialog->isVisible());

  // Trigger hotkey chart action again
  gazebo::gui::g_hotkeyChartAct->trigger();

  // Check that dialog is open
  QVERIFY(hotkeyDialog->isVisible());

  // Terminate
  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(HotkeyDialog_TEST)
