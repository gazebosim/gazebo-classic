/*
 * Copyright 2011 Nate Koenig
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

#include "gazebo/gui/Gui.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/TimePanel.hh"
#include "gazebo/gui/MainWindow_TEST.hh"

/////////////////////////////////////////////////
void MainWindow_TEST::Wireframe()
{
  this->resMaxPercentChange = 3.0;
  this->shareMaxPercentChange = 1.0;

  this->Load("empty.world");
  QCoreApplication::processEvents();

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Wait a little bit so that time increases.
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
  }

  mainWindow->hide();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(MainWindow_TEST)
