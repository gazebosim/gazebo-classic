/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#include "gazebo/gui/SplashScreen.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/SplashScreen_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void SplashScreen_TEST::Show()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  gazebo::gui::SplashScreen *splashObj = new gazebo::gui::SplashScreen();
  QVERIFY(splashObj != NULL);

  QVERIFY(splashObj->Visible());

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();

  QVERIFY(mainWindow != NULL);
  // Create the main window.
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Process events.
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(210);
    QCoreApplication::processEvents();
  }

  QVERIFY(!splashObj->Visible());

  mainWindow->close();
  delete mainWindow;
  delete splashObj;
}

// Generate a main function for the test
QTEST_MAIN(SplashScreen_TEST)
