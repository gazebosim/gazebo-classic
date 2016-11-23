/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include "gazebo/gui/RenderWidget.hh"
#include "gazebo/gui/TimePanel.hh"
#include "gazebo/gui/TimePanel_TEST.hh"

/////////////////////////////////////////////////
void TimePanel_TEST::SetPaused()
{
  this->Load("empty.world", false, false, false);

  // Create a new time panel widget
  gazebo::gui::TimePanel *timePanel = new gazebo::gui::TimePanel;
  QVERIFY(timePanel != NULL);

  // verify initial state
  QVERIFY(!timePanel->IsPaused());

  // set paused state and verify
  timePanel->SetPaused(true);
  QVERIFY(timePanel->IsPaused());

  timePanel->SetPaused(false);
  QVERIFY(!timePanel->IsPaused());
  delete timePanel;
}

/////////////////////////////////////////////////
void TimePanel_TEST::SpaceBar()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("empty.world", false, false, false);

  // Create the main window.
  auto mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get the time panel
  auto timePanel = mainWindow->RenderWidget()->GetTimePanel();
  QVERIFY(timePanel != NULL);

  // verify initial state
  QVERIFY(!timePanel->IsPaused());

  // Press space bar
  QTest::keyClick(timePanel, Qt::Key_Space);

  // Process some events until it gets paused
  for (unsigned int i = 0; i < 10 && !timePanel->IsPaused(); ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }
// The following expectation fails on our Ubuntu jenkins machines,
// but not locally (issue #1958)
// disabling for now
#ifndef __linux__
  QVERIFY(timePanel->IsPaused());
#endif

  // Press space bar
  QTest::keyClick(timePanel, Qt::Key_Space);

  // Process some events until it gets unpaused
  for (unsigned int i = 0; i < 10 && timePanel->IsPaused(); ++i)
  {
    this->ProcessEventsAndDraw(mainWindow, 1);
  }
  QVERIFY(!timePanel->IsPaused());

  delete timePanel;
}

// Generate a main function for the test
QTEST_MAIN(TimePanel_TEST)
