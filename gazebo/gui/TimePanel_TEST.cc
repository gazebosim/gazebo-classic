/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/gui/TimePanel.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/gui/TimePanel_TEST.hh"

/////////////////////////////////////////////////
void TimePanel_TEST::ValidTimes()
{
  QBENCHMARK
  {
    this->Load("empty.world", false, false, true);

    gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
    QVERIFY(mainWindow != NULL);
    // Create the main window.
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
    gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
    QVERIFY(cam != NULL);

    // Create a new time panel widget
    gazebo::gui::TimePanel *timePanel = new gazebo::gui::TimePanel;

    // Get the percent real time line
    QLineEdit *percentEdit = timePanel->findChild<QLineEdit*>(
        "timePanelPercentRealTime");

    // Get the sim time line
    QLineEdit *simTimeEdit = timePanel->findChild<QLineEdit*>(
        "timePanelSimTime");

    // Get the real time line
    QLineEdit *realTimeEdit = timePanel->findChild<QLineEdit*>(
        "timePanelRealTime");

    // Get the fps line
    QLineEdit *fpsEdit = timePanel->findChild<QLineEdit*>("timePanelFPS");

    QVERIFY(percentEdit != NULL);
    QVERIFY(simTimeEdit != NULL);
    QVERIFY(realTimeEdit != NULL);
    QVERIFY(fpsEdit != NULL);

    // Wait a little bit so that time increases.
    for (unsigned int i = 0; i < 10000; ++i)
    {
      gazebo::common::Time::NSleep(500000);
      QCoreApplication::processEvents();
    }

    std::string txt;
    double value;

    // Make sure real time is greater than zero
    txt = realTimeEdit->text().toStdString();
    value = boost::lexical_cast<double>(txt.substr(txt.find(".")));
    QVERIFY(value > 0.0);

    // Make sure sim time is greater than zero
    txt = simTimeEdit->text().toStdString();
    value = boost::lexical_cast<double>(txt.substr(txt.find(".")));
    QVERIFY(value > 0.0);

    // Make sure the percent real time is greater than zero
    txt = percentEdit->text().toStdString();
    value = boost::lexical_cast<double>(txt.substr(0, txt.find(" ")));
    QVERIFY(value > 0.0);

    // Make sure the percent real time is greater than zero
    txt = fpsEdit->text().toStdString();
    value = boost::lexical_cast<double>(txt.substr(0, txt.find(" ")));
    QVERIFY(value > 55.0);
    QVERIFY(value < 75.0);

    cam->Fini();
    mainWindow->close();
    delete mainWindow;
  }
}

/////////////////////////////////////////////////
void TimePanel_TEST::Visibility()
{
  this->Load("empty.world");

  // Create a new time panel widget
  gazebo::gui::TimePanel *timePanel = new gazebo::gui::TimePanel;

  // Get the percent real time line
  QLineEdit *percentEdit = timePanel->findChild<QLineEdit *>(
      "timePanelPercentRealTime");

  // Get the sim time line
  QLineEdit *simTimeEdit = timePanel->findChild<QLineEdit *>(
      "timePanelSimTime");

  // Get the real time line
  QLineEdit *realTimeEdit = timePanel->findChild<QLineEdit *>(
      "timePanelRealTime");

  // Get the step button
  QAction *stepButton = timePanel->findChild<QAction *>(
      "timePanelStepAction");

  QLineEdit *iterationsEdit = timePanel->findChild<QLineEdit *>(
      "timePanelIterations");

  // visible by default
  QVERIFY(percentEdit->isVisible());
  QVERIFY(simTimeEdit->isVisible());
  QVERIFY(realTimeEdit->isVisible());
  QVERIFY(stepButton->isVisible());
  QVERIFY(iterationsEdit->isVisible());

  // hide the widgets
  timePanel->ShowRealTimeFactor(false);
  timePanel->ShowSimTime(false);
  timePanel->ShowRealTime(false);
  timePanel->ShowStepWidget(false);
  timePanel->ShowIterations(false);

  QVERIFY(!percentEdit->isVisible());
  QVERIFY(!simTimeEdit->isVisible());
  QVERIFY(!realTimeEdit->isVisible());
  QVERIFY(!stepButton->isVisible());
  QVERIFY(!iterationsEdit->isVisible());

  // show the widgets again
  timePanel->ShowRealTimeFactor(true);
  timePanel->ShowSimTime(true);
  timePanel->ShowRealTime(true);
  timePanel->ShowStepWidget(true);
  timePanel->ShowIterations(true);

  QVERIFY(percentEdit->isVisible());
  QVERIFY(simTimeEdit->isVisible());
  QVERIFY(realTimeEdit->isVisible());
  QVERIFY(stepButton->isVisible());
  QVERIFY(iterationsEdit->isVisible());
}

// Generate a main function for the test
QTEST_MAIN(TimePanel_TEST)
