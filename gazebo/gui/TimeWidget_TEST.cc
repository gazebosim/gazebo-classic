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

#include "gazebo/gui/TimeWidget.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/RenderWidget.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/Actions.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/gui/TimeWidget_TEST.hh"

/////////////////////////////////////////////////
void TimeWidget_TEST::Reset()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("empty.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);

  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Wait a little bit so that time increases.
  this->ProcessEventsAndDraw(NULL, 10, 100);

  // Get the time panel
  gazebo::gui::TimePanel *timePanel = mainWindow->RenderWidget()->
      GetTimePanel();
  QVERIFY(timePanel != NULL);

  // Get the time widget
  gazebo::gui::TimeWidget *timeWidget =
      timePanel->findChild<gazebo::gui::TimeWidget *>("timeWidget");
  QVERIFY(timeWidget != NULL);
  timeWidget->setVisible(true);

  // Get the sim time line edit
  QLineEdit *simTimeEdit = timeWidget->findChild<QLineEdit *>(
      "timeWidgetSimTime");
  QVERIFY(simTimeEdit != NULL);

  // Get the real time line edit
  QLineEdit *realTimeEdit = timeWidget->findChild<QLineEdit *>(
      "timeWidgetRealTime");
  QVERIFY(realTimeEdit != NULL);

  QLineEdit *iterationsEdit = timeWidget->findChild<QLineEdit *>(
      "timeWidgetIterations");
  QVERIFY(iterationsEdit != NULL);

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

  // Make sure iteration is greater than zero
  txt = iterationsEdit->text().toStdString();
  int intValue = boost::lexical_cast<int>(txt);
  QVERIFY(intValue > 0);

  // verify reset action is not null
  QVERIFY(gazebo::gui::g_resetWorldAct != NULL);
  QVERIFY(gazebo::gui::g_resetWorldAct->isEnabled());
  QVERIFY(!mainWindow->IsPaused());

  // pause the world before resetting
  mainWindow->Pause();

  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(mainWindow->IsPaused());

  // trigger reset world
  gazebo::gui::g_resetWorldAct->trigger();

  this->ProcessEventsAndDraw(mainWindow);

  // Make sure real time is zero
  txt = realTimeEdit->text().toStdString();
  value = boost::lexical_cast<double>(txt.substr(txt.find(".")));
  QVERIFY(ignition::math::equal(value, 0.0));

  // Make sure sim time is zero
  txt = simTimeEdit->text().toStdString();
  value = boost::lexical_cast<double>(txt.substr(txt.find(".")));
  QVERIFY(ignition::math::equal(value, 0.0));

  // Make sure iteration is zero
  txt = iterationsEdit->text().toStdString();
  intValue = boost::lexical_cast<int>(txt);
  QCOMPARE(intValue, 0);

  // play the simulation and again and verify time increases
  mainWindow->Play();

  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(!mainWindow->IsPaused());

  // Wait a little bit so that time increases.
  this->ProcessEventsAndDraw(NULL, 10, 100);

  // Make sure real time is greater than zero
  txt = realTimeEdit->text().toStdString();
  value = boost::lexical_cast<double>(txt.substr(txt.find(".")));
  QVERIFY(value > 0.0);

  // Make sure sim time is greater than zero
  txt = simTimeEdit->text().toStdString();
  value = boost::lexical_cast<double>(txt.substr(txt.find(".")));
  QVERIFY(value > 0.0);

  // Make sure iteration is greater than zero
  txt = iterationsEdit->text().toStdString();
  intValue = boost::lexical_cast<int>(txt);
  QVERIFY(intValue > 0);

  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void TimeWidget_TEST::ValidTimes()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  QBENCHMARK
  {
    this->Load("empty.world", false, false, false);

    // Create the main window.
    gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
    QVERIFY(mainWindow != NULL);

    mainWindow->Load();
    mainWindow->Init();
    mainWindow->show();

    this->ProcessEventsAndDraw(mainWindow);

    gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
    QVERIFY(cam != NULL);

    // Get the time panel
    gazebo::gui::TimePanel *timePanel = mainWindow->RenderWidget()->
        GetTimePanel();
    QVERIFY(timePanel != NULL);

    // Get the time widget
    gazebo::gui::TimeWidget *timeWidget =
        timePanel->findChild<gazebo::gui::TimeWidget *>("timeWidget");
    QVERIFY(timeWidget != NULL);
    timeWidget->setVisible(true);

    // Get the percent real time line
    QLineEdit *percentEdit = timeWidget->findChild<QLineEdit *>(
        "timeWidgetPercentRealTime");
    QVERIFY(percentEdit != NULL);

    // Get the sim time line
    QLineEdit *simTimeEdit = timeWidget->findChild<QLineEdit *>(
        "timeWidgetSimTime");
    QVERIFY(simTimeEdit != NULL);

    // Get the real time line
    QLineEdit *realTimeEdit = timeWidget->findChild<QLineEdit *>(
        "timeWidgetRealTime");
    QVERIFY(realTimeEdit != NULL);

    // Get the fps line
    QLineEdit *fpsEdit = timeWidget->findChild<QLineEdit *>("timeWidgetFPS");
    QVERIFY(fpsEdit != NULL);

    // some machines are unable to hit the target FPS
    // sample update time and determine whether to skip FPS lower bound check
    bool skipFPSTest = false;
    gazebo::common::Time t = gazebo::common::Time::GetWallTime();
    QCoreApplication::processEvents();
    double dt = (gazebo::common::Time::GetWallTime()-t).Double();
    if (dt >= 0.01)
    {
      std::cerr << "Skipping lower bound FPS check" << std::endl;
      skipFPSTest = true;
    }
    unsigned int iterations = skipFPSTest ? 50 : 5000;
    double lowerFPSBound = skipFPSTest ? 0 : 45;

    // Wait a little bit so that time increases.
    this->ProcessEventsAndDraw(NULL, iterations, 1);

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

    // Make sure the fps is somewhere close to 60 fps
    txt = fpsEdit->text().toStdString();
    value = boost::lexical_cast<double>(txt.substr(0, txt.find(" ")));

    QVERIFY(value > lowerFPSBound);
    QVERIFY(value < 75.0);

    cam->Fini();
    mainWindow->close();
    delete mainWindow;
  }
}

/////////////////////////////////////////////////
void TimeWidget_TEST::Visibility()
{
  this->Load("empty.world", false, false, false);

  // Create a new time widget
  gazebo::gui::TimeWidget *timeWidget = new gazebo::gui::TimeWidget;
  timeWidget->setVisible(true);
  QVERIFY(timeWidget->isVisible());

  // Get the percent real time line
  QLineEdit *percentEdit = timeWidget->findChild<QLineEdit *>(
      "timeWidgetPercentRealTime");

  // Get the sim time line
  QLineEdit *simTimeEdit = timeWidget->findChild<QLineEdit *>(
      "timeWidgetSimTime");

  // Get the real time line
  QLineEdit *realTimeEdit = timeWidget->findChild<QLineEdit *>(
      "timeWidgetRealTime");

  // Get the step button
  QAction *stepButton = timeWidget->findChild<QAction *>(
      "timeWidgetStepAction");

  QLineEdit *iterationsEdit = timeWidget->findChild<QLineEdit *>(
      "timeWidgetIterations");

  // visible by default
  QVERIFY(percentEdit->isVisible());
  QVERIFY(simTimeEdit->isVisible());
  QVERIFY(realTimeEdit->isVisible());
  QVERIFY(stepButton->isVisible());
  QVERIFY(iterationsEdit->isVisible());

  // hide the widgets
  timeWidget->ShowRealTimeFactor(false);
  timeWidget->ShowSimTime(false);
  timeWidget->ShowRealTime(false);
  timeWidget->ShowStepWidget(false);
  timeWidget->ShowIterations(false);

  QVERIFY(!percentEdit->isVisible());
  QVERIFY(!simTimeEdit->isVisible());
  QVERIFY(!realTimeEdit->isVisible());
  QVERIFY(!stepButton->isVisible());
  QVERIFY(!iterationsEdit->isVisible());

  // show the widgets again
  timeWidget->ShowRealTimeFactor(true);
  timeWidget->ShowSimTime(true);
  timeWidget->ShowRealTime(true);
  timeWidget->ShowStepWidget(true);
  timeWidget->ShowIterations(true);

  QVERIFY(percentEdit->isVisible());
  QVERIFY(simTimeEdit->isVisible());
  QVERIFY(realTimeEdit->isVisible());
  QVERIFY(stepButton->isVisible());
  QVERIFY(iterationsEdit->isVisible());

  delete timeWidget;
}

// Generate a main function for the test
QTEST_MAIN(TimeWidget_TEST)
