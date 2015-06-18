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

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/TimeWidget.hh"
#include "gazebo/gui/LogPlayWidget.hh"
#include "gazebo/gui/LogPlayWidget_TEST.hh"

/////////////////////////////////////////////////
void LogPlayWidget_TEST::Visibility()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

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

  // Get time widget
  gazebo::gui::TimeWidget *timeWidget = mainWindow->findChild<
      gazebo::gui::TimeWidget *>("timeWidget");

  // Get log playback widget
  gazebo::gui::LogPlayWidget *logPlayWidget = mainWindow->findChild<
      gazebo::gui::LogPlayWidget *>("logPlayWidget");

  // Check that it is not in playback mode
  QVERIFY(timeWidget->isVisible());
  QVERIFY(!logPlayWidget->isVisible());

  // Create a node and a publisher for communication.
  gazebo::transport::NodePtr node = gazebo::transport::NodePtr(
      new gazebo::transport::Node());;
  node->Init();

  gazebo::transport::PublisherPtr pub = node->Advertise<
      gazebo::msgs::WorldStatistics>("~/world_stats");

  // Create a world stats message
  gazebo::msgs::WorldStatistics msg;
  gazebo::msgs::Set(msg.mutable_sim_time(), gazebo::common::Time::Zero);
  gazebo::msgs::Set(msg.mutable_real_time(), gazebo::common::Time::Zero);
  gazebo::msgs::Set(msg.mutable_pause_time(), gazebo::common::Time::Zero);
  msg.set_iterations(0);
  msg.set_paused(true);

  gazebo::msgs::LogPlaybackStatistics logStats;
  gazebo::msgs::Set(logStats.mutable_start_time(), gazebo::common::Time::Zero);
  gazebo::msgs::Set(logStats.mutable_end_time(),
      gazebo::common::Time(200, 123));
  msg.mutable_log_playback_stats()->CopyFrom(logStats);

  pub->Publish(msg);

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check that it is in playback mode
 // QVERIFY(!timeWidget->isVisible());
//  QVERIFY(logPlayWidget->isVisible());




  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(LogPlayWidget_TEST)
