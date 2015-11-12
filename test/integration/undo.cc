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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/RenderWidget.hh"
#include "gazebo/gui/TimePanel.hh"
#include "gazebo/gui/UserCmdHistory.hh"
#include "undo.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void UndoTest::OnUndoRedo(ConstUndoRedoPtr &_msg)
{
  if (_msg->undo())
    this->g_undoMsgReceived = true;
  else
    this->g_redoMsgReceived = true;
}

/////////////////////////////////////////////////
void UndoTest::OnUserCmdStats(ConstUserCmdStatsPtr &_msg)
{
  g_undoCmdCount = _msg->undo_cmd_count();
  g_redoCmdCount = _msg->redo_cmd_count();
}

/////////////////////////////////////////////////
void UndoTest::MsgPassing()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", true, false, true);

  // Get world
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

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

  // Check actions
  QVERIFY(gazebo::gui::g_undoAct != NULL);
  QVERIFY(gazebo::gui::g_redoAct != NULL);
  QVERIFY(gazebo::gui::g_undoHistoryAct != NULL);
  QVERIFY(gazebo::gui::g_redoHistoryAct != NULL);

  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == false);
  QVERIFY(gazebo::gui::g_redoAct->isEnabled() == false);
  QVERIFY(gazebo::gui::g_undoHistoryAct->isEnabled() == false);
  QVERIFY(gazebo::gui::g_redoHistoryAct->isEnabled() == false);

  // Transport
  gazebo::transport::NodePtr node;
  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::PublisherPtr userCmdPub =
      node->Advertise<gazebo::msgs::UserCmd>("~/user_cmd");
  gazebo::transport::PublisherPtr undoRedoPub =
      node->Advertise<gazebo::msgs::UndoRedo>("~/undo_redo");
  gazebo::transport::SubscriberPtr undoRedoSub = node->Subscribe("~/undo_redo",
      &UndoTest::OnUndoRedo, this);
  gazebo::transport::SubscriberPtr userCmdStatsSub =
      node->Subscribe("~/user_cmd_stats",
      &UndoTest::OnUserCmdStats, this);

  // Check that no user cmd stats have been received
  QCOMPARE(g_undoCmdCount, -1);
  QCOMPARE(g_redoCmdCount, -1);

  // Publish a few command msgs as if the user was performing commands
  std::vector<gazebo::common::Time> cmdTimes;
  for (auto num : {1, 2, 3})
  {
    world->Step(100);

    gazebo::msgs::UserCmd msg;
    msg.set_description("description_" + std::to_string(num));
    msg.set_type(gazebo::msgs::UserCmd::MOVING);

    userCmdPub->Publish(msg);

    // Process some events and draw the screen
    for (size_t i = 0; i < 10; ++i)
    {
      gazebo::common::Time::MSleep(30);
      QCoreApplication::processEvents();
      mainWindow->repaint();
    }

    // Save sim times
    cmdTimes.push_back(world->GetSimTime());
    if (num == 0)
      QVERIFY(cmdTimes[num] > gazebo::common::Time::Zero);
    else
      QVERIFY(cmdTimes[num] > cmdTimes[num-1]);

    // Check that the server received the message and published proper stats
    QCOMPARE(g_undoCmdCount, num);
    QCOMPARE(g_redoCmdCount, 0);

    // Check that only undo was enabled
    QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);
    QVERIFY(gazebo::gui::g_redoAct->isEnabled() == false);
    QVERIFY(gazebo::gui::g_undoHistoryAct->isEnabled() == true);
    QVERIFY(gazebo::gui::g_redoHistoryAct->isEnabled() == false);
  }
  world->Step(100);

  // Get sim time
  gazebo::common::Time currentTime;
  gazebo::common::Time afterCmdsTime = world->GetSimTime();
  QVERIFY(afterCmdsTime > cmdTimes[2]);

  // Trigger undo
  QVERIFY(this->g_undoMsgReceived == false);
  gazebo::gui::g_undoAct->trigger();

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check undo msg was published
  QVERIFY(this->g_undoMsgReceived == true);

  // Check we went back to the last cmd time
  currentTime = world->GetSimTime();
  QVERIFY(currentTime == cmdTimes[2]);

  // Check that the server received the message and published proper stats
  QCOMPARE(g_undoCmdCount, 2);
  QCOMPARE(g_redoCmdCount, 1);

  // Check that redo is also enabled now
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_redoAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_undoHistoryAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_redoHistoryAct->isEnabled() == true);

  // Trigger redo
  QVERIFY(this->g_redoMsgReceived == false);
  gazebo::gui::g_redoAct->trigger();

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check undo msg was published
  QVERIFY(this->g_redoMsgReceived == true);

  // Check we moved forward to the moment undo was triggered
  currentTime = world->GetSimTime();
  QVERIFY(currentTime == afterCmdsTime);

  // Check that the server received the message and published proper stats
  QCOMPARE(g_undoCmdCount, 3);
  QCOMPARE(g_redoCmdCount, 0);

  // Check that redo is not enabled anymore
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_redoAct->isEnabled() == false);
  QVERIFY(gazebo::gui::g_undoHistoryAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_redoHistoryAct->isEnabled() == false);

  // Publish an undo request with skipped samples
  {
    gazebo::msgs::UndoRedo msg;
    msg.set_undo(true);
    msg.set_id(0);

    undoRedoPub->Publish(msg);
  }

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check we went back to the first cmd time
  currentTime = world->GetSimTime();
  QVERIFY(currentTime == cmdTimes[0]);

  // Check that the server received the message and published proper stats
  QCOMPARE(g_undoCmdCount, 0);
  QCOMPARE(g_redoCmdCount, 3);

  // Check that redo is enabled but undo isn't
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == false);
  QVERIFY(gazebo::gui::g_redoAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_undoHistoryAct->isEnabled() == false);
  QVERIFY(gazebo::gui::g_redoHistoryAct->isEnabled() == true);

  // Publish a redo request with skipped samples
  {
    gazebo::msgs::UndoRedo msg;
    msg.set_undo(false);
    msg.set_id(2);

    undoRedoPub->Publish(msg);
  }

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check we moved forward to the moment undo was triggered
  currentTime = world->GetSimTime();
  QVERIFY(currentTime == afterCmdsTime);

  // Check that the server received the message and published proper stats
  QCOMPARE(g_undoCmdCount, 3);
  QCOMPARE(g_redoCmdCount, 0);

  // Check that undo is enabled but redo isn't
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_redoAct->isEnabled() == false);
  QVERIFY(gazebo::gui::g_undoHistoryAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_redoHistoryAct->isEnabled() == false);

  // Clean up
  node.reset();
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void UndoTest::UndoResetTime()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world");

  // Get world
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

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

  // Get box and move it
  auto box = world->GetModel("box");
  QVERIFY(box != NULL);
  auto boxInitialPose = box->GetWorldPose();

  auto boxFinalPose = gazebo::math::Pose(10, 20, 0.5, 0, 0, 0);
  box->SetWorldPose(boxFinalPose);
  QVERIFY(box->GetWorldPose() != boxInitialPose);
  QVERIFY(box->GetWorldPose() == boxFinalPose);

  // Get sim time
  world->SetPaused(true);
  auto initialTime = world->GetSimTime();
  QVERIFY(initialTime != gazebo::common::Time::Zero);

  // Reset time
  mainWindow->GetRenderWidget()->GetTimePanel()->OnTimeReset();

  // Check time
  int sleep = 0;
  int maxSleep = 100;
  auto newTime = world->GetSimTime();
  while (newTime != gazebo::common::Time::Zero && sleep < maxSleep)
  {
    newTime = world->GetSimTime();
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Initial time [" << initialTime << "] new time [" << newTime
      << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(newTime == gazebo::common::Time::Zero);

  // Check that box pose wasn't reset
  QVERIFY(box->GetWorldPose() == boxFinalPose);
  QVERIFY(box->GetWorldPose() != boxInitialPose);

  // Undo
  QVERIFY(gazebo::gui::g_undoAct != NULL);
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);

  gazebo::gui::g_undoAct->trigger();

  // Check time is back to initial time
  sleep = 0;
  maxSleep = 10;
  newTime = world->GetSimTime();
  while (newTime != initialTime && sleep < maxSleep)
  {
    newTime = world->GetSimTime();
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Initial time [" << initialTime << "] new time [" << newTime
      << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(newTime == initialTime);
}

/////////////////////////////////////////////////
void UndoTest::UndoResetWorld()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world");

  // Get world
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

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

  // Get box and move it
  auto box = world->GetModel("box");
  QVERIFY(box != NULL);
  auto boxInitialPose = box->GetWorldPose();

  gazebo::math::Pose boxFinalPose = gazebo::math::Pose(10, 20, 0.5, 0, 0, 0);
  box->SetWorldPose(boxFinalPose);
  QVERIFY(box->GetWorldPose() != boxInitialPose);
  QVERIFY(box->GetWorldPose() == boxFinalPose);

  // Get sim time
  world->SetPaused(true);
  auto initialTime = world->GetSimTime();
  QVERIFY(initialTime != gazebo::common::Time::Zero);

  // Reset world
  gazebo::gui::g_resetWorldAct->trigger();

  // Check time and box pose
  int sleep = 0;
  int maxSleep = 100;
  auto newTime = world->GetSimTime();
  auto boxNewPose = box->GetWorldPose();
  while (newTime != gazebo::common::Time::Zero && boxNewPose != boxInitialPose
      && sleep < maxSleep)
  {
    newTime = world->GetSimTime();
    boxNewPose = box->GetWorldPose();
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Initial time [" << initialTime << "] new time [" << newTime
        << "] Initial pose [" << boxInitialPose << "] new pose [" << boxNewPose
        << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(newTime == gazebo::common::Time::Zero);
  QVERIFY(boxNewPose == boxInitialPose);

  // Undo
  QVERIFY(gazebo::gui::g_undoAct != NULL);
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);

  gazebo::gui::g_undoAct->trigger();

  // Check time and box were reset
  sleep = 0;
  newTime = world->GetSimTime();
  boxNewPose = box->GetWorldPose();
  while (newTime != initialTime && boxNewPose == boxInitialPose &&
      sleep < maxSleep)
  {
    newTime = world->GetSimTime();
    boxNewPose = box->GetWorldPose();
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Initial time [" << initialTime << "] new time [" << newTime
        << "] Initial pose [" << boxInitialPose << "] new pose [" << boxNewPose
        << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(newTime == initialTime);
  QVERIFY(boxNewPose == boxFinalPose);
}

/////////////////////////////////////////////////
void UndoTest::UndoResetModelPoses()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world");

  // Get world
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

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

  // Get box and move it
  auto box = world->GetModel("box");
  QVERIFY(box != NULL);
  auto boxInitialPose = box->GetWorldPose();

  gazebo::math::Pose boxFinalPose = gazebo::math::Pose(10, 20, 0.5, 0, 0, 0);
  box->SetWorldPose(boxFinalPose);
  QVERIFY(box->GetWorldPose() != boxInitialPose);
  QVERIFY(box->GetWorldPose() == boxFinalPose);

  // Get sim time
  world->SetPaused(true);
  auto initialTime = world->GetSimTime();
  QVERIFY(initialTime != gazebo::common::Time::Zero);

  // Reset model poses
  gazebo::gui::g_resetModelsAct->trigger();

  // Check time and box pose
  int sleep = 0;
  int maxSleep = 100;
  auto boxNewPose = box->GetWorldPose();
  while (boxNewPose != boxInitialPose && sleep < maxSleep)
  {
    boxNewPose = box->GetWorldPose();
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Initial pose [" << boxInitialPose << "] new pose [" << boxNewPose
        << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(boxNewPose == boxInitialPose);

  // Check that time wasn't reset
  QVERIFY(world->GetSimTime() != gazebo::common::Time::Zero);
  QVERIFY(world->GetSimTime() == initialTime + gazebo::common::Time(0.001));

  // Undo
  QVERIFY(gazebo::gui::g_undoAct != NULL);
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);

  gazebo::gui::g_undoAct->trigger();

  // Check time and box were reset
  sleep = 0;
  boxNewPose = box->GetWorldPose();
  while (boxNewPose == boxInitialPose && sleep < maxSleep)
  {
    boxNewPose = box->GetWorldPose();
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Initial pose [" << boxInitialPose << "] new pose [" << boxNewPose
        << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(boxNewPose == boxFinalPose);
}

// Generate a main function for the test
QTEST_MAIN(UndoTest)
