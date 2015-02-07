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

#include <gtest/gtest.h>
#include <chrono>
#include <thread>

#include "gazebo/common/Event.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "1434_gui_plugin.hh"
//#include "test/ServerFixture.hh"
//#include "test_config.h"

using namespace gazebo;


/////////////////////////////////////////////////
void Issue1434Test::CheckGuiIface()
{
  this->Load("worlds/issue1434.world");

  /*gui::MainWindow* mainWindow = gazebo::gui::get_main_window();
  QVERIFY(mainWindow != NULL);*/

  gazebo::gui::load();
  gazebo::gui::init();

  std::string world = gazebo::gui::get_world();
  QVERIFY(world == "issue1434");
}
/////////////////////////////////////////////////
void Issue1434Test::CheckGuiEventsSuccess()
{
  this->Load("worlds/empty.world");

  gazebo::gui::load();
  gazebo::gui::init();

  // Connect to the manipMode event
  event::ConnectionPtr connection = gazebo::gui::Events::ConnectManipMode(
      boost::bind(&Issue1434Test::OnManipMode, this, _1));

  // Fire the event. This should trigger OnManipMode
  gui::Events::manipMode("test");

  // Wait plenty of time for the callback to finish
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // OnManipMode should have set this boolean to true.
  QVERIFY(this->manipModeFired);
}
/////////////////////////////////////////////////
void Issue1434Test::CheckGuiEvents()
{
  this->Load("worlds/issue1434.world");

  gazebo::gui::load();
  gazebo::gui::init();

  // Connect to the manipMode event
  event::ConnectionPtr connection = gazebo::gui::Events::ConnectManipMode(
      boost::bind(&Issue1434Test::OnManipMode, this, _1));

  // Fire the event. This should trigger OnManipMode
  gui::Events::manipMode("test");

  // Wait plenty of time for the callback to finish
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // OnManipMode should have set this boolean to true.
  QVERIFY(this->manipModeFired);
}

QTEST_MAIN(Issue1434Test)
