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
#include "test/ServerFixture.hh"
#include "test_config.h"

using namespace gazebo;

class Issue1434Test : public ServerFixture
{
  public: bool manipModeFired = false;

  public: void OnManipMode(const std::string& /*_mode*/)
    {
      this->manipModeFired = true;
    }
};

/////////////////////////////////////////////////
TEST_F(Issue1434Test, CheckGuiIface)
{
  Load("worlds/issue1434.world");
  gui::MainWindow* mainWindow = gazebo::gui::get_main_window();
  EXPECT_TRUE(mainWindow != NULL);

  std::string world = gazebo::gui::get_world();
  EXPECT_EQ(world, "issue1434");
}

/////////////////////////////////////////////////
TEST_F(Issue1434Test, CheckGuiEvents)
{
  Load("worlds/issue1434.world");
  // Connect to the manipMode event
  event::ConnectionPtr connection = gazebo::gui::Events::ConnectManipMode(
      boost::bind(&Issue1434Test::OnManipMode, this, _1));

  // Fire the event. This should trigger OnManipMode
  gui::Events::manipMode;

  // Wait for the callback to finish
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // OnManipMode should have set this boolean to true.
  EXPECT_TRUE(this->manipModeFired);
}
