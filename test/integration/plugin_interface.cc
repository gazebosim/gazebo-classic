/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <cmath>
#include <gtest/gtest-spi.h>

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;

class PluginInterface : public ServerFixture
{
};

TEST_F(PluginInterface, LoadParams)
{
  // most of the tests are acutally done in PluginInterfaceTest#Load()
  EXPECT_NO_FATAL_FAILURE(
    this->Load("worlds/plugin_interface_test.world", true));

  physics::WorldPtr world = physics::get_world();
  ASSERT_NE(world, nullptr);

  ASSERT_GT(world->ModelCount(), 0u);
  physics::ModelPtr model = world->Models()[0];
  ASSERT_NE(model, nullptr);

  ASSERT_EQ(model->GetPluginCount(), 1u);

  // HACK The ASSERTs inside the plugin's Load() method are actually running
  // in a separate thread which can't be captured by GTest (so far). So we
  // instead just let that thread print out the failed tests to std::cerr and
  // then we fail here.
  ASSERT_EQ(model->GetName(), std::string("success")) <<
    "Some of the plugin tests failed. "
    "Look into standard error output for details.";
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
