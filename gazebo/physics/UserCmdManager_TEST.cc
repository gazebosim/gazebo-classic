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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "gazebo/physics/UserCmdManager.hh"
#include "sdf/sdf.hh"

using namespace gazebo;

/////////////////////////////////////////////////
class UserCmdManagerTest : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(UserCmdManagerTest, CreateCmd)
{
  // Load a world
  Load("test/worlds/empty_test.world", true);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  // Create a manager
  physics::UserCmdManager *manager = new physics::UserCmdManager(world);
  EXPECT_TRUE(manager != NULL);

  // Create the command
  std::string id = "unique_id";
  std::string description = "Command description";
  msgs::UserCmd::Type type = msgs::UserCmd::MOVING;
  physics::UserCmd *cmd = new physics::UserCmd(id, world, description, type);
  EXPECT_TRUE(cmd != NULL);

  // Check data
  EXPECT_EQ(id, cmd->Id());
  EXPECT_EQ(description, cmd->Description());
  EXPECT_EQ(type, cmd->Type());

  // Clean up
  delete cmd;
  cmd = NULL;
  delete manager;
  manager = NULL;
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
