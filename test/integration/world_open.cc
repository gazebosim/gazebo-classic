/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <string>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

//////////////////////////////////////////////////
class WorldOpenTest : public ServerFixture,
                      public testing::WithParamInterface<const char*>
{
  /// \brief Test publishing a new world message and check the world is reset.
  /// \param[in] _physicsEngine Physics Engine type.
  public: void NewWorldMsg(const std::string &_physicsEngine);

  /// \brief Test publishing an open world message and check that another world
  /// is opened.
  /// \param[in] _physicsEngine Physics Engine type.
  public: void OpenWorldMsg(const std::string &_physicsEngine);
};

//////////////////////////////////////////////////
void WorldOpenTest::NewWorldMsg(const std::string &_physicsEngine)
{
  // Load a world with simple shapes
  this->Load("worlds/shapes.world", false, _physicsEngine);

  auto world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Check all the entities that are present
  EXPECT_TRUE(world->GetModel("ground_plane") != NULL);
  EXPECT_TRUE(world->Light("sun") != NULL);
  EXPECT_TRUE(world->GetModel("box") != NULL);
  EXPECT_TRUE(world->GetModel("sphere") != NULL);
  EXPECT_TRUE(world->GetModel("cylinder") != NULL);

  // Create transport
  auto node = transport::NodePtr(new transport::Node());
  node->Init();
  auto pub = node->Advertise<gazebo::msgs::ServerControl>(
      "/gazebo/server/control");

  // Check time is not zero
  for (unsigned int i = 0; i < 10; ++i)
    gazebo::common::Time::MSleep(100);

  auto time = world->GetSimTime();
  EXPECT_TRUE(time > common::Time::Zero);

  // Publish new world message
  gazebo::msgs::ServerControl msg;
  msg.set_new_world(true);
  pub->Publish(msg);

  // Wait for message to be received
  int sleep = 0;
  int maxSleep = 10;
  bool worldHasBeenDeleted = false;
  bool newWorldHasBeenCreated = false;
  while (sleep < maxSleep && !newWorldHasBeenCreated)
  {
    try
    {
      world = physics::get_world("default");
      if (world && worldHasBeenDeleted)
        newWorldHasBeenCreated = true;
    }
    catch (...)
    {
      worldHasBeenDeleted = true;
    }
    gazebo::common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_TRUE(worldHasBeenDeleted);
  EXPECT_TRUE(newWorldHasBeenCreated);
  ASSERT_TRUE(world != NULL);
  EXPECT_TRUE(world->GetSimTime() < time);

  EXPECT_TRUE(world->GetModel("ground_plane") != NULL);
  EXPECT_TRUE(world->Light("sun") != NULL);
  EXPECT_TRUE(world->GetModel("box") == NULL);
  EXPECT_TRUE(world->GetModel("sphere") == NULL);
  EXPECT_TRUE(world->GetModel("cylinder") == NULL);
}

//////////////////////////////////////////////////
void WorldOpenTest::OpenWorldMsg(const std::string &_physicsEngine)
{
  // load a world with simple shapes
  this->Load("worlds/empty.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Create transport
  auto node = transport::NodePtr(new transport::Node());
  node->Init();
  auto pub = node->Advertise<gazebo::msgs::ServerControl>(
      "/gazebo/server/control");

  // Array of worlds to be tested [world][model][light][world plugin]
  std::string worlds[10][10][10][10];

  worlds[0][0][0][0] = "worlds/shapes.world";
  worlds[0][1][0][0] = "ground_plane";
  worlds[0][2][0][0] = "box";
  worlds[0][3][0][0] = "cylinder";
  worlds[0][4][0][0] = "sphere";
  worlds[0][0][1][0] = "sun";

  worlds[1][0][0][0] = "worlds/lights.world";
  worlds[1][1][0][0] = "ground_plane";
  worlds[1][0][1][0] = "point";
  worlds[1][0][2][0] = "point1";
  worlds[1][0][3][0] = "point2";
  worlds[1][0][4][0] = "spot";
  worlds[1][0][5][0] = "directional";

  worlds[2][0][0][0] = "worlds/nested_model.world";
  worlds[2][1][0][0] = "ground_plane";
  worlds[2][2][0][0] = "model_00";
  worlds[2][0][1][0] = "sun";

  worlds[3][0][0][0] = "worlds/heightmap_dem.world";
  worlds[3][1][0][0] = "heightmap";
  worlds[3][0][1][0] = "sun";

  worlds[4][0][0][0] = "worlds/transporter.world";
  worlds[4][1][0][0] = "ground_plane";
  worlds[4][0][0][1] = "transporter";
  worlds[4][0][1][0] = "sun";

  worlds[5][0][0][0] = "worlds/blank.world";

  for (unsigned int i = 0; i < 10; ++i)
  {
    if (worlds[i][0][0][0] == "")
      break;

    // Publish open world message
    gazebo::msgs::ServerControl msg;
    msg.set_open_filename(worlds[i][0][0][0]);
    pub->Publish(msg);

    // Wait for message to be received
    int sleep = 0;
    int maxSleep = 10;
    while (sleep < maxSleep)
    {
      gazebo::common::Time::MSleep(100);
      sleep++;
    }

    // Get the new world
    world = physics::get_world("default");
    ASSERT_TRUE(world != NULL);

    // Check models
    int count = 0;
    for (unsigned int j = 1; j < 10; ++j)
    {
      if (worlds[i][j][0][0] == "")
        break;

      gzmsg << "Checking world [" << worlds[i][0][0][0] << "] model [" <<
          worlds[i][j][0][0] << "]" << std::endl;
      EXPECT_TRUE(world->GetModel(worlds[i][j][0][0]) != NULL);
      count++;
    }
    EXPECT_EQ(count, world->GetModelCount());

    // Check lights
    count = 0;
    for (unsigned int j = 1; j < 10; ++j)
    {
      if (worlds[i][0][j][0] == "")
        break;

      gzmsg << "Checking world [" << worlds[i][0][0][0] << "] light [" <<
          worlds[i][0][j][0] << "]" << std::endl;
      EXPECT_TRUE(world->Light(worlds[i][0][j][0]) != NULL);
      count++;
    }
    EXPECT_EQ(count, world->Lights().size());

    // Check world plugins
    count = 0;
    for (unsigned int j = 1; j < 10; ++j)
    {
      if (worlds[i][0][0][j] == "")
        break;
      count++;
    }
    EXPECT_EQ(count, world->PluginCount());
  }
}

/////////////////////////////////////////////////
TEST_P(WorldOpenTest, NewWorldMsg)
{
  this->NewWorldMsg(this->GetParam());
}

/////////////////////////////////////////////////
TEST_P(WorldOpenTest, OpenWorldMsg)
{
  this->OpenWorldMsg(this->GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, WorldOpenTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

