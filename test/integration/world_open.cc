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
  // load a world with simple shapes
  this->Load("worlds/shapes.world", true, _physicsEngine);
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

  // Publish new world message
  gazebo::msgs::ServerControl msg;
  msg.set_new_world(true);
  pub->Publish(msg);

  // Wait for message to be received
  int sleep = 0;
  int maxSleep = 10;
  while (sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    sleep++;
  }

  // Check that now we are in the empty world
  auto newWorld = physics::get_world("default");
  ASSERT_TRUE(newWorld != NULL);

  EXPECT_TRUE(newWorld->GetModel("ground_plane") != NULL);
  EXPECT_TRUE(newWorld->Light("sun") != NULL);
  EXPECT_TRUE(newWorld->GetModel("box") == NULL);
  EXPECT_TRUE(newWorld->GetModel("sphere") == NULL);
  EXPECT_TRUE(newWorld->GetModel("cylinder") == NULL);
}

//////////////////////////////////////////////////
void WorldOpenTest::OpenWorldMsg(const std::string &_physicsEngine)
{
  // load a world with simple shapes
  this->Load("worlds/shapes.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Check all the entities that are present
  EXPECT_TRUE(world->GetModel("ground_plane") != NULL);
  EXPECT_TRUE(world->Light("sun") != NULL);
  EXPECT_TRUE(world->GetModel("box") != NULL);
  EXPECT_TRUE(world->GetModel("sphere") != NULL);
  EXPECT_TRUE(world->GetModel("cylinder") != NULL);
  EXPECT_TRUE(world->Light("point") == NULL);
  EXPECT_TRUE(world->Light("point1") == NULL);
  EXPECT_TRUE(world->Light("point2") == NULL);
  EXPECT_TRUE(world->Light("spot") == NULL);
  EXPECT_TRUE(world->Light("directional") == NULL);

  // Create transport
  auto node = transport::NodePtr(new transport::Node());
  node->Init();
  auto pub = node->Advertise<gazebo::msgs::ServerControl>(
      "/gazebo/server/control");

  // Publish open world message
  gazebo::msgs::ServerControl msg;
  msg.set_open_filename("worlds/lights.world");
  pub->Publish(msg);

  // Wait for message to be received
  int sleep = 0;
  int maxSleep = 10;
  while (sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    sleep++;
  }

  // Check that now we are in the empty world
  auto newWorld = physics::get_world("default");
  ASSERT_TRUE(newWorld != NULL);

  EXPECT_TRUE(newWorld->GetModel("ground_plane") != NULL);
  EXPECT_TRUE(newWorld->Light("sun") != NULL);
  EXPECT_TRUE(newWorld->GetModel("box") == NULL);
  EXPECT_TRUE(newWorld->GetModel("sphere") == NULL);
  EXPECT_TRUE(newWorld->GetModel("cylinder") == NULL);
  EXPECT_TRUE(newWorld->Light("point") != NULL);
  EXPECT_TRUE(newWorld->Light("point1") != NULL);
  EXPECT_TRUE(newWorld->Light("point2") != NULL);
  EXPECT_TRUE(newWorld->Light("spot") != NULL);
  EXPECT_TRUE(newWorld->Light("directional") != NULL);
}

/////////////////////////////////////////////////
TEST_P(WorldOpenTest, NewWorldMsg)
{
  this->NewWorldMsg(GetParam());
}

/////////////////////////////////////////////////
TEST_P(WorldOpenTest, OpenWorldMsg)
{
  this->NewWorldMsg(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, WorldOpenTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

