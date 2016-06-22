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

#include "gazebo/physics/PhysicsIface.hh"

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/TransportIface.hh"

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

  /// \brief Test opening and closing worlds with sensors.
  /// \param[in] _physicsEngine Physics Engine type.
  public: void OpenWorldSensors(const std::string &_physicsEngine);
};

//////////////////////////////////////////////////
void WorldOpenTest::NewWorldMsg(const std::string &_physicsEngine)
{
  // Load a world with simple shapes
  this->Load("worlds/shapes.world", false, _physicsEngine);

  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Check all the entities that are present
  EXPECT_TRUE(world->GetModel("ground_plane") != nullptr);
  EXPECT_TRUE(world->Light("sun") != nullptr);
  EXPECT_TRUE(world->GetModel("box") != nullptr);
  EXPECT_TRUE(world->GetModel("sphere") != nullptr);
  EXPECT_TRUE(world->GetModel("cylinder") != nullptr);

  // Terminate ServerFixture's default transport
  {
    this->factoryPub.reset();
    this->requestPub.reset();
    this->node->Fini();
    this->node.reset();
  }

  // Create new transport
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
  int maxSleep = 100;
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
    catch(...)
    {
      worldHasBeenDeleted = true;
    }
    gazebo::common::Time::MSleep(300);
    sleep++;
  }
  EXPECT_TRUE(worldHasBeenDeleted);
  EXPECT_TRUE(newWorldHasBeenCreated);
  ASSERT_TRUE(world != nullptr);
  EXPECT_TRUE(world->GetSimTime() < time);

  EXPECT_TRUE(world->GetModel("ground_plane") != nullptr);
  EXPECT_TRUE(world->Light("sun") != nullptr);
  EXPECT_TRUE(world->GetModel("box") == nullptr);
  EXPECT_TRUE(world->GetModel("sphere") == nullptr);
  EXPECT_TRUE(world->GetModel("cylinder") == nullptr);
}

//////////////////////////////////////////////////
void WorldOpenTest::OpenWorldMsg(const std::string &_physicsEngine)
{
  // load a world with simple shapes
  this->Load("worlds/empty.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Terminate ServerFixture's default transport
  {
    this->factoryPub.reset();
    this->requestPub.reset();
    this->node->Fini();
    this->node.reset();
  }

  // Create new transport
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

  // Invalid world keeps previous world
  worlds[3][0][0][0] = "invalid_world";
  worlds[3][1][0][0] = "ground_plane";
  worlds[3][2][0][0] = "model_00";
  worlds[3][0][1][0] = "sun";

  worlds[4][0][0][0] = "worlds/heightmap.world";
  worlds[4][1][0][0] = "box1";
  worlds[4][2][0][0] = "box2";
  worlds[4][3][0][0] = "box3";
  worlds[4][4][0][0] = "box4";
  worlds[4][5][0][0] = "heightmap";
  worlds[4][0][1][0] = "sun";

  worlds[5][0][0][0] = "worlds/transporter.world";
  worlds[5][1][0][0] = "ground_plane";
  worlds[5][0][0][1] = "transporter";
  worlds[5][0][1][0] = "sun";

  worlds[6][0][0][0] = "worlds/blank.world";

  for (unsigned int i = 0; i < 10; ++i)
  {
    if (worlds[i][0][0][0] == "")
      break;

    // Publish open world message
    gazebo::msgs::ServerControl msg;
    msg.set_open_filename(worlds[i][0][0][0]);
    pub->Publish(msg);

    // Wait for world to be created
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
      catch(...)
      {
        worldHasBeenDeleted = true;
      }
      gazebo::common::Time::MSleep(1000);
      sleep++;
    }
    // Check that old world was deleted and new world was created, unless the
    // world is invalid
    EXPECT_EQ(worldHasBeenDeleted, worlds[i][0][0][0] != "invalid_world");
    EXPECT_EQ(newWorldHasBeenCreated, worlds[i][0][0][0] != "invalid_world");
    ASSERT_TRUE(world != nullptr);

    EXPECT_TRUE(world->SensorsInitialized());
    EXPECT_TRUE(world->PluginsLoaded());

    // Check models
    size_t count = 0;
    for (unsigned int j = 1; j < 10; ++j)
    {
      if (worlds[i][j][0][0] == "")
        break;

      gzmsg << "Checking world [" << worlds[i][0][0][0] << "] model [" <<
          worlds[i][j][0][0] << "]" << std::endl;
      EXPECT_TRUE(world->GetModel(worlds[i][j][0][0]) != nullptr);
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
      EXPECT_TRUE(world->Light(worlds[i][0][j][0]) != nullptr);
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

//////////////////////////////////////////////////
void WorldOpenTest::OpenWorldSensors(const std::string &_physicsEngine)
{
  // Load a world with cameras
  this->Load("worlds/camera.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Sleep to ensure transport topics are all advertised
  world->Step(100);
  common::Time::MSleep(100);

  // Check the sensors are there
  {
    EXPECT_TRUE(world->SensorsInitialized());
    EXPECT_TRUE(sensors::SensorManager::Instance()->SensorsInitialized());
    EXPECT_EQ(sensors::SensorManager::Instance()->GetSensors().size(), 2u);

    auto camera = world->GetModel("camera");
    EXPECT_TRUE(camera != nullptr);
    auto camera2 = world->GetModel("camera 2");
    EXPECT_TRUE(camera2 != nullptr);

    auto cameraLink = camera->GetLink("link");
    EXPECT_TRUE(cameraLink != nullptr);
    auto camera2Link = camera2->GetLink("link");
    EXPECT_TRUE(camera2Link != nullptr);

    // Check they have sensors
    EXPECT_EQ(cameraLink->GetSensorCount(), 1u);
    EXPECT_EQ(camera2Link->GetSensorCount(), 1u);
  }

  // Check advertised topics
  auto msgTypes = gazebo::transport::getAdvertisedTopics();
  EXPECT_FALSE(msgTypes.empty());

  // Check there are camera topics being advertised
  int count1 = 0;
  for (auto msgType : msgTypes)
  {
    for (auto topic : msgType.second)
    {
      if (topic.find("camera") != std::string::npos)
      {
        gzdbg << "Found camera topic [" << topic << "]" << std::endl;
        count1++;
      }
    }
  }
  EXPECT_GT(count1, 0);

  // Terminate ServerFixture's default transport
  {
    this->factoryPub.reset();
    this->requestPub.reset();
    this->node->Fini();
    this->node.reset();
  }

  // Create new transport
  auto node = transport::NodePtr(new transport::Node());
  node->Init();
  auto pub = node->Advertise<gazebo::msgs::ServerControl>(
      "/gazebo/server/control");

  // Publish new world message
  {
    gazebo::msgs::ServerControl msg;
    msg.set_new_world(true);
    pub->Publish(msg);
  }

  // Wait for old world to be removed and new world opened.
  int sleep = 0;
  int maxSleep = 30;
  while (sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    sleep++;
  }

  // Get the new world
  world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);
  world->SetPaused(true);
  world->Step(100);
  common::Time::MSleep(100);

  // Check there are no sensors
  {
    EXPECT_TRUE(world->SensorsInitialized());
    EXPECT_TRUE(sensors::SensorManager::Instance()->SensorsInitialized());
    EXPECT_EQ(sensors::SensorManager::Instance()->GetSensors().size(), 0u);

    auto camera = world->GetModel("camera");
    EXPECT_TRUE(camera == nullptr);
    auto camera2 = world->GetModel("camera 2");
    EXPECT_TRUE(camera2 == nullptr);
  }

  // Check advertised topics
  msgTypes = gazebo::transport::getAdvertisedTopics();
  EXPECT_FALSE(msgTypes.empty());

  // Check there are no camera topics being advertised
  int count2 = 0;
  for (auto msgType : msgTypes)
  {
    for (auto topic : msgType.second)
    {
      if (topic.find("camera") != std::string::npos)
      {
        gzdbg << "Found camera topic [" << topic << "]" << std::endl;
        count2++;
      }
    }
  }
  EXPECT_EQ(count2, 0);

  // Publish open world message
  {
    gazebo::msgs::ServerControl msg;
    msg.set_open_filename("worlds/magnetometer.world");
    pub->Publish(msg);
  }

  // Wait for message to be received
  sleep = 0;
  while (sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    sleep++;
  }

  // Get the new world
  world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);
  world->SetPaused(true);
  world->Step(100);
  common::Time::MSleep(100);

  // Check sensor
  {
    EXPECT_TRUE(world->SensorsInitialized());
    EXPECT_TRUE(sensors::SensorManager::Instance()->SensorsInitialized());
    EXPECT_EQ(sensors::SensorManager::Instance()->GetSensors().size(), 1u);

    auto model = world->GetModel("magnetometerModel");
    EXPECT_TRUE(model != nullptr);

    auto link = model->GetLink("link");
    EXPECT_TRUE(link != nullptr);

    EXPECT_EQ(link->GetSensorCount(), 1u);
  }

  // Check advertised topics
  msgTypes = gazebo::transport::getAdvertisedTopics();
  EXPECT_FALSE(msgTypes.empty());

  // Check there are magnetometer topics
  int count3 = 0;
  for (auto msgType : msgTypes)
  {
    for (auto topic : msgType.second)
    {
      if (topic.find("magnetometer") != std::string::npos)
      {
        gzdbg << "Found magnetometer topic [" << topic << "]" << std::endl;
        count3++;
      }
    }
  }
  EXPECT_EQ(count3, 1);

  // Publish open world message
  {
    gazebo::msgs::ServerControl msg;
    msg.set_open_filename("worlds/camera.world");
    pub->Publish(msg);
  }

  // Wait for message to be received
  sleep = 0;
  while (sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    sleep++;
  }

  // Get the new world
  world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);
  world->SetPaused(true);
  world->Step(100);
  common::Time::MSleep(100);

  // Check the sensors are there
  {
    EXPECT_TRUE(world->SensorsInitialized());
    EXPECT_TRUE(sensors::SensorManager::Instance()->SensorsInitialized());
    EXPECT_EQ(sensors::SensorManager::Instance()->GetSensors().size(), 2u);

    auto camera = world->GetModel("camera");
    EXPECT_TRUE(camera != nullptr);
    auto camera2 = world->GetModel("camera 2");
    EXPECT_TRUE(camera2 != nullptr);

    auto cameraLink = camera->GetLink("link");
    EXPECT_TRUE(cameraLink != nullptr);
    auto camera2Link = camera2->GetLink("link");
    EXPECT_TRUE(camera2Link != nullptr);

    // Check they have sensors
    EXPECT_EQ(cameraLink->GetSensorCount(), 1u);
    EXPECT_EQ(camera2Link->GetSensorCount(), 1u);
  }

  // Check advertised topics
  msgTypes = gazebo::transport::getAdvertisedTopics();
  EXPECT_FALSE(msgTypes.empty());

  // Check there are camera topics being advertised
  int count4 = 0;
  for (auto msgType : msgTypes)
  {
    for (auto topic : msgType.second)
    {
      if (topic.find("camera") != std::string::npos)
      {
        gzdbg << "Found camera topic [" << topic << "]" << std::endl;
        count4++;
      }
    }
  }
  EXPECT_GT(count4, 0);

  // Check there are as many topics as the previous camera world
  EXPECT_EQ(count4, count1);
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

/////////////////////////////////////////////////
TEST_P(WorldOpenTest, OpenWorldSensors)
{
  this->OpenWorldSensors(this->GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, WorldOpenTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

