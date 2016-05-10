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

#include "gazebo/test/ServerFixture.hh"
#include "test/util.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/World.hh"

using namespace gazebo;

class WorldTest : public ServerFixture { };

//////////////////////////////////////////////////
TEST_F(WorldTest, AllowRenaming)
{
  // Load a world
  this->Load("worlds/blank.world", true);
  auto world = physics::get_world("default");

  std::string modelName("new_model");

  // Check the model hasn't been created
  EXPECT_TRUE(world->GetModel(modelName) == nullptr);
  EXPECT_EQ(world->GetModelCount(), 0u);

  // Spawn model
  msgs::Model msg;
  msg.set_name(modelName);
  msg.add_link();

  std::string modelSDFStr(
    "<sdf version='" + std::string(SDF_VERSION) + "'>"
    + msgs::ModelToSDF(msg)->ToString("")
    + "</sdf>");

  msgs::Factory facMsg;
  facMsg.set_sdf(modelSDFStr);
  this->factoryPub->Publish(facMsg);

  // Wait for the entity to spawn
  int sleep = 0;
  int maxSleep = 10;
  while (sleep < maxSleep && !world->GetModel(modelName))
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_TRUE(world->GetModel(modelName) != nullptr);
  EXPECT_EQ(world->GetModelCount(), 1u);

  // Try to spawn with same name without allowing renaming
  facMsg.set_sdf(modelSDFStr);
  facMsg.set_allow_renaming(false);
  this->factoryPub->Publish(facMsg);

  // Wait and check no models are inserted
  sleep = 0;
  while (sleep < maxSleep && world->GetModelCount() == 1u)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_EQ(world->GetModelCount(), 1u);

  // Now try again, but allow renaming
  facMsg.set_sdf(modelSDFStr);
  facMsg.set_allow_renaming(true);
  this->factoryPub->Publish(facMsg);

  // Check a new entity is spawned with a different name
  sleep = 0;
  while (sleep < maxSleep && !world->GetModel(modelName + "_0"))
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_TRUE(world->GetModel(modelName + "_0") != nullptr);
  EXPECT_EQ(world->GetModelCount(), 2u);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
