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

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/test/ServerFixture.hh"
#include "test/util.hh"

using namespace gazebo;

class WorldTest : public ServerFixture {};

//////////////////////////////////////////////////
/// \brief Test the factory message's allow_renaming flag and unique model name
/// generation.
TEST_F(WorldTest, UniqueModelName)
{
  // Load a blank world
  this->Load("worlds/blank.world", true);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  std::string modelName("new_model");

  // Check the model hasn't been created
  EXPECT_TRUE(world->GetModel(modelName) == nullptr);
  EXPECT_EQ(world->GetModelCount(), 0u);
  EXPECT_EQ(world->UniqueModelName(modelName), modelName);

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
  ASSERT_TRUE(world->GetModel(modelName) != nullptr);
  EXPECT_EQ(world->GetModelCount(), 1u);
  EXPECT_EQ(world->UniqueModelName(modelName), modelName + "_0");

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
  EXPECT_EQ(world->UniqueModelName(modelName), modelName + "_0");

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
  ASSERT_TRUE(world->GetModel(modelName + "_0") != nullptr);
  EXPECT_EQ(world->GetModelCount(), 2u);
  EXPECT_EQ(world->UniqueModelName(modelName), modelName + "_1");
}

//////////////////////////////////////////////////
/// \brief Test publishing a factory message to edit a model.
TEST_F(WorldTest, EditName)
{
  // Load a world with simple shapes
  this->Load("worlds/blank.world", true);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);
  EXPECT_EQ(world->GetModelCount(), 0u);

  // Spawn a box
  {
    msgs::Model msg;
    msg.set_name("box");
    msgs::AddBoxLink(msg, 1.0, ignition::math::Vector3d::One);

    std::string modelSDFStr(
      "<sdf version='" + std::string(SDF_VERSION) + "'>"
      + msgs::ModelToSDF(msg)->ToString("")
      + "</sdf>");

    msgs::Factory facMsg;
    facMsg.set_sdf(modelSDFStr);
    this->factoryPub->Publish(facMsg);
  }

  // Wait for the model to be inserted
  int sleep = 0;
  int maxSleep = 10;
  while (sleep < maxSleep && !world->GetModel("box"))
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_EQ(world->GetModelCount(), 1u);

  // Check the box model weighs 1 Kg
  {
    auto box = world->GetModel("box");
    ASSERT_TRUE(box != nullptr);

    EXPECT_EQ(box->GetLinks().size(), 1u);
    auto link = box->GetLink("link_1");
    ASSERT_TRUE(link != nullptr);

    auto inertial = link->GetInertial();
    ASSERT_TRUE(inertial != nullptr);

    EXPECT_EQ(inertial->GetMass(), 1.0);
  }

  // Edit model mass
  {
    msgs::Model msg;
    msg.set_name("box");
    msgs::AddBoxLink(msg, 2.0, ignition::math::Vector3d::One);

    std::string modelSDFStr(
      "<sdf version='" + std::string(SDF_VERSION) + "'>"
      + msgs::ModelToSDF(msg)->ToString("")
      + "</sdf>");

    msgs::Factory facMsg;
    facMsg.set_sdf(modelSDFStr);
    facMsg.set_edit_name("box");
    this->factoryPub->Publish(facMsg);
  }

  // Wait for the model to change
  sleep = 0;
  while (sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }

  // World still has the same number of models
  EXPECT_EQ(world->GetModelCount(), 1u);

  // Check the box model now weighs 2 Kg
  {
    auto box = world->GetModel("box");
    ASSERT_TRUE(box != nullptr);

    EXPECT_EQ(box->GetLinks().size(), 1u);
    auto link = box->GetLink("link_1");
    ASSERT_TRUE(link != nullptr);

    auto inertial = link->GetInertial();
    ASSERT_TRUE(inertial != nullptr);

    EXPECT_EQ(inertial->GetMass(), 2.0);
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
