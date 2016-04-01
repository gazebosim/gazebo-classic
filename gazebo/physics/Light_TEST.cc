/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Light.hh"

using namespace gazebo;

class LightTest : public ServerFixture { };

//////////////////////////////////////////////////
TEST_F(LightTest, Constructor)
{
  // Load a world
  this->Load("worlds/empty.world", true);
  physics::WorldPtr world = physics::get_world("default");

  // Create a base to be the light's parent
  physics::BasePtr basePtr;
  basePtr.reset(new physics::Base(physics::BasePtr()));
  basePtr->SetName("world_root_element");
  basePtr->SetWorld(world);

  // Create the light
  physics::LightPtr lightPtr(new physics::Light(basePtr));
  EXPECT_TRUE(lightPtr != NULL);
}

//////////////////////////////////////////////////
TEST_F(LightTest, LightMsg)
{
  ignition::math::Pose3d pose(1, 2, 3, 0.1, 0.2, 0.3);

  // Load a world
  this->Load("worlds/empty.world", true);
  physics::WorldPtr world = physics::get_world("default");

  // Create a base to be the light's parent
  physics::BasePtr basePtr;
  basePtr.reset(new physics::Base(physics::BasePtr()));
  basePtr->SetName("world_root_element");
  basePtr->SetWorld(world);

  // Create the light
  physics::LightPtr lightPtr(new physics::Light(basePtr));
  EXPECT_TRUE(lightPtr != NULL);

  // Create the light message
  msgs::Light lightMsg;
  lightMsg.set_name("test_light");
  msgs::Set(lightMsg.mutable_pose(), pose);
  msgs::Set(lightMsg.mutable_diffuse(), common::Color(0.4, 0.5, 0.6));
  lightMsg.set_type(msgs::Light::SPOT);

  // Process message
  lightPtr->ProcessMsg(lightMsg);

  // Check pose
  EXPECT_EQ(lightPtr->GetWorldPose(), pose);

  // Get message
  msgs::Light newLightMsg;
  lightPtr->FillMsg(newLightMsg);

  // Check message against original message
  EXPECT_EQ(lightMsg.name(), newLightMsg.name());

  EXPECT_EQ(lightMsg.diffuse().r(), newLightMsg.diffuse().r());
  EXPECT_EQ(lightMsg.diffuse().g(), newLightMsg.diffuse().g());
  EXPECT_EQ(lightMsg.diffuse().b(), newLightMsg.diffuse().b());

  EXPECT_EQ(lightMsg.pose().position().x(), newLightMsg.pose().position().x());
  EXPECT_EQ(lightMsg.pose().position().y(), newLightMsg.pose().position().y());
  EXPECT_EQ(lightMsg.pose().position().z(), newLightMsg.pose().position().z());
  EXPECT_EQ(lightMsg.pose().orientation().w(),
         newLightMsg.pose().orientation().w());
  EXPECT_EQ(lightMsg.pose().orientation().x(),
         newLightMsg.pose().orientation().x());
  EXPECT_EQ(lightMsg.pose().orientation().y(),
         newLightMsg.pose().orientation().y());
  EXPECT_EQ(lightMsg.pose().orientation().z(),
         newLightMsg.pose().orientation().z());

  EXPECT_EQ(lightMsg.type(), newLightMsg.type());
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

