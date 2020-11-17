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
#include "gazebo/physics/Actor.hh"

#include "test/util.hh"

using namespace gazebo;

class ActorTest : public ServerFixture { };

//////////////////////////////////////////////////
TEST_F(ActorTest, Load)
{
  // Load a world with an actor
  this->Load("worlds/actor.world", true);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Get model
  auto model = world->ModelByName("actor");
  ASSERT_TRUE(model != nullptr);

  // Convert to actor
  auto actor = boost::dynamic_pointer_cast<physics::Actor>(model);
  ASSERT_TRUE(actor != nullptr);

  // Check it is active
  EXPECT_TRUE(actor->IsActive());

  // Check the SDF
  auto sdf = actor->GetSDF();
  ASSERT_TRUE(sdf != nullptr);

  EXPECT_TRUE(sdf->HasElement("skin"));
  EXPECT_TRUE(sdf->HasElement("animation"));
  EXPECT_TRUE(sdf->HasElement("script"));

  // Check the skeleton animation was loaded
  auto skelAnims = actor->SkeletonAnimations();
  EXPECT_FALSE(skelAnims.empty());
  EXPECT_EQ(skelAnims.size(), 1u);
  EXPECT_TRUE(skelAnims["walking"] != nullptr);

  // Check some params are false for actors
  EXPECT_FALSE(actor->WindMode());
  EXPECT_FALSE(actor->GetSelfCollide());
}

//////////////////////////////////////////////////
TEST_F(ActorTest, TrajectoryFromSDF)
{
  // Load a world with an actor
  this->Load("worlds/actor.world", true);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Get model
  auto model = world->ModelByName("actor");
  ASSERT_TRUE(model != nullptr);

  // Convert to actor
  auto actor = boost::dynamic_pointer_cast<physics::Actor>(model);
  ASSERT_TRUE(actor != nullptr);

  // Check initial pose
  auto startPose = actor->WorldPose();
  EXPECT_EQ(startPose, ignition::math::Pose3d::Zero);
  EXPECT_EQ(actor->ScriptTime(), 0.0);

  // Pass some time and check actor is near the target pose
  world->Step(4000);

  ignition::math::Vector3d target(1.0, 0.0, 1.0);
  EXPECT_LT((target - actor->WorldPose().Pos()).Length(), 0.1);

  // Difference betwee script time and sim time is lower than update rate
  EXPECT_LT(fabs(actor->ScriptTime() - world->SimTime().Double()), 1.0 / 30);

  // Pass some time and check actor is near the target pose
  world->Step(4000);

  target.Set(0.3, -1.0, 1.0);
  EXPECT_LT((target - actor->WorldPose().Pos()).Length(), 0.1);

  // Difference betwee script time and sim time is lower than update rate
  EXPECT_LT(fabs(actor->ScriptTime() - world->SimTime().Double()), 1.0 / 30);
}

//////////////////////////////////////////////////
TEST_F(ActorTest, ActorCollision)
{
  // Load a world with an actor
  this->Load("test/worlds/actor_collisions.world", true);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Get model
  auto model = world->ModelByName("actor");
  ASSERT_TRUE(model != nullptr);

  // Convert to actor
  auto actor = boost::dynamic_pointer_cast<physics::Actor>(model);
  ASSERT_TRUE(actor != nullptr);

  // Step until the animation ends and check actor is inactive
  world->Step(5000);
  EXPECT_FALSE(actor->IsActive());

  // Change the actor pose to simulate collision
  ignition::math::Pose3d poseCollision(3.0, 0.0, 1.0, 0.0, 0.0, 3.14);
  actor->SetWorldPose(poseCollision);

  // Pass some time and check actor went back to the final pose
  world->Step(6000);
  ignition::math::Vector3d poseTarget(2.0, 0.0, 1.0);
  EXPECT_FALSE(actor->IsActive());
  EXPECT_LT((poseTarget - actor->WorldPose().Pos()).Length(), 0.1);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

