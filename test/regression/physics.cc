/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "ServerFixture.hh"
#include "physics/Physics.hh"
#include "physics/physics.h"

using namespace gazebo;
class PhysicsTest : public ServerFixture
{
};


TEST_F(PhysicsTest, State)
{
  Load("worlds/empty.world");
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  physics::WorldState worldState = world->GetState();
  physics::ModelState modelState = worldState.GetModelState(0); 
  physics::LinkState linkState = modelState.GetLinkState(0); 
  physics::CollisionState collisionState = linkState.GetCollisionState(0); 

  math::Pose pose;
  EXPECT_EQ(1, worldState.GetModelStateCount());
  EXPECT_EQ(1, modelState.GetLinkStateCount());
  EXPECT_EQ(1, linkState.GetCollisionStateCount());
  EXPECT_EQ(pose, modelState.GetPose());
  EXPECT_EQ(pose, linkState.GetPose());
  EXPECT_EQ(pose, collisionState.GetPose());

  Unload();
  Load("worlds/shapes.world");
  world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);
  worldState = world->GetState();

  for (unsigned int i = 0; i < worldState.GetModelStateCount(); ++i)
  {
    modelState = worldState.GetModelState(i);
    if (modelState.GetName() == "plane")
      pose.Set(math::Vector3(0, 0, 0), math::Quaternion(0, 0, 0));
    else if (modelState.GetName() == "box")
      pose.Set(math::Vector3(0, 0, 0.5), math::Quaternion(0, 0, 0));
    else if (modelState.GetName() == "sphere")
      pose.Set(math::Vector3(0, 1.5, 0.5), math::Quaternion(0, 0, 0));
    else if (modelState.GetName() == "cylinder")
      pose.Set(math::Vector3(0, -1.5, 0.5), math::Quaternion(0, 0, 0));

    EXPECT_TRUE(pose == modelState.GetPose());
  }

  // Move the box
  world->GetModelByName("box")->SetWorldPose(
      math::Pose(math::Vector3(1,2,3), math::Quaternion(0, 0, 0)));

  // Make sure the box has been moved
  physics::ModelState modelState2 = world->GetState().GetModelState("box");
  pose.Set(math::Vector3(1,2,3), math::Quaternion(0, 0, 0));
  EXPECT_TRUE(pose == modelState2.GetPose());

  // Reset world state, and check for correctness
  world->SetState(worldState);
  modelState2 = world->GetState().GetModelState("box");
  pose.Set(math::Vector3(0,0,.5), math::Quaternion(0, 0, 0));
  EXPECT_TRUE(pose == modelState2.GetPose());

}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
