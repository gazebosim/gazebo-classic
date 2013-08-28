/*
 * Copyright 2013 Open Source Robotics Foundation
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

using namespace gazebo;
class GzPhysics : public ServerFixture
{
};

/////////////////////////////////////////////////
// \brief Test setting the gravity. 
TEST_F(GzPhysics, Gravity)
{
  Load("worlds/empty_test.world");

  // Spawn a box that will eventually float up.
  SpawnBox("box", math::Vector3(1, 1, 1), math::Vector3(10, 10, 1),
      math::Vector3(0, 0, 0));

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world);

  // Get a pointer to the model
  physics::ModelPtr model = world->GetModel("box");
  ASSERT_TRUE(model);

  EXPECT_EQ(model->GetWorldPose(), math::Pose(0,0,0,0,0,0));

  SetPause(true);

  // Change gravity
  custom_exec("gz physics -g 0,0,9.8");

  world->Step(100);

  EXPECT_GT(model->GetWorldPose().pos.z,0);
}

/////////////////////////////////////////////////
// \brief Test setting the gravity. 
TEST_F(GzPhysics, Gravity)
{
  Load("worlds/empty_test.world");

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world);

  // Change step size
  custom_exec("gz physics -g 0,0,9.8");

  world->Step(100);
}



/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
