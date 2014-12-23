/*
 * Copyright 2013-2014 Open Source Robotics Foundation
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

#include <boost/filesystem.hpp>
#include "ServerFixture.hh"

using namespace gazebo;
class GzModel : public ServerFixture
{
};

/////////////////////////////////////////////////
/// \brief Test spawning a model from SDF ('gz model -f')
TEST_F(GzModel, Spawn)
{
  Load("worlds/empty_test.world");

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  std::string cmd;
  boost::filesystem::path path;

  path = path / TEST_PATH / "models" / "box.sdf";

  // Spawn a model with few parameters.
  {
    cmd = "gz model -w default -m my_model -f " + path.string();

    SetPause(true);

    // Spawn a model.
    custom_exec(cmd);

    world->Step(100);

    EXPECT_TRUE(world->GetModel("my_model") != NULL);
  }

  // Spawn another box at a different location
  {
    cmd = "gz model -w default -m next_model -x 5 -y 2 -z 9000 "
      "-R 0.1 -P 0.2 -Y 0.3 -f " + path.string();

    SetPause(true);

    // Spawn a model.
    custom_exec(cmd);
    world->Step(100);

    EXPECT_TRUE(world->GetModel("next_model") != NULL);

    physics::ModelPtr model = world->GetModel("next_model");
    ASSERT_TRUE(model != NULL);

    // Check the pose of the spawned model
    math::Pose pose = model->GetWorldPose();
    math::Vector3 rpy = pose.rot.GetAsEuler();

    EXPECT_NEAR(pose.pos.x, 5, 1e-5);
    EXPECT_NEAR(pose.pos.y, 2, 1e-5);
    EXPECT_LT(pose.pos.z, 9000.0);
    EXPECT_GT(pose.pos.z, 8900.0);

    EXPECT_NEAR(rpy.x, 0.1, 1e-5);
    EXPECT_NEAR(rpy.y, 0.2, 1e-5);
    EXPECT_NEAR(rpy.z, 0.3, 1e-5);
  }
}

/////////////////////////////////////////////////
/// \brief Test spawning and deleting a model from SDF ('gz model -d')
TEST_F(GzModel, SpawnAndDelete)
{
  Load("worlds/empty_test.world");

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  boost::filesystem::path path = TEST_PATH;
  path = path / "models" / "box.sdf";

  std::string cmd = "gz model -w default -m my_model -f " + path.string();

  // Spawn a model.
  custom_exec(cmd);

  common::Time::MSleep(1000);

  EXPECT_TRUE(HasEntity("my_model"));

  // Delete a model.
  custom_exec("gz model -w default -m my_model -d ");

  common::Time::MSleep(1000);

  EXPECT_FALSE(world->GetModel("my_model"));
}

/////////////////////////////////////////////////
/// \brief Test spawning and moving a model
TEST_F(GzModel, SpawnAndMove)
{
  Load("worlds/empty.world");

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  boost::filesystem::path path = TEST_PATH;
  path = path / "models" / "box.sdf";

  std::string cmd = "cat ";
  cmd += path.string() + " | gz model -w default -m my_model -s ";

  // Spawn a model.
  custom_exec(cmd);

  common::Time::MSleep(1000);

  EXPECT_TRUE(HasEntity("my_model"));
  physics::ModelPtr model = world->GetModel("my_model");

  world->SetPaused(true);

  // Move a model.
  custom_exec("gz model -w default -m my_model -x 10 -y 11 -z 5 "
      "-R 0.1 -P 0.2 -Y 0.3");

  common::Time::MSleep(1000);

  EXPECT_EQ(model->GetWorldPose(), math::Pose(10, 11, 5, 0.1, 0.2, 0.3));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
