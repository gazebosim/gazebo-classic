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
#include "physics/physics.hh"

using namespace gazebo;
class BulletTest : public ServerFixture
{
};


TEST_F(BulletTest, EmptyWorldTest)
{
  // load an empty world with bullet physics engine
  Load("worlds/empty_bullet.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // simulate several steps
  int steps = 20;
  world->StepWorld(steps);
  double t = world->GetSimTime().Double();
  double dt = world->GetPhysicsEngine()->GetStepTime();
  EXPECT_GT(t, 0.99*dt*static_cast<double>(steps));
}

TEST_F(BulletTest, SpawnDropTest)
{
  // load an empty world with bullet physics engine
  Load("worlds/empty_bullet.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // spawn some simple shapes and check to see that they start falling
  double z0 = 3;
  SpawnBox("test_box", math::Vector3(1, 1, 1), math::Vector3(0, 0, z0),
    math::Vector3::Zero);
  SpawnSphere("test_sphere", math::Vector3(4, 0, z0), math::Vector3::Zero);
  SpawnCylinder("test_cylinder", math::Vector3(8, 0, z0), math::Vector3::Zero);

  std::list<std::string> model_names;
  model_names.push_back("test_box");
  model_names.push_back("test_sphere");
  model_names.push_back("test_cylinder");
 
  int steps = 20;
  physics::ModelPtr model;
  math::Pose pose1, pose2;
  // math::Vector3 vel1, vel2;

  for (std::list<std::string>::iterator iter = model_names.begin();
    iter != model_names.end(); ++iter)
  {
    // Make sure the model is loaded
    model = world->GetModel(*iter);
    ASSERT_TRUE(model);

    // Step forward and check downward velocity and decreasing position
    world->StepWorld(steps);
    // vel1 = model->GetWorldLinearVel();
    pose1 = model->GetWorldPose();
    // EXPECT_LT(vel1.z, 0);
    EXPECT_LT(pose1.pos.z, z0);

    world->StepWorld(steps);
    // vel2 = model->GetWorldLinearVel();
    pose2 = model->GetWorldPose();
    // EXPECT_LT(vel2.z, vel1.z);
    EXPECT_LT(pose2.pos.z, pose1.pos.z);
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
