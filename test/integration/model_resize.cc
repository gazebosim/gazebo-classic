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
#include "gazebo/physics/physics.hh"
#include "helper_physics_generator.hh"

#define PHYSICS_TOL 1e-2
using namespace gazebo;

class ModelResizeTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  public: void SimpleShapes(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////////////////
// SimpleShapes: (Test adapted from PhysicsTest::SpawnDrop)
// Load a world, check that gravity points along z axis, spawn simple
// shapes (box, sphere, cylinder), resize them to be smaller, verify that they
// then start falling.
////////////////////////////////////////////////////////////////////////
void ModelResizeTest::SimpleShapes(const std::string &_physicsEngine)
{
  // load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  math::Vector3 g = physics->GetGravity();
  // Assume gravity vector points down z axis only.
  EXPECT_EQ(g.x, 0);
  EXPECT_EQ(g.y, 0);
  EXPECT_LE(g.z, -9.8);

  // get physics time step
  double dt = physics->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // spawn some simple shapes with unit size
  double z0 = 0.5;
  std::map<std::string, math::Vector3> modelPos;
  modelPos["test_box"] = math::Vector3(0, 0, z0);
  modelPos["test_sphere"] = math::Vector3(4, 0, z0);
  modelPos["test_cylinder"] = math::Vector3(8, 0, z0);

  SpawnBox("test_box", math::Vector3(1, 1, 1), modelPos["test_box"],
      math::Vector3::Zero);
  SpawnSphere("test_sphere", modelPos["test_sphere"], math::Vector3::Zero);
  SpawnCylinder("test_cylinder", modelPos["test_cylinder"],
      math::Vector3::Zero);

  int steps = 2;
  physics::ModelPtr model;
  math::Pose pose1, pose2;
  math::Vector3 vel1, vel2;
  double t, x0 = 0;

  // Verify the initial model pose is where we set it to be.
  for (std::map<std::string, math::Vector3>::iterator iter = modelPos.begin();
    iter != modelPos.end(); ++iter)
  {
    std::string name = iter->first;
    // Make sure the model is loaded
    model = world->GetModel(name);
    EXPECT_TRUE(model != NULL);

    pose1 = model->GetWorldPose();
    x0 = modelPos[name].x;

    EXPECT_EQ(pose1.pos.x, x0);
    EXPECT_EQ(pose1.pos.y, 0);
    EXPECT_NEAR(pose1.pos.z, z0, PHYSICS_TOL);
  }

  // resize model to half of it's size
  double scaleFactor = 0.5;
  for (std::map<std::string, math::Vector3>::iterator iter = modelPos.begin();
    iter != modelPos.end(); ++iter)
  {
    std::string name = iter->first;
    model = world->GetModel(name);
    model->SetScale(math::Vector3(scaleFactor, scaleFactor, scaleFactor));
  }

  // Model should start falling after resize. This loop steps the world forward
  // and makes sure that each model falls, expecting downward z velocity and
  // decreasing z position.
  for (std::map<std::string, math::Vector3>::iterator iter = modelPos.begin();
    iter != modelPos.end(); ++iter)
  {
    std::string name = iter->first;
    model = world->GetModel(name);
    if (model != NULL)
    {
      // Step once and check downward z velocity
      world->StepWorld(1);
      vel1 = model->GetWorldLinearVel();
      t = world->GetSimTime().Double();
      EXPECT_EQ(vel1.x, 0);
      EXPECT_EQ(vel1.y, 0);
      EXPECT_NEAR(vel1.z, g.z*t, -g.z*t*PHYSICS_TOL);
      // Need to step at least twice to check decreasing z position
      world->StepWorld(steps - 1);
      pose1 = model->GetWorldPose();
      x0 = modelPos[name].x;
      EXPECT_EQ(pose1.pos.x, x0);
      EXPECT_EQ(pose1.pos.y, 0);
      EXPECT_NEAR(pose1.pos.z, z0 + g.z/2*t*t, (z0+g.z/2*t*t)*PHYSICS_TOL);
      // Check once more and just make sure they keep falling
      world->StepWorld(steps);
      vel2 = model->GetWorldLinearVel();
      pose2 = model->GetWorldPose();
      EXPECT_LT(vel2.z, vel1.z);
      EXPECT_LT(pose2.pos.z, pose1.pos.z);
    }
    else
    {
      gzerr << "Error loading model " << name << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }

  // Predict time of contact with ground plane.
  double tHit = sqrt(2*(z0-0.5*scaleFactor) / (-g.z));
  // Time to advance, allow 0.5 s settling time.
  // This assumes inelastic collisions with the ground.
  double dtHit = tHit+0.5 - world->GetSimTime().Double();
  steps = ceil(dtHit / dt);
  EXPECT_GT(steps, 0);
  world->StepWorld(steps);

  // This loop checks the velocity and pose of each model 0.5 seconds
  // after the time of predicted ground contact. The velocity is expected
  // to be small, and the pose is expected to be underneath the initial pose.
  for (std::map<std::string, math::Vector3>::iterator iter = modelPos.begin();
    iter != modelPos.end(); ++iter)
  {
    std::string name = iter->first;
    // Make sure the model is loaded
    model = world->GetModel(name);
    if (model != NULL)
    {
      gzdbg << "Check ground contact of model " << name << '\n';
      // Check that model is resting on ground
      pose1 = model->GetWorldPose();
      x0 = modelPos[name].x;
      EXPECT_NEAR(pose1.pos.x, x0, PHYSICS_TOL);
      EXPECT_NEAR(pose1.pos.y, 0, PHYSICS_TOL);
      EXPECT_NEAR(pose1.pos.z, 0.5*scaleFactor, PHYSICS_TOL);
    }
    else
    {
      gzerr << "Error loading model " << name << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }
}

TEST_P(ModelResizeTest, SimpleShapes)
{
  SimpleShapes(GetParam());
}

INSTANTIATE_PHYSICS_ENGINES_TEST(ModelResizeTest)

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
