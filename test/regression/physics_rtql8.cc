/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "SimplePendulumIntegrator.hh"

#define PHYSICS_TOL 1e-2
using namespace gazebo;
class PhysicsTest : public ServerFixture
{
  public: void EmptyWorld_JS(std::string _worldFile);

  public: void EmptyWorld(const std::string &_physicsEngine);
  public: void SpawnDrop(const std::string &_physicsEngine);
  public: void SpawnDropCoGOffset(const std::string &_physicsEngine);
  public: void RevoluteJoint(const std::string &_physicsEngine);
  public: void SimplePendulum(const std::string &_physicsEngine);
  public: void CollisionFiltering(const std::string &_physicsEngine);
};

void PhysicsTest::EmptyWorld_JS(std::string _worldFile)
{
  // Load an empty world
  Load(_worldFile, true);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  // simulate a couple seconds
  world->StepWorld(2000);
  double t = world->GetSimTime().Double();
  // verify that time moves forward
  EXPECT_GT(t, 0);

  Unload();
}

#ifdef HAVE_RTQL8
TEST_F(PhysicsTest, RTQL8_FIRST_TEST)
{
  EmptyWorld("worlds/rtql8/falling_box.world");
}
#endif // HAVE_RTQL8

////////////////////////////////////////////////////////////////////////
// EmptyWorld:
// Load a world, take a few steps, and verify that time is increasing.
// This is the most basic physics engine test.
////////////////////////////////////////////////////////////////////////
void PhysicsTest::EmptyWorld(const std::string &_physicsEngine)
{
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // simulate 1 step
  world->StepWorld(1);
  double t = world->GetSimTime().Double();
  // verify that time moves forward
  EXPECT_GT(t, 0);

  // simulate a few steps
  int steps = 20;
  world->StepWorld(steps);
  double dt = world->GetPhysicsEngine()->GetStepTime();
  EXPECT_GT(dt, 0);
  t = world->GetSimTime().Double();
  EXPECT_GT(t, 0.99*dt*static_cast<double>(steps+1));
}

//TEST_F(PhysicsTest, EmptyWorldODE)
//{
//  EmptyWorld("ode");
//}

//#ifdef HAVE_BULLET
//TEST_F(PhysicsTest, EmptyWorldBullet)
//{
//  EmptyWorld("bullet");
//}
//#endif  // HAVE_BULLET

//#ifdef HAVE_RTQL8
//TEST_F(PhysicsTest, EmptyWorldRTQL8)
//{
//  EmptyWorld("rtql8");
//}
//#endif  // HAVE_RTQL8

////////////////////////////////////////////////////////////////////////
// SpawnDrop:
// Load a world, check that gravity points along z axis, spawn simple
// shapes (box, sphere, cylinder), verify that they fall and hit the
// ground plane. The test currently assumes inelastic collisions.
////////////////////////////////////////////////////////////////////////
void PhysicsTest::SpawnDrop(const std::string &_physicsEngine)
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
  double dt = physics->GetStepTime();
  EXPECT_GT(dt, 0);

  // spawn some simple shapes and check to see that they start falling
  double z0 = 3;
  std::map<std::string, math::Vector3> modelPos;
  modelPos["test_box"] = math::Vector3(0, 0, z0);
  modelPos["test_sphere"] = math::Vector3(4, 0, z0);
  modelPos["test_cylinder"] = math::Vector3(8, 0, z0);
  modelPos["test_empty"] = math::Vector3(12, 0, z0);

  // FIXME Trimesh drop test passes in bullet but fails in ode because
  // the mesh bounces to the side when it hits the ground.
  // See issue #513. Uncomment test when issue is resolved.
  // modelPos["test_trimesh"] = math::Vector3(16, 0, z0);

  SpawnBox("test_box", math::Vector3(1, 1, 1), modelPos["test_box"],
      math::Vector3::Zero);
  SpawnSphere("test_sphere", modelPos["test_sphere"], math::Vector3::Zero);
  SpawnCylinder("test_cylinder", modelPos["test_cylinder"],
      math::Vector3::Zero);
  //SpawnEmptyLink("test_empty", modelPos["test_empty"], math::Vector3::Zero);
  std::string trimeshPath =
      "file://media/models/cube_20k/meshes/cube_20k.stl";
  // SpawnTrimesh("test_trimesh", trimeshPath, math::Vector3(0.5, 0.5, 0.5),
  //    modelPos["test_trimesh"], math::Vector3::Zero);

  int steps = 2;
  physics::ModelPtr model;
  math::Pose pose1, pose2;
  math::Vector3 vel1, vel2;

  double t, x0 = 0;
  // This loop steps the world forward and makes sure that each model falls,
  // expecting downward z velocity and decreasing z position.
  for (std::map<std::string, math::Vector3>::iterator iter = modelPos.begin();
    iter != modelPos.end(); ++iter)
  {
    std::string name = iter->first;
    // Make sure the model is loaded
    model = world->GetModel(name);
    if (model != NULL)
    {
      gzdbg << "Check freefall of model " << name << '\n';
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
  double tHit = sqrt(2*(z0-0.5) / (-g.z));
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
      // Check that velocity is small
      vel1 = model->GetWorldLinearVel();
      t = world->GetSimTime().Double();
      EXPECT_NEAR(vel1.x, 0, PHYSICS_TOL);
      EXPECT_NEAR(vel1.y, 0, PHYSICS_TOL);
      if (name == "test_empty")
        EXPECT_NEAR(vel1.z, g.z*t, -g.z*t*PHYSICS_TOL);
      else
        EXPECT_NEAR(vel1.z, 0, PHYSICS_TOL);

      // Check that model is resting on ground
      pose1 = model->GetWorldPose();
      x0 = modelPos[name].x;
      EXPECT_NEAR(pose1.pos.x, x0, PHYSICS_TOL);
      EXPECT_NEAR(pose1.pos.y, 0, PHYSICS_TOL);
      if (name == "test_empty")
      {
        EXPECT_NEAR(pose1.pos.z, z0+g.z/2*t*t,
            fabs((z0+g.z/2*t*t)*PHYSICS_TOL));
      }
      else
        EXPECT_NEAR(pose1.pos.z, 0.5, PHYSICS_TOL);
    }
    else
    {
      gzerr << "Error loading model " << name << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }
}

//TEST_F(PhysicsTest, SpawnDropODE)
//{
//  SpawnDrop("ode");
//}

//#ifdef HAVE_BULLET
//TEST_F(PhysicsTest, SpawnDropBullet)
//{
//  SpawnDrop("bullet");
//}
//#endif  // HAVE_BULLET

#ifdef HAVE_RTQL8
TEST_F(PhysicsTest, SpawnDropRTQL8)
{
  SpawnDrop("rtql8");
}
#endif  // HAVE_RTQL8


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
