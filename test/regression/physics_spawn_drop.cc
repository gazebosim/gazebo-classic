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
#include "gazebo/physics/physics.hh"
#include "SimplePendulumIntegrator.hh"

#define HAVE_SIMBODY 1

#define PHYSICS_TOL 1e-2
using namespace gazebo;
class PhysicsTest : public ServerFixture
{
  public: void SpawnDrop(const std::string &_physicsEngine);
  public: void SpawnDropCoGOffset(const std::string &_physicsEngine);
};

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
  SpawnEmptyLink("test_empty", modelPos["test_empty"], math::Vector3::Zero);
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
/*
TEST_F(PhysicsTest, SpawnDropODE)
{
  SpawnDrop("ode");
}

#ifdef HAVE_BULLET
TEST_F(PhysicsTest, SpawnDropBullet)
{
  SpawnDrop("bullet");
}
#endif  // HAVE_BULLET

#ifdef HAVE_SIMBODY
TEST_F(PhysicsTest, SpawnDropSimbody)
{
  SpawnDrop("simbody");
}
#endif  // HAVE_SIMBODY
*/

////////////////////////////////////////////////////////////////////////
// SpawnDropCoGOffset:
// Load a world, check that gravity points along z axis, spawn several
// spheres of varying radii and center of gravity (cg) location.
//  sphere1: smaller radius, centered cg
//  sphere2: larger radius, centered cg
//  sphere3: larger radius, lowered cg
//  sphere4: larger radius, raised cg
//  sphere5: larger radius, y offset cg
//  sphere6: larger radius, x offset cg
//  sphere7: larger radius, 45 deg offset cg
//  sphere8: larger radius, -30 deg offset cg
// The bottom of each sphere is at the same height, and it is verified
// that they hit the ground at the same time. Also, sphere5 should start
// rolling to the side when it hits the ground.
////////////////////////////////////////////////////////////////////////
void PhysicsTest::SpawnDropCoGOffset(const std::string &_physicsEngine)
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
  EXPECT_LT(g.z, 0);

  // get physics time step
  double dt = physics->GetStepTime();
  EXPECT_GT(dt, 0);

  // spawn some spheres and check to see that they start falling
  double z0 = 3;
  double r1 = 0.5, r2 = 1.5;
  math::Vector3 v30 = math::Vector3::Zero;
  math::Vector3 cog;
  math::Angle angle;

  std::vector<std::string> modelNames;
  std::vector<double> x0s;
  std::vector<double> y0s;
  std::vector<double> radii;
  std::vector<math::Vector3> cogs;

  // sphere1 and sphere2 have c.g. at center of sphere, different sizes
  modelNames.push_back("small_centered_sphere");
  x0s.push_back(0);
  y0s.push_back(0);
  radii.push_back(r1);
  cogs.push_back(v30);

  modelNames.push_back("large_centered_sphere");
  x0s.push_back(4);
  y0s.push_back(0);
  radii.push_back(r2);
  cogs.push_back(v30);

  // sphere3 has c.g. below the center
  modelNames.push_back("lowered_cog_sphere");
  x0s.push_back(8);
  y0s.push_back(0);
  radii.push_back(r2);
  cogs.push_back(math::Vector3(0, 0, -r1));

  // sphere4 has c.g. above the center
  modelNames.push_back("raised_cog_sphere");
  x0s.push_back(-4);
  y0s.push_back(0);
  radii.push_back(r2);
  cogs.push_back(math::Vector3(0, 0, r1));

  // sphere5 has c.g. to the side along y axis; it will roll
  modelNames.push_back("cog_y_offset_sphere");
  x0s.push_back(-8);
  y0s.push_back(0);
  radii.push_back(r2);
  cogs.push_back(math::Vector3(0, r1, 0));

  // sphere6 has c.g. to the side along x axis; it will roll
  modelNames.push_back("cog_x_offset_sphere");
  x0s.push_back(15);
  y0s.push_back(0);
  radii.push_back(r2);
  cogs.push_back(math::Vector3(r1, 0, 0));

  // sphere7 has c.g. to the side diagonally; it will roll
  modelNames.push_back("cog_xy_45deg_offset_sphere");
  x0s.push_back(0);
  y0s.push_back(8);
  radii.push_back(r2);
  angle.SetFromDegree(45);
  cogs.push_back(math::Vector3(r1*cos(angle.Radian()),
                               r1*sin(angle.Radian()), 0));

  // sphere8 has c.g. to the side diagonally; it will roll
  modelNames.push_back("cog_xy_-30deg_offset_sphere");
  x0s.push_back(0);
  y0s.push_back(-8);
  radii.push_back(r2);
  angle.SetFromDegree(-30);
  cogs.push_back(math::Vector3(r1*cos(angle.Radian()),
                               r1*sin(angle.Radian()), 0));

  unsigned int i;
  for (i = 0; i < modelNames.size(); ++i)
  {
    SpawnSphere(modelNames[i], math::Vector3(x0s[i], y0s[i], z0+radii[i]),
                v30, cogs[i], radii[i]);
  }

  int steps = 2;
  physics::ModelPtr model;
  math::Pose pose1, pose2;
  math::Vector3 vel1, vel2;


  gzerr << "paused\n";
  getchar();

  double t, x0 = 0, y0 = 0, radius;
  // This loop steps the world forward and makes sure that each model falls,
  // expecting downward z velocity and decreasing z position.
  for (i = 0; i < modelNames.size(); ++i)
  {
    // Make sure the model is loaded
    model = world->GetModel(modelNames[i]);
    x0 = x0s[i];
    y0 = y0s[i];
    radius = radii[i];
    if (model != NULL)
    {
      gzdbg << "Check freefall of model " << modelNames[i] << '\n';
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
      EXPECT_NEAR(pose1.pos.x, x0, PHYSICS_TOL*PHYSICS_TOL);
      EXPECT_NEAR(pose1.pos.y, y0, PHYSICS_TOL*PHYSICS_TOL);
      EXPECT_NEAR(pose1.pos.z, z0+radius + g.z/2*t*t,
                  (z0+radius+g.z/2*t*t)*PHYSICS_TOL);

      // Check once more and just make sure they keep falling
      world->StepWorld(steps);
      vel2 = model->GetWorldLinearVel();
      pose2 = model->GetWorldPose();
      EXPECT_LT(vel2.z, vel1.z);
      EXPECT_LT(pose2.pos.z, pose1.pos.z);
    }
    else
    {
      gzerr << "Error loading model " << modelNames[i] << '\n';
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
  // after the time of predicted ground contact. Except for sphere5,
  // the velocity is expected to be small, and the pose is expected
  // to be underneath the initial pose. sphere5 is expected to be rolling.
  for (i = 0; i < modelNames.size(); ++i)
  {
    // Make sure the model is loaded
    model = world->GetModel(modelNames[i]);
    x0 = x0s[i];
    y0 = y0s[i];
    radius = radii[i];
    cog = cogs[i];
    if (model != NULL)
    {
      gzdbg << "Check ground contact and roll without slip of model "
            << modelNames[i] << '\n';

      // Check that velocity is small
      vel1 = model->GetWorldLinearVel();
      vel2 = model->GetWorldAngularVel();

      // vertical component of linear and angular velocity should be small
      EXPECT_NEAR(vel1.z, 0, PHYSICS_TOL);
      EXPECT_NEAR(vel2.z, 0, PHYSICS_TOL);

      // expect small values for directions with no offset
      if (cog.x == 0)
      {
        EXPECT_NEAR(vel1.x, 0, PHYSICS_TOL);
        EXPECT_NEAR(vel2.y, 0, PHYSICS_TOL);
      }
      // expect rolling in direction of cog offset
      else
      {
        EXPECT_GT(vel1.x*cog.x, 0.2*cog.x*cog.x);
        EXPECT_GT(vel2.y*cog.x, 0.2*cog.x*cog.x);
      }

      if (cog.y == 0)
      {
        EXPECT_NEAR(vel1.y, 0, PHYSICS_TOL);
        EXPECT_NEAR(vel2.x, 0, PHYSICS_TOL);
      }
      else
      {
        EXPECT_GT(vel1.y*cog.y,  0.2*cog.y*cog.y);
        EXPECT_LT(vel2.x*cog.y, -0.2*cog.y*cog.y);
      }

      // Expect roll without slip
      EXPECT_NEAR(vel1.x,  vel2.y*radius, PHYSICS_TOL);
      EXPECT_NEAR(vel1.y, -vel2.x*radius, PHYSICS_TOL);

      // Use GetWorldLinearVel with global offset to check roll without slip
      // Expect small linear velocity at contact point
      math::Vector3 vel3 = model->GetLink()->GetWorldLinearVel(
          math::Vector3(0, 0, -radius), math::Quaternion(0, 0, 0));
      EXPECT_NEAR(vel3.x, 0, PHYSICS_TOL);
      EXPECT_NEAR(vel3.y, 0, PHYSICS_TOL);
      EXPECT_NEAR(vel3.z, 0, PHYSICS_TOL);
      // Expect speed at top of sphere to be double the speed at center
      math::Vector3 vel4 = model->GetLink()->GetWorldLinearVel(
          math::Vector3(0, 0, radius), math::Quaternion(0, 0, 0));
      EXPECT_NEAR(vel4.y, 2*vel1.y, PHYSICS_TOL);
      EXPECT_NEAR(vel4.x, 2*vel1.x, PHYSICS_TOL);
      EXPECT_NEAR(vel4.z, 0, PHYSICS_TOL);

      // Check that model is resting on ground
      pose1 = model->GetWorldPose();
      EXPECT_NEAR(pose1.pos.z, radius, PHYSICS_TOL);

      // expect no pose change for directions with no offset
      if (cog.x == 0)
      {
        EXPECT_NEAR(pose1.pos.x, x0, PHYSICS_TOL);
      }
      // expect rolling in direction of cog offset
      else
      {
        EXPECT_GT((pose1.pos.x-x0) * cog.x, cog.x * cog.x);
      }

      // expect no pose change for directions with no offset
      if (cog.y == 0)
      {
        EXPECT_NEAR(pose1.pos.y, y0, PHYSICS_TOL);
      }
      // expect rolling in direction of cog offset
      else
      {
        EXPECT_GT((pose1.pos.y-y0) * cog.y, cog.y * cog.y);
      }
    }
    else
    {
      gzerr << "Error loading model " << modelNames[i] << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }

  gzerr << "done\n";
  getchar();

}
TEST_F(PhysicsTest, SpawnDropCoGOffsetODE)
{
  gzerr << "running ODE\n";
  SpawnDropCoGOffset("ode");
}

/*
#ifdef HAVE_BULLET
TEST_F(PhysicsTest, SpawnDropCoGOffsetBullet)
{
  SpawnDropCoGOffset("bullet");
}
#endif  // HAVE_BULLET
*/

#ifdef HAVE_SIMBODY
TEST_F(PhysicsTest, SpawnDropCoGOffsetSimbody)
{
  gzerr << "running simbody\n";
  SpawnDropCoGOffset("simbody");
}
#endif  // HAVE_SIMBODY


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
