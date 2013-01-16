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
  public: void EmptyWorld(const std::string &_worldFile);
  public: void SpawnDrop(const std::string &_worldFile);
};

////////////////////////////////////////////////////////////////////////
// EmptyWorld:
// Load a world, take a few steps, and verify that time is increasing.
// This is the most basic physics engine test.
////////////////////////////////////////////////////////////////////////
void PhysicsTest::EmptyWorld(const std::string &_worldFile)
{
  // Load an empty world
  Load(_worldFile, true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

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

TEST_F(PhysicsTest, EmptyWorldODE)
{
  EmptyWorld("worlds/empty.world");
}

#ifdef HAVE_BULLET
TEST_F(PhysicsTest, EmptyWorldBullet)
{
  EmptyWorld("worlds/empty_bullet.world");
}
#endif  // HAVE_BULLET

////////////////////////////////////////////////////////////////////////
// SpawnDrop:
// Load a world, check that gravity points along z axis, spawn simple
// shapes (box, sphere, cylinder), verify that they fall and hit the
// ground plane. The test currently assumes inelastic collisions.
////////////////////////////////////////////////////////////////////////
void PhysicsTest::SpawnDrop(const std::string &_worldFile)
{
  // load an empty world
  Load(_worldFile, true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
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
  SpawnBox("test_box", math::Vector3(1, 1, 1), math::Vector3(0, 0, z0),
    math::Vector3::Zero);
  SpawnSphere("test_sphere", math::Vector3(4, 0, z0), math::Vector3::Zero);
  SpawnCylinder("test_cylinder", math::Vector3(8, 0, z0), math::Vector3::Zero);

  std::list<std::string> model_names;
  model_names.push_back("test_box");
  model_names.push_back("test_sphere");
  model_names.push_back("test_cylinder");

  int steps = 2;
  physics::ModelPtr model;
  math::Pose pose1, pose2;
  math::Vector3 vel1, vel2;

  double t, x0 = 0;
  // This loop steps the world forward and makes sure that each model falls,
  // expecting downward z velocity and decreasing z position.
  for (std::list<std::string>::iterator iter = model_names.begin();
    iter != model_names.end(); ++iter)
  {
    // Make sure the model is loaded
    model = world->GetModel(*iter);
    if (model != NULL)
    {
      gzdbg << "Check freefall of model " << *iter << '\n';
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
      if (*iter == "test_box")
        x0 = 0;
      else if (*iter == "test_sphere")
        x0 = 4;
      else if (*iter == "test_cylinder")
        x0 = 8;
      EXPECT_EQ(pose1.pos.x, x0);
      EXPECT_EQ(pose1.pos.y, 0);
      EXPECT_NEAR(pose1.pos.z, z0 - g.z/2*t*t, (z0-g.z/2*t*t)*PHYSICS_TOL);

      // Check once more and just make sure they keep falling
      world->StepWorld(steps);
      vel2 = model->GetWorldLinearVel();
      pose2 = model->GetWorldPose();
      EXPECT_LT(vel2.z, vel1.z);
      EXPECT_LT(pose2.pos.z, pose1.pos.z);
    }
    else
    {
      gzerr << "Error loading model " << *iter << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }

  // Wait until they've all hit the ground plane
  double tHit = sqrt(2*(z0-0.5) / (-g.z));
  // Time to advance, allow 0.5 s settling time.
  double dtHit = tHit+0.5 - world->GetSimTime().Double();
  steps = ceil(dtHit / dt);
  EXPECT_GT(steps, 0);
  world->StepWorld(steps);

  // This loop checks the velocity and pose of each model 0.5 seconds
  // after the time of predicted ground contact. The velocity is expected
  // to be small, and the pose is expected to be underneath the initial pose.
  for (std::list<std::string>::iterator iter = model_names.begin();
    iter != model_names.end(); ++iter)
  {
    // Make sure the model is loaded
    model = world->GetModel(*iter);
    if (model != NULL)
    {
      gzdbg << "Check ground contact of model " << *iter << '\n';
      // Check that velocity is small
      vel1 = model->GetWorldLinearVel();
      EXPECT_NEAR(vel1.x, 0, PHYSICS_TOL);
      EXPECT_NEAR(vel1.y, 0, PHYSICS_TOL);
      EXPECT_NEAR(vel1.z, 0, PHYSICS_TOL);

      // Check that model is resting on ground
      pose1 = model->GetWorldPose();
      if (*iter == "test_box")
        x0 = 0;
      else if (*iter == "test_sphere")
        x0 = 4;
      else if (*iter == "test_cylinder")
        x0 = 8;
      EXPECT_NEAR(pose1.pos.x, x0, PHYSICS_TOL);
      EXPECT_NEAR(pose1.pos.y, 0, PHYSICS_TOL);
      EXPECT_NEAR(pose1.pos.z, 0.5, PHYSICS_TOL);
    }
    else
    {
      gzerr << "Error loading model " << *iter << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }
}

TEST_F(PhysicsTest, SpawnDropODE)
{
  SpawnDrop("worlds/empty.world");
}

#ifdef HAVE_BULLET
TEST_F(PhysicsTest, SpawnDropBullet)
{
  SpawnDrop("worlds/empty_bullet.world");
}
#endif  // HAVE_BULLET

TEST_F(PhysicsTest, State)
{
  /// \TODO: Redo state test
  /*
  Load("worlds/empty.world");
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  physics::WorldState worldState = world->GetState();
  physics::ModelState modelState = worldState.GetModelState(0);
  physics::LinkState linkState = modelState.GetLinkState(0);
  physics::CollisionState collisionState = linkState.GetCollisionState(0);

  math::Pose pose;
  EXPECT_EQ(static_cast<unsigned int>(1), worldState.GetModelStateCount());
  EXPECT_EQ(static_cast<unsigned int>(1), modelState.GetLinkStateCount());
  EXPECT_EQ(static_cast<unsigned int>(1), linkState.GetCollisionStateCount());
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
  world->GetModel("box")->SetWorldPose(
      math::Pose(math::Vector3(1, 2, 0.5), math::Quaternion(0, 0, 0)));

  gazebo::common::Time::MSleep(10);

  // Make sure the box has been moved
  physics::ModelState modelState2 = world->GetState().GetModelState("box");
  pose.Set(math::Vector3(1, 2, 0.5), math::Quaternion(0, 0, 0));
  EXPECT_TRUE(pose == modelState2.GetPose());

  // Reset world state, and check for correctness
  world->SetState(worldState);
  modelState2 = world->GetState().GetModelState("box");
  pose.Set(math::Vector3(0, 0, 0.5), math::Quaternion(0, 0, 0));
  EXPECT_TRUE(pose == modelState2.GetPose());
  Unload();
  */
}

TEST_F(PhysicsTest, JointDampingTest)
{
  Load("worlds/damp_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  int i = 0;
  while (!this->HasEntity("model_4_mass_1_ixx_1_damping_10") && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }

  if (i > 20)
    gzthrow("Unable to get model_4_mass_1_ixx_1_damping_10");

  physics::ModelPtr model = world->GetModel("model_4_mass_1_ixx_1_damping_10");
  EXPECT_TRUE(model != NULL);

  {
    // compare against recorded data only
    double test_duration = 1.5;
    double dt = world->GetPhysicsEngine()->GetStepTime();
    int steps = test_duration/dt;

    for (int i = 0; i < steps; i++)
    {
      world->StepWorld(1);  // theoretical contact, but
      // gzdbg << "box time [" << world->GetSimTime().Double()
      //       << "] vel [" << model->GetWorldLinearVel()
      //       << "] pose [" << model->GetWorldPose()
      //       << "]\n";
    }

    EXPECT_EQ(world->GetSimTime().Double(), 1.5);

    math::Vector3 vel = model->GetWorldLinearVel();
    math::Pose pose = model->GetWorldPose();

    EXPECT_EQ(vel.x, 0.0);

    EXPECT_LT(vel.y, -10.2006);
    EXPECT_GT(vel.y, -10.2008);
    EXPECT_LT(vel.z, -6.51766);
    EXPECT_GT(vel.z, -6.51768);

    EXPECT_EQ(pose.pos.x, 3.0);
    EXPECT_LT(pose.pos.y, 5.0e-6);
    EXPECT_GT(pose.pos.y, 0.0);
    EXPECT_GT(pose.pos.z, 10.099);
    EXPECT_LT(pose.pos.z, 10.101);
    EXPECT_GT(pose.rot.GetAsEuler().x, 0.567336);
    EXPECT_LT(pose.rot.GetAsEuler().x, 0.567338);
    EXPECT_EQ(pose.rot.GetAsEuler().y, 0.0);
    EXPECT_EQ(pose.rot.GetAsEuler().z, 0.0);
  }
}

TEST_F(PhysicsTest, DropStuff)
{
  Load("worlds/drop_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  int i = 0;
  while (!this->HasEntity("cylinder") && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }

  if (i > 20)
    gzthrow("Unable to get cylinder");

  {
    // todo: get parameters from drop_test.world
    double test_duration = 1.5;
    double z = 10.5;
    double v = 0.0;
    double g = -10.0;
    double dt = world->GetPhysicsEngine()->GetStepTime();

    // world->StepWorld(1428);  // theoretical contact, but
    // world->StepWorld(100);  // integration error requires few more steps

    int steps = test_duration/dt;
    bool post_contact_correction = false;

    for (int i = 0; i < steps; i++)
    {
      // integrate here to see when the collision should happen
      v = v + dt * g;
      z = z + dt * v;

      world->StepWorld(1);  // theoretical contact, but
      {
        physics::ModelPtr box_model = world->GetModel("box");
        if (box_model)
        {
          math::Vector3 vel = box_model->GetWorldLinearVel();
          math::Pose pose = box_model->GetWorldPose();
          // gzdbg << "box time [" << world->GetSimTime().Double()
          //      << "] sim z [" << pose.pos.z
          //      << "] exact z [" << z
          //      << "] sim vz [" << vel.z
          //      << "] exact vz [" << v << "]\n";
          if (z > 0.5 || !post_contact_correction)
          {
            EXPECT_LT(fabs(vel.z - v) , 0.0001);
            EXPECT_LT(fabs(pose.pos.z - z) , 0.0001);
          }
          else
          {
            EXPECT_LT(fabs(vel.z), 0.0101);  // sometimes -0.01, why?
            EXPECT_LT(fabs(pose.pos.z - 0.5), 0.00001);
          }
        }

        physics::ModelPtr sphere_model = world->GetModel("sphere");
        if (sphere_model)
        {
          math::Vector3 vel = sphere_model->GetWorldLinearVel();
          math::Pose pose = sphere_model->GetWorldPose();
          // gzdbg << "sphere time [" << world->GetSimTime().Double()
          //       << "] sim z [" << pose.pos.z
          //       << "] exact z [" << z
          //       << "] sim vz [" << vel.z
          //       << "] exact vz [" << v << "]\n";
          if (z > 0.5 || !post_contact_correction)
          {
            EXPECT_LT(fabs(vel.z - v), 0.0001);
            EXPECT_LT(fabs(pose.pos.z - z), 0.0001);
          }
          else
          {
            EXPECT_LT(fabs(vel.z), 3e-5);
            EXPECT_LT(fabs(pose.pos.z - 0.5), 0.00001);
          }
        }

        physics::ModelPtr cylinder_model = world->GetModel("cylinder");
        if (cylinder_model)
        {
          math::Vector3 vel = cylinder_model->GetWorldLinearVel();
          math::Pose pose = cylinder_model->GetWorldPose();
          // gzdbg << "cylinder time [" << world->GetSimTime().Double()
          //       << "] sim z [" << pose.pos.z
          //       << "] exact z [" << z
          //       << "] sim vz [" << vel.z
          //       << "] exact vz [" << v << "]\n";
          if (z > 0.5 || !post_contact_correction)
          {
            EXPECT_LT(fabs(vel.z - v), 0.0001);
            EXPECT_LT(fabs(pose.pos.z - z), 0.0001);
          }
          else
          {
            EXPECT_LT(fabs(vel.z), 0.011);
            EXPECT_LT(fabs(pose.pos.z - 0.5), 0.0001);
          }
        }
      }
      if (z < 0.5) post_contact_correction = true;
    }
  }
}


TEST_F(PhysicsTest, CollisionTest)
{
  // check conservation of mementum for linear inelastic collision
  Load("worlds/collision_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  int i = 0;
  while (!this->HasEntity("sphere") && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }

  if (i > 20)
    gzthrow("Unable to get sphere");

  {
    // todo: get parameters from drop_test.world
    double test_duration = 1.1;
    double dt = world->GetPhysicsEngine()->GetStepTime();

    double f = 1000.0;
    double v = 0;
    double x = 0;

    int steps = test_duration/dt;

    for (int i = 0; i < steps; i++)
    {
      double t = world->GetSimTime().Double();

      world->StepWorld(1);  // theoretical contact, but
      {
        physics::ModelPtr box_model = world->GetModel("box");
        if (box_model)
        {
          math::Vector3 vel = box_model->GetWorldLinearVel();
          math::Pose pose = box_model->GetWorldPose();
          // gzdbg << "box time [" << t
          //      << "] sim x [" << pose.pos.x
          //      << "] ideal x [" << x
          //      << "] sim vx [" << vel.x
          //      << "] ideal vx [" << v
          //      << "]\n";

          if (i == 0)
            box_model->GetLink("link")->SetForce(math::Vector3(1000, 0, 0));
          EXPECT_LT(fabs(pose.pos.x - x), 0.00001);
          EXPECT_LT(fabs(vel.x - v), 0.00001);
        }

        physics::ModelPtr sphere_model = world->GetModel("sphere");
        if (sphere_model)
        {
          math::Vector3 vel = sphere_model->GetWorldLinearVel();
          math::Pose pose = sphere_model->GetWorldPose();
          // gzdbg << "sphere time [" << world->GetSimTime().Double()
          //      << "] sim x [" << pose.pos.x
          //      << "] ideal x [" << x
          //      << "] sim vx [" << vel.x
          //      << "] ideal vx [" << v
          //      << "]\n";
          if (t < 1.001)
          {
            EXPECT_EQ(pose.pos.x, 2);
            EXPECT_EQ(vel.x, 0);
          }
          else
          {
            EXPECT_LT(fabs(pose.pos.x - x - 1.0), 0.00001);
            EXPECT_LT(fabs(vel.x - v), 0.00001);
          }
        }
      }

      // integrate here to see when the collision should happen
      double impulse = dt*f;
      if (i == 0) v = v + impulse;
      else if (t >= 1.0) v = dt*f/ 2.0;  // inelastic col. w/ eqal mass.
      x = x + dt * v;
    }
  }
}


TEST_F(PhysicsTest, SimplePendulumTest)
{
  Load("worlds/simple_pendulums.world", true);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  int i = 0;
  while (!this->HasEntity("model_1") && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }

  if (i > 20)
    gzthrow("Unable to get model_1");

  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();
  EXPECT_TRUE(physicsEngine);
  physics::ModelPtr model = world->GetModel("model_1");
  EXPECT_TRUE(model);
  physics::LinkPtr link = model->GetLink("link_2");  // sphere link at end
  EXPECT_TRUE(link);

  double g = 9.81;
  double l = 10.0;
  double m = 10.0;

  double e_start;

  {
    // check velocity / energy
    math::Vector3 vel = link->GetWorldLinearVel();
    math::Pose pos = link->GetWorldPose();
    double pe = 9.81 * m * pos.pos.z;
    double ke = 0.5 * m * (vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
    e_start = pe + ke;
    // gzdbg << "total energy [" << e_start
    //       << "] pe[" << pe
    //       << "] ke[" << ke
    //       << "] p[" << pos.pos.z
    //       << "] v[" << vel
    //       << "]\n";
  }
  physicsEngine->SetStepTime(0.0001);
  physicsEngine->SetSORPGSIters(1000);

  {
    // test with global contact_max_correcting_vel at 0 as set by world file
    //   here we expect significant energy loss as the velocity correction
    //   is set to 0
    int steps = 10;  // @todo: make this more general
    for (int i = 0; i < steps; i ++)
    {
      world->StepWorld(2000);
      {
        // check velocity / energy
        math::Vector3 vel = link->GetWorldLinearVel();
        math::Pose pos = link->GetWorldPose();
        double pe = 9.81 * m * pos.pos.z;
        double ke = 0.5 * m * (vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
        double e = pe + ke;
        double e_tol = 3.0*static_cast<double>(i+1)
          / static_cast<double>(steps);
        // gzdbg << "total energy [" << e
        //       << "] pe[" << pe
        //       << "] ke[" << ke
        //       << "] p[" << pos.pos.z
        //       << "] v[" << vel
        //       << "] error[" << e - e_start
        //       << "] tol[" << e_tol
        //       << "]\n";

        EXPECT_LT(fabs(e - e_start), e_tol);
      }

      physics::JointPtr joint = model->GetJoint("joint_0");
      if (joint)
      {
        double integ_theta = (
          PendulumAngle(g, l, 1.57079633, 0.0, world->GetSimTime().Double(),
          0.000001) - 1.5707963);
        double actual_theta = joint->GetAngle(0).Radian();
        // gzdbg << "time [" << world->GetSimTime().Double()
        //       << "] exact [" << integ_theta
        //       << "] actual [" << actual_theta
        //       << "] pose [" << model->GetWorldPose()
        //       << "]\n";
         EXPECT_LT(fabs(integ_theta - actual_theta) , 0.01);
      }
    }
  }



  {
    // test with global contact_max_correcting_vel at 100
    // here we expect much lower energy loss
    world->Reset();
    physicsEngine->SetContactMaxCorrectingVel(100);

    int steps = 10;  // @todo: make this more general
    for (int i = 0; i < steps; i ++)
    {
      world->StepWorld(2000);
      {
        // check velocity / energy
        math::Vector3 vel = link->GetWorldLinearVel();
        math::Pose pos = link->GetWorldPose();
        double pe = 9.81 * m * pos.pos.z;
        double ke = 0.5 * m * (vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
        double e = pe + ke;
        double e_tol = 3.0*static_cast<double>(i+1)
          / static_cast<double>(steps);
        // gzdbg << "total energy [" << e
        //       << "] pe[" << pe
        //       << "] ke[" << ke
        //       << "] p[" << pos.pos.z
        //       << "] v[" << vel
        //       << "] error[" << e - e_start
        //       << "] tol[" << e_tol
        //       << "]\n";

        EXPECT_LT(fabs(e - e_start), e_tol);
      }

      physics::JointPtr joint = model->GetJoint("joint_0");
      if (joint)
      {
        double integ_theta = (
          PendulumAngle(g, l, 1.57079633, 0.0, world->GetSimTime().Double(),
          0.000001) - 1.5707963);
        double actual_theta = joint->GetAngle(0).Radian();
        // gzdbg << "time [" << world->GetSimTime().Double()
        //       << "] exact [" << integ_theta
        //       << "] actual [" << actual_theta
        //       << "] pose [" << model->GetWorldPose()
        //       << "]\n";
         EXPECT_LT(fabs(integ_theta - actual_theta) , 0.01);
      }
    }
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
