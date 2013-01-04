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

  double t, dt;

  // simulate one step
  world->StepWorld(1);
  t = world->GetSimTime().Double();
  dt = world->GetPhysicsEngine()->GetStepTime();
  EXPECT_GT(t, 0.99*dt);

  // simulate several steps
  int steps = 20;
  world->StepWorld(steps-1);
  t = world->GetSimTime().Double();
  dt = world->GetPhysicsEngine()->GetStepTime();
  EXPECT_GT(t, 0.99*dt*static_cast<double>(steps));
  #ifdef HAVE_BULLET
  gzerr << "HAVE_BULLET\n";
  #endif
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
 
  int steps = 2;
  physics::ModelPtr model;
  math::Pose pose1, pose2;
  math::Vector3 vel1, vel2;

  for (std::list<std::string>::iterator iter = model_names.begin();
    iter != model_names.end(); ++iter)
  {
    // Make sure the model is loaded
    model = world->GetModel(*iter);
    ASSERT_TRUE(model);

    // Step forward and check downward z velocity and decreasing z position
    world->StepWorld(steps);
    vel1 = model->GetWorldLinearVel();
    pose1 = model->GetWorldPose();
    EXPECT_LT(vel1.z, -1e-16);
    EXPECT_LT(pose1.pos.z, z0);
    // Also check that x and y velocities are zero
    EXPECT_EQ(vel1.x, 0);
    EXPECT_EQ(vel1.y, 0);

    world->StepWorld(steps);
    vel2 = model->GetWorldLinearVel();
    pose2 = model->GetWorldPose();
    EXPECT_LT(vel2.z, vel1.z);
    EXPECT_LT(pose2.pos.z, pose1.pos.z);
    EXPECT_EQ(vel1.x, 0);
    EXPECT_EQ(vel1.y, 0);
  }

  // wait until they all hit the ground plane, and make sure they've stopped
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  math::Vector3 g = physics->GetGravity();
  double dt = world->GetPhysicsEngine()->GetStepTime();
  // Compute time tHit when shapes are predicted to hit ground plane.
  // All shapes have bottom surface 0.5 m below the center.
  ASSERT_LT(g.z, 0);
  double tHit = sqrt(2*(z0-0.5) / (-g.z));
  // Time to advance
  double dtHit = tHit - world->GetSimTime().Double();
  // Step forward 10% farther than we expect
  int stepsToHit = ceil(dtHit*1.1 / dt);
  ASSERT_GT(stepsToHit, 0);
  world->StepWorld(stepsToHit);

  double velFreeFall = tHit * g.z;
  for (std::list<std::string>::iterator iter = model_names.begin();
    iter != model_names.end(); ++iter)
  {
    // Make sure the model is loaded
    model = world->GetModel(*iter);
    ASSERT_TRUE(model);

    vel1 = model->GetWorldLinearVel();
    pose1 = model->GetWorldPose();
    gzdbg << *iter << " pose " << pose1 << '\n';
    gzdbg << *iter << " vel  " << vel1 << '\n';

    // Expect model velocity to be small
    EXPECT_LT(fabs(vel1.x), 1e-2);
    EXPECT_LT(fabs(vel1.y), 1e-2);
    EXPECT_LT(fabs(vel1.z), 1e-2);

    // Expect bottom of shape to not be near surface
    EXPECT_LT(pose1.pos.z-0.5,  0.05);
    EXPECT_GT(pose1.pos.z-0.5, -0.05);
  }
}

TEST_F(BulletTest, SpawnResizeTest)
{
  // load an empty world with bullet physics engine
  Load("worlds/empty_bullet.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // spawn some simple shapes at ground level
  double z0 = 0.5;
  SpawnBox("test_box", math::Vector3(1, 1, 1), math::Vector3(0, 0, z0),
    math::Vector3::Zero);
  SpawnSphere("test_sphere", math::Vector3(4, 0, z0), math::Vector3::Zero);
  SpawnCylinder("test_cylinder", math::Vector3(8, 0, z0), math::Vector3::Zero);

  std::list<std::string> model_names;
  model_names.push_back("test_box");
  model_names.push_back("test_sphere");
  model_names.push_back("test_cylinder");

  // Step forward and allow objects to come to rest
  world->StepWorld(100);

  // The following test doesn't yet pass for bullet

  // // Now test the resize functions for simple shapes.
  // physics::LinkPtr link;
  // physics::CollisionPtr collision;
  // physics::ShapePtr shape;
  // for (std::list<std::string>::iterator iter = model_names.begin();
  //   iter != model_names.end(); ++iter)
  // {
  //   // Make sure the model is loaded
  //   model = world->GetModel(*iter);
  //   ASSERT_TRUE(model);
  //   link = model->GetLink("canonical");
  //   ASSERT_TRUE(link);
  //   unsigned int id=0;
  //   collision = link->GetCollision(id);
  //   ASSERT_TRUE(collision);
  //   shape = collision->GetShape();
  //   ASSERT_TRUE(shape);
  //   if (shape->HasType(physics::Base::BOX_SHAPE))
  //   {
  //     gzdbg << "Changing box size\n";
  //     physics::BoxShapePtr box;
  //     box = boost::shared_dynamic_cast<physics::BoxShape>(shape);
  //     box->SetSize(math::Vector3(.5, .5, .5));
  //   }
  //   else if (shape->HasType(physics::Base::SPHERE_SHAPE))
  //   {
  //     gzdbg << "Changing sphere size\n";
  //     physics::SphereShapePtr sphere;
  //     sphere = boost::shared_dynamic_cast<physics::SphereShape>(shape);
  //     sphere->SetRadius(0.25);
  //   }
  //   else if (shape->HasType(physics::Base::CYLINDER_SHAPE))
  //   {
  //     gzdbg << "Changing cylinder size\n";
  //     physics::CylinderShapePtr cylinder;
  //     cylinder = boost::shared_dynamic_cast<physics::CylinderShape>(shape);
  //     cylinder->SetSize(0.25, 0.5);
  //   }
  //   else
  //   {
  //     // We shouldn't ever get here.
  //     ASSERT_EQ(0, 1);
  //   }
  // }

  // // Each shape is now smaller. They should fall again.
  // world->StepWorld(2);
  // for (std::list<std::string>::iterator iter = model_names.begin();
  //   iter != model_names.end(); ++iter)
  // {
  //   // Make sure the model is loaded
  //   model = world->GetModel(*iter);
  //   ASSERT_TRUE(model);

  //   vel1 = model->GetWorldLinearVel();
  //   // z velocity should be negative
  //   EXPECT_LT(vel1.z, -1e-10);
  //   // z velocity should be about g*2*dt
  //   EXPECT_LT(fabs(vel1.z - 2*g.z*dt), 1e-2);
  // }

  // // Expect them to stop again when hitting the ground.
  // dtHit = sqrt(2*0.25 / (-g.z)) - 2*dt;
  // stepsToHit = ceil(dtHit*1.1 / dt);
  // ASSERT_GT(stepsToHit, 0);
  // world->StepWorld(stepsToHit);

  // for (std::list<std::string>::iterator iter = model_names.begin();
  //   iter != model_names.end(); ++iter)
  // {
  //   // Make sure the model is loaded
  //   model = world->GetModel(*iter);
  //   ASSERT_TRUE(model);

  //   vel1 = model->GetWorldLinearVel();
  //   pose1 = model->GetWorldPose();

  //   // Expect model velocity to be near zero
  //   EXPECT_LT(fabs(vel1.z), 1e-2);

  //   // Expect center of shape to have dropped
  //   EXPECT_LT(pose1.pos.z, 0.4);

  //   // Expect bottom of shape to not be overly penetrated
  //   EXPECT_GT(pose1.pos.z-0.25, -0.025);
  // }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
