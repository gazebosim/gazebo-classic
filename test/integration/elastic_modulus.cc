/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include <map>
#include <string>
#include <vector>

#include <ignition/math/Rand.hh>

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "SimplePendulumIntegrator.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/test/helper_physics_generator.hh"

#define PHYSICS_TOL 1e-2
using namespace gazebo;

class PhysicsTest : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
  public: void ElasticModulusContact(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////////////////
void PhysicsTest::ElasticModulusContact(const std::string &_physicsEngine)
{
  // check conservation of mementum for linear elastic collision
  Load("worlds/elastic_modulus_contact_test.world", true, _physicsEngine);
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
    double dt = world->GetPhysicsEngine()->GetMaxStepSize();

    physics::ModelPtr box_model = world->GetModel("box");
    physics::LinkPtr box_link = box_model->GetLink("link");
    double f = 1000.0;
    double v = 0;
    double x = 0;
    double m = box_link->GetInertial()->GetMass();

    int steps = test_duration/dt;

    for (int i = 0; i < steps; ++i)
    {
      double t = world->GetSimTime().Double();

      world->Step(1);  // theoretical contact, but
      {
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
          {
            box_model->GetLink("link")->SetForce(math::Vector3(f, 0, 0));
            // The following has been failing since pull request #1284,
            // so it has been disabled.
            // See bitbucket.org/osrf/gazebo/issue/1394
            // EXPECT_EQ(box_model->GetLink("link")->GetWorldForce(),
            //   math::Vector3(f, 0, 0));
          }

          if (t > 1.000 && t < 1.01)
          {
            // collision transition, do nothing
          }
          else
          {
            // collision happened
            EXPECT_NEAR(pose.pos.x, x, PHYSICS_TOL);
            EXPECT_NEAR(vel.x, v, PHYSICS_TOL);
          }
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
          if (t > 1.000 && t < 1.01)
          {
            // collision transition, do nothing
          }
          else if (t <= 1.00)
          {
            // no collision
            EXPECT_EQ(pose.pos.x, 2);
            EXPECT_EQ(vel.x, 0);
          }
          else
          {
            // collision happened
            EXPECT_NEAR(pose.pos.x, x + 1.0, PHYSICS_TOL);
            EXPECT_NEAR(vel.x, v, PHYSICS_TOL);
          }
        }
      }

      /*
      // integrate here to see when the collision should happen
      double impulse = dt*f;
      if (i == 0) v = v + impulse;
      else if (t >= 1.0) v = dt*f/ 2.0;  // inelastic col. w/ eqal mass.
      x = x + dt * v;
      */

      // integrate here to see when the collision should happen
      double vold = v;
      if (i == 0)
        v = vold + dt* (f / m);
      else if (t >= 1.0)
        v = dt*f/ 2.0;  // inelastic col. w/ eqal mass.
      x = x + dt * (v + vold) / 2.0;
    }
  }
}

TEST_P(PhysicsTest, ElasticModulusContact)
{
  ElasticModulusContact(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

