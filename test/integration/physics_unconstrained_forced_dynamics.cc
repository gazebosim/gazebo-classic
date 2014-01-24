/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/msgs/msgs.hh"
#include "helper_physics_generator.hh"

#define PHYSICS_TOL 1e-2
using namespace gazebo;

class UnconstrainedForced : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
  public: void DampedLinear(const std::string &_physicsEngine);
  public: void DampedAngular(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////////////////
// Damped linear
////////////////////////////////////////////////////////////////////////
void UnconstrainedForced::DampedLinear(const std::string &_physicsEngine)
{
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // set time step size to 0.001
  double dt = 0.01;
  physics->SetMaxStepSize(dt);

  // turn off gravity
  physics->SetGravity(math::Vector3(0, 0, 0));

  // spawn a box in midair (h = 2)
  SpawnBox("rigid_body",
    math::Vector3(1, 1, 1),  // size
    math::Vector3(0, 0, 2),  // position
    math::Vector3(0, 0, 0),  // orientation
    false);  // not static

  // get handle to box model
  physics::ModelPtr model = world->GetModel("rigid_body");
  ASSERT_TRUE(model.get() != NULL);

  // get handle to canonical link
  physics::LinkPtr link =  model->GetLink();

  // set link mass
  double mass = 1.0;
  link->GetInertial()->SetMass(mass);

  // set link moi for box
  double ixx = 1.0 / 12.0 * mass * (2.0);
  link->GetInertial()->SetInertiaMatrix(ixx, ixx, ixx, 0, 0, 0);

  // declare damping coefficient
  double damping = 0.1;

  // declare initial velocity of box
  math::Vector3 v0(1.0, 0, 0);

  // set initial velocity of link
  link->SetLinearVel(v0);

  // get initial position of link
  math::Vector3 p0 = link->GetWorldPose().pos;

  // cache initial sim time
  double t0 = world->GetSimTime().Double();

  // get time step size
  // double dt = world->GetPhysicsEngine()->GetMaxStepSize();

  // run simulation for 1 second
  double testDuration = 1.0;  // 1 second

  // calculate number of steps needed
  unsigned int steps = testDuration / dt;

  // compute first order integrated velocity
  double v1 = v0.x;

  for (unsigned int step = 0; step < steps; ++step)
  {
    double t = world->GetSimTime().Double();

    // compute analytical velocity
    math::Vector3 va = v0 * exp(-damping / ixx * (t - t0));

    // compute analytical position by integrating velocity
    math::Vector3 pa = p0 - ixx * v0 / damping * (
      exp(-damping / ixx * t) -
      exp(-damping / ixx * t0));

    // get simulation position and velocity
    math::Vector3 p = link->GetWorldPose().pos;
    math::Vector3 v = link->GetWorldLinearVel();

    // compute damping force
    math::Vector3 dampingForce = -damping * v;

    // debug results
    gzdbg << "engine: [" << _physicsEngine
          << "] t: [" << t
          << "] v: [" << v
          << "] vdebug: [" << v1
          << "] va: [" << va
          << "] p: [" << p
          << "] pa: [" << pa
          << "] fd: [" << dampingForce
          << "]\n";

    // check results
    EXPECT_GT(dt, 0);


    // apply force to simulate damping force
    link->SetForce(dampingForce);

    // take a step
    world->Step(1);

    // compute first order integrated velocity
    v1 = v1 + dt * dampingForce.x / mass;
  }
}

TEST_P(UnconstrainedForced, DampedLinear)
{
  DampedLinear(GetParam());
}

////////////////////////////////////////////////////////////////////////
// torsional spring force check angular dynamics
////////////////////////////////////////////////////////////////////////
void UnconstrainedForced::DampedAngular(const std::string &_physicsEngine)
{
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // set time step size to 0.001
  double dt = 0.01;
  physics->SetMaxStepSize(dt);

  // turn off gravity
  physics->SetGravity(math::Vector3(0, 0, 0));

  // spawn a box in midair (h = 2)
  SpawnBox("rigid_body",
    math::Vector3(1, 1, 1),  // size
    math::Vector3(0, 0, 2),  // position
    math::Vector3(0, 0, 0),  // orientation
    false);  // not static

  // get handle to box model
  physics::ModelPtr model = world->GetModel("rigid_body");
  ASSERT_TRUE(model.get() != NULL);

  // get handle to canonical link
  physics::LinkPtr link =  model->GetLink();

  // set link mass
  double mass = 1.0;
  link->GetInertial()->SetMass(mass);

  // set link moi for box
  double ixx = 1.0 / 12.0 * mass * (2.0);
  link->GetInertial()->SetInertiaMatrix(ixx, ixx, ixx, 0, 0, 0);

  // declare damping coefficient
  double damping = 0.1;

  // declare initial angular velocity of box
  math::Vector3 v0(1.0, 0, 0);

  // set initial velocity of link
  link->SetAngularVel(v0);

  // get initial position of link
  math::Vector3 r0 = link->GetWorldPose().rot.GetAsEuler();

  // cache initial sim time
  double t0 = world->GetSimTime().Double();

  // get time step size
  // double dt = world->GetPhysicsEngine()->GetMaxStepSize();

  // run simulation for 1 second
  double testDuration = 1.0;  // 1 second

  // calculate number of steps needed
  unsigned int steps = testDuration / dt;

  // compute first order integrated velocity
  double v1 = v0.x;

  for (unsigned int step = 0; step < steps; ++step)
  {
    double t = world->GetSimTime().Double();

    // compute analytical velocity
    math::Vector3 va = v0 * exp(-damping / mass * (t - t0));

    // compute analytical position by integrating velocity
    math::Vector3 ra = r0 - mass * v0 / damping * (
      exp(-damping / mass * t) -
      exp(-damping / mass * t0));

    // get simulation position and velocity
    math::Vector3 r = link->GetWorldPose().rot.GetAsEuler();
    math::Vector3 v = link->GetWorldAngularVel();

    // compute damping force
    math::Vector3 dampingForce = -damping * v;

    // debug results
    gzdbg << "engine: [" << _physicsEngine
          << "] t: [" << t
          << "] v: [" << v
          << "] vdebug: [" << v1
          << "] va: [" << va
          << "] r: [" << r
          << "] ra: [" << ra
          << "] fd: [" << dampingForce
          << "]\n";

    // check results
    EXPECT_GT(dt, 0);


    // apply force to simulate damping force
    link->SetTorque(dampingForce);

    // take a step
    world->Step(1);

    // compute first order integrated velocity
    v1 = v1 + dt * dampingForce.x / mass;
  }
}

TEST_P(UnconstrainedForced, DampedAngular)
{
  DampedAngular(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, UnconstrainedForced,
  PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
