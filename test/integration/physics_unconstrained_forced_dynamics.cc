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
  public: void LinearDamper(const std::string &_physicsEngine);
  public: void LinearSpring(const std::string &_physicsEngine);
  public: void LinearSpringDamper(const std::string &_physicsEngine);
  public: void AngularDamper(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////////////////
// Damper linear
////////////////////////////////////////////////////////////////////////
void UnconstrainedForced::LinearDamper(const std::string &_physicsEngine)
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
  double dt = 0.001;
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

  std::cout << "engine t p.x p.y p.z "
        << "p_analytical.x p_analytical.y p_analytical.z "
        << "v.x v.y v.z v_debug "
        << "v_analytical.x v_analytical.y v_analytical.z "
        << "f.x f.y f.z\n";

  for (unsigned int step = 0; step < steps; ++step)
  {
    double t = world->GetSimTime().Double();

    // compute analytical velocity
    math::Vector3 va = v0 * exp(-damping / mass * (t - t0));

    // compute analytical position by integrating velocity
    math::Vector3 pa = p0 - mass * v0 / damping * (
      exp(-damping / mass * t) -
      exp(-damping / mass * t0));

    // get simulation position and velocity
    math::Vector3 p = link->GetWorldPose().pos;
    math::Vector3 v = link->GetWorldLinearVel();

    // compute damping force
    math::Vector3 damperForce = -damping * v;

    // debug results
    std::cout <<  _physicsEngine << " " << t
          << " " << p << " " << pa
          << " " << v << " " << v1 << " " << va
          << " " << damperForce << "\n";

    // check results
    EXPECT_GT(dt, 0);


    // apply force to simulate damping force
    link->SetForce(damperForce);

    // take a step
    world->Step(1);

    // compute first order integrated velocity
    v1 = v1 + dt * damperForce.x / mass;
  }
}

TEST_P(UnconstrainedForced, LinearDamper)
{
  LinearDamper(GetParam());
}

////////////////////////////////////////////////////////////////////////
// Linear Spring Damper
////////////////////////////////////////////////////////////////////////
void UnconstrainedForced::LinearSpringDamper(const std::string &_physicsEngine)
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

  // declare stiffness coefficient
  double stiffness = 10.0;

  // declare initial position of box
  math::Vector3 p0(1.0, 0, 2.0);

  // declare spring reference position
  math::Vector3 pr(0.0, 0, 2.0);

  // set initial pose of link
  link->SetWorldPose(math::Pose(p0, math::Quaternion()));

  // get initial position of link
  math::Vector3 v0 = link->GetWorldLinearVel();

  // cache initial sim time
  double t0 = world->GetSimTime().Double();

  // get time step size
  // double dt = world->GetPhysicsEngine()->GetMaxStepSize();

  // run simulation for 1 second
  double testDuration = 1.0;  // 1 second

  // calculate number of steps needed
  unsigned int steps = testDuration / dt;

  // compute first order integrated velocity with v1
  double v1 = v0.x;

  // compute analytical natural frequency
  double wn = sqrt(stiffness / mass);

  // compute analytical xi (critical damping ratio)
  double xi = damping / (mass * 2.0 * wn);

  // over/under damped
  bool underdamped = (xi < 1.0) ? true : false;

  std::cout << "engine t p.x p.y p.z "
        << "p_analytical.x p_analytical.y p_analytical.z "
        << "v.x v.y v.z v_debug "
        << "v_analytical.x v_analytical.y v_analytical.z "
        << "f.x f.y f.z\n";

  for (unsigned int step = 0; step < steps; ++step)
  {
    double t = world->GetSimTime().Double();

    // compute analytical position for underdamped or overdamped case
    math::Vector3 pa;

    // compute analytical velocity by differentiating position
    math::Vector3 va;

    if (underdamped)
    {
      // compute analytical damping ratio
      double wd = wn * sqrt(1.0 - xi * xi);

      // compute analytical constants based on initial conditions
      math::Vector3 a(sqrt(
        (wd*wd*p0.x*p0.x + (v0.x + xi * wn * p0.x) * (v0.x + xi * wn * p0.x)) /
        (wd * wd)), 0, 0);

      // compute analytical frequency lag
      double phi = atan(wd * p0.x / (v0.x + xi * wn * p0.x));

      // compute analytical position
      pa = a * exp(-xi * wn * (t - t0)) * sin( wd * (t - t0) + phi);

      // compute analytical velocity (by chain rule)
      va = -xi*wn*pa +
           wd * a * exp(-xi * wn * (t - t0)) * cos( wd * (t - t0) + phi);
    }
    else
    {
      // compute analytical damping ratio
      double wd = wn * sqrt(xi * xi - 1.0);

      // compute analytical constants based on initial conditions
      math::Vector3 a1 = (-v0 + (-xi + sqrt(xi*xi -1.0))*wn*p0) / (2.0 * wd);
      math::Vector3 a2 = ( v0 + ( xi + sqrt(xi*xi -1.0))*wn*p0) / (2.0 * wd);

      // compute analytical position
      pa = exp(-xi * wn * (t - t0)) * (
           a1 * exp(-wd*wn*(t - t0)) +
           a2 * exp( wd*wn*(t - t0)));

      // compute analytical velocity (by chain rule)
      va = -xi*wn*pa +
           exp(-xi * wn * (t - t0)) * (
           -wd*wn*a1 * exp(-wd*wn*(t - t0)) +
            wd*wn*a2 * exp( wd*wn*(t - t0)));
    }

    // get simulation position and velocity
    math::Vector3 p = link->GetWorldPose().pos;
    math::Vector3 v = link->GetWorldLinearVel();

    // compute spring force, reference position is 0
    math::Vector3 springForce = -stiffness * (p - pr);
    math::Vector3 damperForce = -damping * v;

    // debug results
    std::cout <<  _physicsEngine << " " << t
          << " " << (p -pr) << " " << pa
          << " " << v << " " << v1 << " " << va
          << " " << springForce + damperForce << "\n";

    // check results
    EXPECT_GT(dt, 0);


    // apply force to simulate spring force
    link->SetForce(springForce + damperForce);

    // take a step
    world->Step(1);

    // compute first order integrated velocity
    v1 = v1 + dt * (springForce.x + damperForce.x) / mass;
  }
}

TEST_P(UnconstrainedForced, LinearSpringDamper)
{
  LinearSpringDamper(GetParam());
}

////////////////////////////////////////////////////////////////////////
// Linear Spring
////////////////////////////////////////////////////////////////////////
void UnconstrainedForced::LinearSpring(const std::string &_physicsEngine)
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

  // declare stiffness coefficient
  double stiffness = 10.0;

  // declare initial position of box
  math::Vector3 p0(1.0, 0, 2.0);

  // declare spring reference position
  math::Vector3 pr(0.0, 0, 2.0);

  // set initial pose of link
  link->SetWorldPose(math::Pose(p0, math::Quaternion()));

  // get initial position of link
  math::Vector3 v0 = link->GetWorldLinearVel();

  // cache initial sim time
  double t0 = world->GetSimTime().Double();

  // get time step size
  // double dt = world->GetPhysicsEngine()->GetMaxStepSize();

  // run simulation for 1 second
  double testDuration = 1.0;  // 1 second

  // calculate number of steps needed
  unsigned int steps = testDuration / dt;

  // compute first order integrated velocity with v1
  double v1 = v0.x;

  std::cout << "engine t p.x p.y p.z "
        << "p_analytical.x p_analytical.y p_analytical.z "
        << "v.x v.y v.z v_debug "
        << "v_analytical.x v_analytical.y v_analytical.z "
        << "f.x f.y f.z\n";

  for (unsigned int step = 0; step < steps; ++step)
  {
    double t = world->GetSimTime().Double();

    // compute analytical natural frequency
    double w = sqrt(stiffness / mass);

    // compute analytical position
    math::Vector3 pa = v0 / w * sin( w * (t - t0)) +
                       (p0 - pr) * cos( w * (t - t0));

    // compute analytical velocity by differentiating position
    math::Vector3 va = v0 * cos( w * (t - t0)) -
                       w * (p0 - pr) * sin( w * (t - t0));

    // get simulation position and velocity
    math::Vector3 p = link->GetWorldPose().pos;
    math::Vector3 v = link->GetWorldLinearVel();

    // compute spring force, reference position is 0
    math::Vector3 springForce = -stiffness * (p - pr);

    // debug results
    std::cout <<  _physicsEngine << " " << t
          << " " << (p -pr) << " " << pa
          << " " << v << " " << v1 << " " << va
          << " " << springForce << "\n";

    // check results
    EXPECT_GT(dt, 0);


    // apply force to simulate spring force
    link->SetForce(springForce);

    // take a step
    world->Step(1);

    // compute first order integrated velocity
    v1 = v1 + dt * springForce.x / mass;
  }
}

TEST_P(UnconstrainedForced, LinearSpring)
{
  LinearSpring(GetParam());
}

////////////////////////////////////////////////////////////////////////
// torsional spring force check angular dynamics
////////////////////////////////////////////////////////////////////////
void UnconstrainedForced::AngularDamper(const std::string &_physicsEngine)
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

  std::cout << "engine t p.x p.y p.z "
        << "p_analytical.x p_analytical.y p_analytical.z "
        << "v.x v.y v.z v_debug "
        << "v_analytical.x v_analytical.y v_analytical.z "
        << "f.x f.y f.z\n";

  for (unsigned int step = 0; step < steps; ++step)
  {
    double t = world->GetSimTime().Double();

    // compute analytical velocity
    math::Vector3 va = v0 * exp(-damping / ixx * (t - t0));

    // compute analytical position by integrating velocity
    math::Vector3 ra = r0 - ixx * v0 / damping * (
      exp(-damping / ixx * t) -
      exp(-damping / ixx * t0));

    // get simulation position and velocity
    math::Vector3 r = link->GetWorldPose().rot.GetAsEuler();
    math::Vector3 v = link->GetWorldAngularVel();

    // compute damping force
    math::Vector3 damperForce = -damping * v;

    // debug results
    std::cout <<  _physicsEngine << " " << t
          << " " << r << " " << ra
          << " " << v << " " << v1 << " " << va
          << " " << damperForce << "\n";

    // check results
    EXPECT_GT(dt, 0);


    // apply force to simulate damping force
    link->SetTorque(damperForce);

    // take a step
    world->Step(1);

    // compute first order integrated velocity
    v1 = v1 + dt * damperForce.x / mass;
  }
}

TEST_P(UnconstrainedForced, AngularDamper)
{
  AngularDamper(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, UnconstrainedForced,
  PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
