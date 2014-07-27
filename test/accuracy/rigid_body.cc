/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include <string.h>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "test/ServerFixture.hh"
#include "test/integration/helper_physics_generator.hh"

using namespace gazebo;

// physics engine
// dt
// number of iterations
// gravity on / off
// collision shape on / off
typedef std::tr1::tuple<const char *
                      , double
                      , int
                      , bool
                      , bool
                      > char1double1int1bool2;
class RigidBodyTest : public ServerFixture,
                      public testing::WithParamInterface<char1double1int1bool2>
{
  /// \brief Test accuracy of unconstrained rigid body motion.
  /// \param[in] _physicsEngine Physics engine to use.
  /// \param[in] _dt Max time step size.
  /// \param[in] _iterations Number of iterations.
  /// \param[in] _gravity Flag for turning gravity on / off.
  /// \param[in] _collision Flag for turning collisions on / off.
  public: void Boxes(const std::string &_physicsEngine
                   , double _dt
                   , int _iterations
                   , bool _gravity
                   , bool _collision
                   );
};

/////////////////////////////////////////////////
// Boxes:
// Spawn a single box and record accuracy for momentum and enery
// conservation
void RigidBodyTest::Boxes(const std::string &_physicsEngine
                        , double _dt
                        , int _iterations
                        , bool _gravity
                        , bool _collision
                        )
{
  // Load a blank world (no ground plane)
  Load("worlds/blank.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // get gravity value
  if (!_gravity)
  {
    physics->SetGravity(math::Vector3::Zero);
  }
  math::Vector3 g = physics->GetGravity();

  // Box size
  const double dx = 1.0;
  const double dy = 4.0;
  const double dz = 9.0;
  const double mass = 10.0;

  // Create box with inertia based on box of uniform density
  msgs::Model msgModel;
  msgModel.set_name(this->GetUniqueString("model"));
  msgs::AddBoxLink(msgModel, mass, math::Vector3(dx, dy, dz));
  if (!_collision)
  {
    msgModel.mutable_link(0)->clear_collision();
  }

  physics::ModelPtr model = this->SpawnModel(msgModel);
  ASSERT_TRUE(model != NULL);

  physics::LinkPtr link = model->GetLink();
  ASSERT_TRUE(link != NULL);

  // Give impulses to set initial conditions
  // This is because SimbodyLink::Set*Vel aren't implemented
  link->SetForce(math::Vector3(1e0, 1e1, 1e2));
  link->SetTorque(math::Vector3(1e4, 1e3, 1e2));
  world->Step(1);
  gzdbg << "energy0: " << link->GetWorldEnergy() << std::endl;

  // change step size after impulses
  physics->SetMaxStepSize(_dt);
  if (_physicsEngine == "ode" || _physicsEngine == "bullet")
  {
    gzdbg << "iters: "
          << boost::any_cast<int>(physics->GetParam("iters"))
          << std::endl;
    physics->SetParam("iters", _iterations);
  }
  // else if (_physicsEngine == "simbody")
  // {
  //   gzdbg << "accuracy: "
  //         << boost::any_cast<double>(physics->GetParam("accuracy"))
  //         << std::endl;
  //   physics->SetParam("accuracy", 1.0 / static_cast<float>(_iterations));
  // }
  const double simDuration = 10.0;
  int steps = floor(simDuration / _dt);

  // initial time
  common::Time t0 = world->GetSimTime();

  // initial linear position in global frame
  math::Vector3 p0 = link->GetWorldInertialPose().pos;

  // initial linear velocity in global frame
  math::Vector3 v0 = link->GetWorldCoGLinearVel();

  // initial angular momentum in global frame
  math::Vector3 H0 = link->GetWorldInertiaMatrix() * link->GetWorldAngularVel();
  double H0mag = H0.GetLength();

  // initial energy
  double E0 = link->GetWorldEnergy();

  // variables to compute statistics on
  math::Vector3Stats linearPositionError;
  math::Vector3Stats linearVelocityError;
  math::Vector3Stats angularMomentumError;
  math::SignalStats energyError;
  {
    const std::string statNames = "MaxAbs";
    EXPECT_TRUE(linearPositionError.InsertStatistics(statNames));
    EXPECT_TRUE(linearVelocityError.InsertStatistics(statNames));
    EXPECT_TRUE(angularMomentumError.InsertStatistics(statNames));
    EXPECT_TRUE(energyError.InsertStatistics(statNames));
  }

  // unthrottle update rate
  physics->SetRealTimeUpdateRate(0.0);
  common::Time startTime = common::Time::GetWallTime();
  for (int i = 0; i < steps; ++i)
  {
    world->Step(1);

    // current time
    double t = (world->GetSimTime() - t0).Double();

    // linear velocity error
    math::Vector3 v = link->GetWorldCoGLinearVel();
    linearVelocityError.InsertData(v - (v0 + g*t));

    // linear position error
    math::Vector3 p = link->GetWorldInertialPose().pos;
    linearPositionError.InsertData(p - (p0 + v0 * t + 0.5*g*t*t));

    // angular momentum error
    math::Vector3 H = link->GetWorldInertiaMatrix()*link->GetWorldAngularVel();
    angularMomentumError.InsertData((H - H0) / H0mag);

    // energy error
    energyError.InsertData((link->GetWorldEnergy() - E0) / E0);
  }
  common::Time elapsedTime = common::Time::GetWallTime() - startTime;
  this->Record("elapsedWallTime", elapsedTime.Double());
  this->Record("simTime", world->GetSimTime().Double());

  // Record statistics on pitch and yaw angles
  this->Record("energy0", E0);
  this->Record("energyError", energyError);
  this->Record("angMomentum0", H0mag);
  this->Record("angMomentumErr", angularMomentumError.mag);
  this->Record("linPositionErr", linearPositionError.mag);
  this->Record("linVelocityErr", linearVelocityError.mag);
}

/////////////////////////////////////////////////
TEST_P(RigidBodyTest, Boxes)
{
  std::string physicsEngine = std::tr1::get<0>(GetParam());
  double dt                 = std::tr1::get<1>(GetParam());
  int iterations            = std::tr1::get<2>(GetParam());
  bool gravity              = std::tr1::get<3>(GetParam());
  bool collisions           = std::tr1::get<4>(GetParam());
  gzdbg << physicsEngine
        << ", dt: " << dt
        << ", iters: " << iterations
        << ", gravity: " << gravity
        << ", collisions: " << collisions
        << std::endl;
  Boxes(physicsEngine
      , dt
      , iterations
      , gravity
      , collisions
      );
}

// INSTANTIATE_TEST_CASE_P(Engines, RigidBodyTest,
//   ::testing::Combine(PHYSICS_ENGINE_VALUES
//   , ::testing::Values(1e-3)
//   , ::testing::Values(50)
//   , ::testing::Values(true)
//   , ::testing::Values(true)
//   ));

// INSTANTIATE_TEST_CASE_P(EnginesDt, RigidBodyTest,
//   ::testing::Combine(PHYSICS_ENGINE_VALUES
//   , ::testing::Range(1e-4, 1.01e-3, 4e-4)
//   , ::testing::Values(50)
//   , ::testing::Values(true)
//   , ::testing::Values(true)
//   ));

// INSTANTIATE_TEST_CASE_P(EnginesIters, RigidBodyTest,
//   ::testing::Combine(::testing::Values("ode", "bullet")
//   , ::testing::Values(1e-3)
//   , ::testing::Range(10, 151, 20)
//   , ::testing::Values(true)
//   , ::testing::Values(true)
//   ));

INSTANTIATE_TEST_CASE_P(EnginesGravity, RigidBodyTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES
  , ::testing::Values(1e-3)
  , ::testing::Values(50)
  , ::testing::Bool()
  , ::testing::Values(true)
  ));

INSTANTIATE_TEST_CASE_P(EnginesCollision, RigidBodyTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES
  , ::testing::Values(1e-3)
  , ::testing::Values(50)
  , ::testing::Values(true)
  , ::testing::Bool()
  ));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
