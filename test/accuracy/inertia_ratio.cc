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
// inertia of large sphere (all others being 1)
// gravity applied
// force on top sphere
typedef std::tr1::tuple<const char * /* physics engine */
                      , int          /* number of iterations */
                      , double       /* dt */
                      , double       /* mass of large sphere */
                      , double       /* gravity */
                      , double       /* force on top sphere */
                      , double       /* stuff */
                      > char1int1double4;
class RigidBodyTest : public ServerFixture,
                      public testing::WithParamInterface<char1int1double4>
{
  /// \brief Test accuracy of unconstrained rigid body motion.
  /// \param[in] _physicsEngine Physics engine to use.
  /// \param[in] _iterations Number of iterations.
  /// \param[in] _dt Max time step size.
  /// \param[in] _mass Mass of large sphere, all others being 1
  /// \param[in] _gravity gravity applied
  /// \param[in] _force force on top sphere
  public: void InertiaRatioSphereStack(const std::string &_physicsEngine
                            , int _iterations
                            , double _dt
                            , double _mass
                            , double _gravity
                            , double _force
                            , double _stuff
                            );

  /// \brief Test accuracy of unconstrained rigid body motion.
  /// \param[in] _physicsEngine Physics engine to use.
  /// \param[in] _iterations Number of iterations.
  /// \param[in] _dt Max time step size.
  /// \param[in] _mass Mass of large sphere, all others being 1
  /// \param[in] _gravity gravity applied
  /// \param[in] _force force on top sphere
  public: void InertiaRatioBoxStack(const std::string &_physicsEngine
                            , int _iterations
                            , double _dt
                            , double _mass
                            , double _gravity
                            , double _force
                            , double _stuff
                            );
};

/////////////////////////////////////////////////
// InertiaRatioSphereStack:
// Spawn a single box and record accuracy for momentum and enery
// conservation
void RigidBodyTest::InertiaRatioSphereStack(const std::string &_physicsEngine
                                , int _iterations
                                , double _dt
                                , double _mass
                                , double _gravity
                                , double _force
                                , double _stuff
                                )
{
  // Load a blank world (no ground plane)
  Load("worlds/sphere_stack.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  ASSERT_EQ(physics->GetType(), _physicsEngine);

  // get model and link
  physics::ModelPtr model = world->GetModel("sphere_5");
  physics::LinkPtr link = model->GetLink("link");

  // get gravity value
  physics->SetGravity(math::Vector3(0, 0, _gravity));

  math::Vector3 g = physics->GetGravity();

  // initial time
  common::Time t0 = world->GetSimTime();

  // initial linear position in global frame
  math::Vector3 p0 = link->GetWorldInertialPose().pos;

  // initial linear velocity in global frame
  const math::Vector3 v0 = link->GetWorldLinearVel();

  // initial angular velocity in global frame
  math::Vector3 w0 = link->GetWorldAngularVel();

  // initial angular momentum in global frame
  math::Vector3 H0 = link->GetWorldInertiaMatrix() * w0;
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

  // set simulation time step size
  physics->SetMaxStepSize(_dt);

  // setup simulation duration
  const double simDuration = 10.0;
  int steps = ceil(simDuration / _dt);

  // unthrottle update rate
  physics->SetRealTimeUpdateRate(0.0);
  common::Time startTime = common::Time::GetWallTime();
  for (int i = 0; i < steps; ++i)
  {
    world->Step(1);

    // current time
    // double t = (world->GetSimTime() - t0).Double();

    // linear velocity error
    math::Vector3 v = link->GetWorldCoGLinearVel();
    linearVelocityError.InsertData(v - v0);

    // linear position error
    math::Vector3 p = link->GetWorldInertialPose().pos;
    linearPositionError.InsertData(p - p0);

    // angular momentum error
    math::Vector3 H = link->GetWorldInertiaMatrix()*link->GetWorldAngularVel();
    angularMomentumError.InsertData((H - H0) / H0mag);

    // energy error
    energyError.InsertData((link->GetWorldEnergy() - E0) / E0);
  }
  common::Time elapsedTime = common::Time::GetWallTime() - startTime;
  this->Record("wallTime", elapsedTime.Double());
  common::Time simTime = (world->GetSimTime() - t0).Double();
  ASSERT_NEAR(simTime.Double(), simDuration, _dt*1.1);
  this->Record("simTime", simTime.Double());
  this->Record("timeRatio", elapsedTime.Double() / simTime.Double());

  // Record statistics on pitch and yaw angles
  this->Record("energy0", E0);
  this->Record("energyError", energyError);
  this->Record("angMomentum0", H0mag);
  this->Record("angMomentumErr", angularMomentumError.mag);
  this->Record("linPositionErr", linearPositionError.mag);
  this->Record("linVelocityErr", linearVelocityError.mag);
}

/////////////////////////////////////////////////
TEST_P(RigidBodyTest, InertiaRatioSphereStack)
{
  std::string physicsEngine = std::tr1::get<0>(GetParam());
  int    iterations         = std::tr1::get<1>(GetParam());
  double dt                 = std::tr1::get<2>(GetParam());
  double mass               = std::tr1::get<3>(GetParam());
  double gravity            = std::tr1::get<4>(GetParam());
  double force              = std::tr1::get<5>(GetParam());
  double stuff              = std::tr1::get<6>(GetParam());
  gzdbg << physicsEngine
        << ", dt: " << dt
        << ", iters: " << iterations
        << ", mass: " << mass
        << ", gravity: " << gravity
        << ", force: " << force
        << std::endl;
  RecordProperty("engine", physicsEngine);
  this->Record("dt", dt);
  RecordProperty("iters", iterations);
  RecordProperty("mass", mass);
  RecordProperty("gravity", gravity);
  RecordProperty("force", force);
  InertiaRatioSphereStack(physicsEngine
      , iterations
      , dt
      , mass
      , gravity
      , force
      , stuff
      );
}

#define DT_MIN 1e-4
#define DT_MAX 1.01e-3
#define DT_STEP 3.0e-4
INSTANTIATE_TEST_CASE_P(EnginesDtLinearSphereStack, RigidBodyTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES
  , ::testing::Values(50)
  , ::testing::Range(DT_MIN, DT_MAX, DT_STEP)
  , ::testing::Values(1.0)
  , ::testing::Values(-1.0) /* gravity */
  , ::testing::Values(0.0)
  , ::testing::Values(0.0)
  ));

/*
INSTANTIATE_TEST_CASE_P(OdeInertiaRatioSphereStack, RigidBodyTest,
  ::testing::Combine(::testing::Values("ode")
  , ::testing::Values(50)
  , ::testing::Values(3.0e-4)
  , ::testing::Values(1.0)
  , ::testing::Values(-100.0)
  , ::testing::Values(0.0)
  , ::testing::Values(0.0)
  ));

INSTANTIATE_TEST_CASE_P(BulletInertiaRatioSphereStack, RigidBodyTest,
  ::testing::Combine(::testing::Values("bullet")
  , ::testing::Values(50)
  , ::testing::Values(3.0e-4)
  , ::testing::Values(1.0)
  , ::testing::Values(-100.0)
  , ::testing::Values(0.0)
  , ::testing::Values(0.0)
  ));

INSTANTIATE_TEST_CASE_P(SimbodyInertiaRatioSphereStack, RigidBodyTest,
  ::testing::Combine(::testing::Values("simbody")
  , ::testing::Values(50)
  , ::testing::Values(3.0e-4)
  , ::testing::Values(1.0)
  , ::testing::Values(-100.0)
  , ::testing::Values(0.0)
  , ::testing::Values(0.0)
  ));

INSTANTIATE_TEST_CASE_P(DartInertiaRatioSphereStack, RigidBodyTest,
  ::testing::Combine(::testing::Values("dart")
  , ::testing::Values(50)
  , ::testing::Values(3.0e-4)
  , ::testing::Values(1.0)
  , ::testing::Values(-100.0)
  , ::testing::Values(0.0)
  , ::testing::Values(0.0)
  ));
*/

/////////////////////////////////////////////////
// InertiaRatioBoxStack:
// Spawn a single box and record accuracy for momentum and enery
// conservation
void RigidBodyTest::InertiaRatioBoxStack(const std::string &_physicsEngine
                                , int _iterations
                                , double _dt
                                , double _mass
                                , double _gravity
                                , double _force
                                , double _stuff
                                )
{
  // Load a blank world (no ground plane)
  Load("worlds/box_stack.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  ASSERT_EQ(physics->GetType(), _physicsEngine);

  // get model and link
  physics::ModelPtr model = world->GetModel("sphere_5");
  physics::LinkPtr link = model->GetLink("link");

  // get gravity value
  physics->SetGravity(math::Vector3(0, 0, _gravity));

  math::Vector3 g = physics->GetGravity();

  // initial time
  common::Time t0 = world->GetSimTime();

  // initial linear position in global frame
  math::Vector3 p0 = link->GetWorldInertialPose().pos;

  // initial linear velocity in global frame
  const math::Vector3 v0 = link->GetWorldLinearVel();

  // initial angular velocity in global frame
  math::Vector3 w0 = link->GetWorldAngularVel();

  // initial angular momentum in global frame
  math::Vector3 H0 = link->GetWorldInertiaMatrix() * w0;
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

  // set simulation time step size
  physics->SetMaxStepSize(_dt);

  // setup simulation duration
  const double simDuration = 10.0;
  int steps = ceil(simDuration / _dt);

  // unthrottle update rate
  physics->SetRealTimeUpdateRate(0.0);
  common::Time startTime = common::Time::GetWallTime();
  for (int i = 0; i < steps; ++i)
  {
    world->Step(1);

    // current time
    // double t = (world->GetSimTime() - t0).Double();

    // linear velocity error
    math::Vector3 v = link->GetWorldCoGLinearVel();
    linearVelocityError.InsertData(v - v0);

    // linear position error
    math::Vector3 p = link->GetWorldInertialPose().pos;
    linearPositionError.InsertData(p - p0);

    // angular momentum error
    math::Vector3 H = link->GetWorldInertiaMatrix()*link->GetWorldAngularVel();
    angularMomentumError.InsertData((H - H0) / H0mag);

    // energy error
    energyError.InsertData((link->GetWorldEnergy() - E0) / E0);
  }
  common::Time elapsedTime = common::Time::GetWallTime() - startTime;
  this->Record("wallTime", elapsedTime.Double());
  common::Time simTime = (world->GetSimTime() - t0).Double();
  ASSERT_NEAR(simTime.Double(), simDuration, _dt*1.1);
  this->Record("simTime", simTime.Double());
  this->Record("timeRatio", elapsedTime.Double() / simTime.Double());

  // Record statistics on pitch and yaw angles
  this->Record("energy0", E0);
  this->Record("energyError", energyError);
  this->Record("angMomentum0", H0mag);
  this->Record("angMomentumErr", angularMomentumError.mag);
  this->Record("linPositionErr", linearPositionError.mag);
  this->Record("linVelocityErr", linearVelocityError.mag);
}

/////////////////////////////////////////////////
TEST_P(RigidBodyTest, InertiaRatioBoxStack)
{
  std::string physicsEngine = std::tr1::get<0>(GetParam());
  int    iterations         = std::tr1::get<1>(GetParam());
  double dt                 = std::tr1::get<2>(GetParam());
  double mass               = std::tr1::get<3>(GetParam());
  double gravity            = std::tr1::get<4>(GetParam());
  double force              = std::tr1::get<5>(GetParam());
  double stuff              = std::tr1::get<6>(GetParam());
  gzdbg << physicsEngine
        << ", dt: " << dt
        << ", iters: " << iterations
        << ", mass: " << mass
        << ", gravity: " << gravity
        << ", force: " << force
        << std::endl;
  RecordProperty("engine", physicsEngine);
  this->Record("dt", dt);
  RecordProperty("iters", iterations);
  RecordProperty("mass", mass);
  RecordProperty("gravity", gravity);
  RecordProperty("force", force);
  InertiaRatioBoxStack(physicsEngine
      , iterations
      , dt
      , mass
      , gravity
      , force
      , stuff
      );
}

#define DT_MIN 1e-4
#define DT_MAX 1.01e-3
#define DT_STEP 3.0e-4
INSTANTIATE_TEST_CASE_P(EnginesDtLinearBoxStack, RigidBodyTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES
  , ::testing::Values(50)
  , ::testing::Range(DT_MIN, DT_MAX, DT_STEP)
  , ::testing::Values(1.0)
  , ::testing::Values(-1.0) /* gravity */
  , ::testing::Values(0.0)
  , ::testing::Values(0.0)
  ));

/*
INSTANTIATE_TEST_CASE_P(OdeInertiaRatioBoxStack, RigidBodyTest,
  ::testing::Combine(::testing::Values("ode")
  , ::testing::Values(50)
  , ::testing::Values(3.0e-4)
  , ::testing::Values(1.0)
  , ::testing::Values(-100.0)
  , ::testing::Values(0.0)
  , ::testing::Values(0.0)
  ));

INSTANTIATE_TEST_CASE_P(BulletInertiaRatioBoxStack, RigidBodyTest,
  ::testing::Combine(::testing::Values("bullet")
  , ::testing::Values(50)
  , ::testing::Values(3.0e-4)
  , ::testing::Values(1.0)
  , ::testing::Values(-100.0)
  , ::testing::Values(0.0)
  , ::testing::Values(0.0)
  ));

INSTANTIATE_TEST_CASE_P(SimbodyInertiaRatioBoxStack, RigidBodyTest,
  ::testing::Combine(::testing::Values("simbody")
  , ::testing::Values(50)
  , ::testing::Values(3.0e-4)
  , ::testing::Values(1.0)
  , ::testing::Values(-100.0)
  , ::testing::Values(0.0)
  , ::testing::Values(0.0)
  ));

INSTANTIATE_TEST_CASE_P(DartInertiaRatioBoxStack, RigidBodyTest,
  ::testing::Combine(::testing::Values("dart")
  , ::testing::Values(50)
  , ::testing::Values(3.0e-4)
  , ::testing::Values(1.0)
  , ::testing::Values(-100.0)
  , ::testing::Values(0.0)
  , ::testing::Values(0.0)
  ));
*/

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
