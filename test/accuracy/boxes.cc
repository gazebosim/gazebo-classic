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
// number of boxes to spawn
// nonlinear trajectory on / off
typedef std::tr1::tuple<const char *
                      , double
                      , int
                      , bool
                      > char1double1int1bool1;
class RigidBodyTest : public ServerFixture,
                      public testing::WithParamInterface<char1double1int1bool1>
{
  /// \brief Test accuracy of unconstrained rigid body motion.
  /// \param[in] _physicsEngine Physics engine to use.
  /// \param[in] _dt Max time step size.
  /// \param[in] _modelCount Number of boxes to spawn.
  /// \param[in] _nonlinear Flag for nonlinear trajectory on / off.
  public: void Boxes(const std::string &_physicsEngine
                   , double _dt
                   , int _modelCount
                   , bool _nonlinear
                   );
};

/////////////////////////////////////////////////
// Boxes:
// Spawn a single box and record accuracy for momentum and enery
// conservation
void RigidBodyTest::Boxes(const std::string &_physicsEngine
                        , double _dt
                        , int _modelCount
                        , bool _nonlinear
                        )
{
  // Load a blank world (no ground plane)
  Load("worlds/blank.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  ASSERT_EQ(physics->GetType(), _physicsEngine);

  // get gravity value
  if (!_nonlinear)
  {
    physics->SetGravity(math::Vector3::Zero);
  }
  math::Vector3 g = physics->GetGravity();

  // Box size
  const double dx = 0.1;
  const double dy = 0.4;
  const double dz = 0.9;
  const double mass = 10.0;
  // inertia matrix, recompute if the above change
  const double Ixx = 0.80833333;
  const double Iyy = 0.68333333;
  const double Izz = 0.14166667;
  const math::Matrix3 I0(Ixx, 0.0, 0.0
                       , 0.0, Iyy, 0.0
                       , 0.0, 0.0, Izz);

  // Create box with inertia based on box of uniform density
  msgs::Model msgModel;
  msgs::AddBoxLink(msgModel, mass, math::Vector3(dx, dy, dz));

  // spawn multiple boxes
  // compute error statistics only on the last box
  ASSERT_GT(_modelCount, 0);
  physics::ModelPtr model;
  physics::LinkPtr link;

  // initial linear velocity in global frame
  const math::Vector3 v0(0.1, 0.4, 0.9);

  // initial angular velocity in global frame
  math::Vector3 w0;

  // initial energy value
  double E0;

  if (!_nonlinear)
  {
    // Use angular velocity with one non-zero component
    // to ensure linear angular trajectory
    w0.Set(1.5e-1, 0.0, 0.0);
    E0 = 4.9090937462499999;
  }
  else
  {
    // Since Ixx > Iyy > Izz,
    // angular velocity with large y component
    // will cause gyroscopic tumbling
    w0.Set(1e-3, 1.5e0, 1.5e-2);
    E0 = 5.668765966704;
  }

  for (int i = 0; i < _modelCount; ++i)
  {
    // give models unique names
    msgModel.set_name(this->GetUniqueString("model"));
    // give models unique positions
    msgs::Set(msgModel.mutable_pose()->mutable_position(),
              math::Vector3(dz*2*i, 0.0, 0.0));

    model = this->SpawnModel(msgModel);
    ASSERT_TRUE(model != NULL);

    link = model->GetLink();
    ASSERT_TRUE(link != NULL);

    // Set initial conditions
    link->SetLinearVel(v0);
    link->SetAngularVel(w0);
  }
  ASSERT_EQ(v0, link->GetWorldCoGLinearVel());
  ASSERT_EQ(w0, link->GetWorldAngularVel());
  ASSERT_EQ(I0, link->GetInertial()->GetMOI());
  ASSERT_NEAR(link->GetWorldEnergy(), E0, 1e-6);

  // initial time
  common::Time t0 = world->GetSimTime();

  // initial linear position in global frame
  math::Vector3 p0 = link->GetWorldInertialPose().pos;

  // initial angular momentum in global frame
  math::Vector3 H0 = link->GetWorldInertiaMatrix() * link->GetWorldAngularVel();
  ASSERT_EQ(H0, math::Vector3(Ixx, Iyy, Izz) * w0);
  double H0mag = H0.GetLength();

  // change step size after setting initial conditions
  // since simbody requires a time step
  physics->SetMaxStepSize(_dt);
  const double simDuration = 10.0;
  int steps = ceil(simDuration / _dt);

  // variables to compute statistics on
  math::Vector3Stats linearPositionError;
  math::Vector3Stats linearVelocityError;
  math::Vector3Stats angularPositionError;
  math::Vector3Stats angularMomentumError;
  math::SignalStats energyError;
  {
    const std::string statNames = "MaxAbs";
    EXPECT_TRUE(linearPositionError.InsertStatistics(statNames));
    EXPECT_TRUE(linearVelocityError.InsertStatistics(statNames));
    EXPECT_TRUE(angularPositionError.InsertStatistics(statNames));
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

    // angular position error
    if (!_nonlinear)
    {
      math::Vector3 a = link->GetWorldInertialPose().rot.GetAsEuler();
      math::Quaternion angleTrue(w0 * t);
      angularPositionError.InsertData(a - angleTrue.GetAsEuler());
    }

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
  this->Record("angMomentumErr", angularMomentumError.Mag());
  this->Record("angPositionErr", angularPositionError.Mag());
  this->Record("linPositionErr", linearPositionError.Mag());
  this->Record("linVelocityErr", linearVelocityError.Mag());
}

/////////////////////////////////////////////////
TEST_P(RigidBodyTest, Boxes)
{
  std::string physicsEngine = std::tr1::get<0>(GetParam());
  double dt                 = std::tr1::get<1>(GetParam());
  int modelCount            = std::tr1::get<2>(GetParam());
  bool nonlinear            = std::tr1::get<3>(GetParam());
  gzdbg << physicsEngine
        << ", dt: " << dt
        << ", modelCount: " << modelCount
        << ", nonlinear: " << nonlinear
        << std::endl;
  RecordProperty("engine", physicsEngine);
  this->Record("dt", dt);
  RecordProperty("modelCount", modelCount);
  RecordProperty("nonlienar", nonlinear);
  Boxes(physicsEngine
      , dt
      , modelCount
      , nonlinear
      );
}

#define DT_MIN 1e-4
#define DT_MAX 1.01e-3
#define DT_STEP 3.0e-4
INSTANTIATE_TEST_CASE_P(EnginesDtLinear, RigidBodyTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES
  , ::testing::Range(DT_MIN, DT_MAX, DT_STEP)
  , ::testing::Values(1)
  , ::testing::Values(false)
  ));

INSTANTIATE_TEST_CASE_P(EnginesDtNonlinear, RigidBodyTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES
  , ::testing::Range(DT_MIN, DT_MAX, DT_STEP)
  , ::testing::Values(1)
  , ::testing::Values(true)
  ));

#define MODELS_MIN 1
#define MODELS_MAX 105
#define MODELS_STEP 20
INSTANTIATE_TEST_CASE_P(OdeBoxes, RigidBodyTest,
  ::testing::Combine(::testing::Values("ode")
  , ::testing::Values(3.0e-4)
  , ::testing::Range(MODELS_MIN, MODELS_MAX, MODELS_STEP)
  , ::testing::Values(true)
  ));

INSTANTIATE_TEST_CASE_P(BulletBoxes, RigidBodyTest,
  ::testing::Combine(::testing::Values("bullet")
  , ::testing::Values(3.0e-4)
  , ::testing::Range(MODELS_MIN, MODELS_MAX, MODELS_STEP)
  , ::testing::Values(true)
  ));

INSTANTIATE_TEST_CASE_P(SimbodyBoxes, RigidBodyTest,
  ::testing::Combine(::testing::Values("simbody")
  , ::testing::Values(7.0e-4)
  , ::testing::Range(MODELS_MIN, MODELS_MAX, MODELS_STEP)
  , ::testing::Values(true)
  ));

INSTANTIATE_TEST_CASE_P(DartBoxes, RigidBodyTest,
  ::testing::Combine(::testing::Values("dart")
  , ::testing::Values(7.0e-4)
  , ::testing::Range(MODELS_MIN, MODELS_MAX, MODELS_STEP)
  , ::testing::Values(true)
  ));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
