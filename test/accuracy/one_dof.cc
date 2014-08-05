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
// number of models to spawn
// gravity on / off
// force disturbance on / off
// torque disturbance on / off
typedef std::tr1::tuple<const char *
                      , double
                      , int
                      , int
                      , bool
                      , bool
                      , bool
                      > char1double1int2bool3;
class OneDofTest : public ServerFixture,
                   public testing::WithParamInterface<char1double1int2bool3>
{
  /// \brief Test accuracy of unconstrained rigid body motion.
  /// \param[in] _physicsEngine Physics engine to use.
  /// \param[in] _dt Max time step size.
  /// \param[in] _iterations Number of iterations.
  /// \param[in] _modelCount Number of models to spawn.
  /// \param[in] _gravity Flag for turning gravity on / off.
  /// \param[in] _force Flag for force disturbance on / off.
  /// \param[in] _torque Flag for torque disturbance on / off.
  public: void Slider(const std::string &_physicsEngine
                   , double _dt
                   , int _iterations
                   , int _modelCount
                   , bool _gravity
                   , bool _force
                   , bool _torque
                   );
};

/////////////////////////////////////////////////
// OneDof:
// Spawn an array of single degree-of-freedom boxes connected
// to the world with a slider joint
// and record accuracy for momentum and enery conservation
void OneDofTest::Slider(const std::string &_physicsEngine
                        , double _dt
                        , int _iterations
                        , int _modelCount
                        , bool _gravity
                        , bool _force
                        , bool _torque
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
  if (!_gravity)
  {
    physics->SetGravity(math::Vector3::Zero);
  }
  math::Vector3 g = physics->GetGravity();

  // Box size
  const double dx = 0.5;
  const double dy = 0.5;
  const double dz = 0.5;
  const double mass = 10.0;
  // inertia matrix, recompute if the above change
  const double Ixx = 0.41666667;
  const double Iyy = 0.41666667;
  const double Izz = 0.41666667;
  const math::Matrix3 I0(Ixx, 0.0, 0.0
                       , 0.0, Iyy, 0.0
                       , 0.0, 0.0, Izz);

  const math::Vector3 axis(0, 1, 0);

  // Create box with inertia based on box of uniform density
  msgs::Model msgModel;
  msgs::AddBoxLink(msgModel, mass, math::Vector3(dx, dy, dz));
  msgModel.add_joint();
  {
    msgs::Joint *joint = msgModel.mutable_joint(0);
    joint->set_name("joint");
    joint->set_type(msgs::Convert("prismatic"));
    joint->set_parent("world");
    joint->set_child(msgModel.link(0).name());
    msgs::Set(joint->mutable_axis1()->mutable_xyz(), axis);
  }

  // spawn multiple boxes
  // compute error statistics only on the last box
  ASSERT_GT(_modelCount, 0);
  physics::ModelPtr model;
  physics::LinkPtr link;
  physics::JointPtr joint;

  // initial linear velocity in global frame
  const math::Vector3 v0(2.5 * axis);

  // initial angular velocity in global frame
  const math::Vector3 w0(0.0, 0.0, 0.0);

  // initial energy value
  const double E0 = 31.25;

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

    joint = model->GetJoint("joint");
    ASSERT_TRUE(joint != NULL);

    // Set initial conditions
    joint->SetVelocity(0, v0.GetLength());
    if (_physicsEngine == "ode")
    {
      link->SetLinearVel(v0);
    }
  }
  ASSERT_EQ(v0, link->GetWorldCoGLinearVel());
  ASSERT_EQ(w0, link->GetWorldAngularVel());
  ASSERT_EQ(I0, link->GetInertial()->GetMOI());
  ASSERT_NEAR(model->GetWorldEnergy(), E0, 1e-6);

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
    if (_force)
    {
      math::Vector3 force = 99 * ((i/10) % 2) * math::Vector3(1, 1, 1);
      force -= axis * axis.Dot(force);
      link->SetForce(force);
    }
    if (_torque)
    {
      link->SetTorque(99 * ((i/10) % 2) * math::Vector3(1, 1, 1));
    }
    world->Step(1);

    // current time
    double t = (world->GetSimTime() - t0).Double();

    // linear velocity error
    math::Vector3 v = link->GetWorldCoGLinearVel();
    linearVelocityError.InsertData(v - v0);

    // linear position error
    math::Vector3 p = link->GetWorldInertialPose().pos;
    linearPositionError.InsertData(p - (p0 + v0 * t));

    // angular position error
    math::Vector3 a = link->GetWorldInertialPose().rot.GetAsEuler();
    angularPositionError.InsertData(a);

    // angular momentum error
    math::Vector3 H = link->GetWorldInertiaMatrix()*link->GetWorldAngularVel();
    angularMomentumError.InsertData((H - H0) / H0mag);

    // energy error
    energyError.InsertData((model->GetWorldEnergy() - E0) / E0);
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
  this->Record("angPositionErr", angularPositionError.mag);
  this->Record("angMomentumErr", angularMomentumError.mag);
  this->Record("linPositionErr", linearPositionError.mag);
  this->Record("linVelocityErr", linearVelocityError.mag);
}

/////////////////////////////////////////////////
TEST_P(OneDofTest, Slider)
{
  std::string physicsEngine = std::tr1::get<0>(GetParam());
  double dt                 = std::tr1::get<1>(GetParam());
  int iterations            = std::tr1::get<2>(GetParam());
  int modelCount            = std::tr1::get<3>(GetParam());
  bool gravity              = std::tr1::get<4>(GetParam());
  bool force                = std::tr1::get<5>(GetParam());
  bool torque               = std::tr1::get<6>(GetParam());
  gzdbg << physicsEngine
        << ", dt: " << dt
        << ", iters: " << iterations
        << ", modelCount: " << modelCount
        << ", gravity: " << gravity
        << ", force: " << force
        << ", torque: " << torque
        << std::endl;
  RecordProperty("engine", physicsEngine);
  this->Record("dt", dt);
  RecordProperty("iters", iterations);
  RecordProperty("modelCount", modelCount);
  RecordProperty("gravity", gravity);
  RecordProperty("force", force);
  RecordProperty("torque", torque);
  Slider(physicsEngine
      , dt
      , iterations
      , modelCount
      , gravity
      , force
      , torque
      );
}

#define DT_MIN 1e-4
#define DT_MAX 1.01e-3
#define DT_STEP 3.0e-4
#define ITERS_MIN 10
#define ITERS_MAX 51
#define ITERS_STEP 20
INSTANTIATE_TEST_CASE_P(EnginesDtItersTorque, OneDofTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES
  , ::testing::Range(DT_MIN, DT_MAX, DT_STEP)
  , ::testing::Range(ITERS_MIN, ITERS_MAX, ITERS_STEP)
  , ::testing::Values(1)
  , ::testing::Values(false)
  , ::testing::Values(false)
  , ::testing::Values(true)
  ));

#define MODELS_MIN 1
#define MODELS_MAX 85
#define MODELS_STEP 20
INSTANTIATE_TEST_CASE_P(OdeSliders, OneDofTest,
  ::testing::Combine(::testing::Values("ode")
  , ::testing::Values(3.0e-4)
  , ::testing::Values(50)
  , ::testing::Range(MODELS_MIN, MODELS_MAX, MODELS_STEP)
  , ::testing::Values(true)
  , ::testing::Values(true)
  , ::testing::Values(false)
  ));

INSTANTIATE_TEST_CASE_P(BulletSliders, OneDofTest,
  ::testing::Combine(::testing::Values("bullet")
  , ::testing::Values(3.0e-4)
  , ::testing::Values(50)
  , ::testing::Range(MODELS_MIN, MODELS_MAX, MODELS_STEP)
  , ::testing::Values(true)
  , ::testing::Values(true)
  , ::testing::Values(false)
  ));

INSTANTIATE_TEST_CASE_P(SimbodySliders, OneDofTest,
  ::testing::Combine(::testing::Values("simbody")
  , ::testing::Values(7.0e-4)
  , ::testing::Values(50)
  , ::testing::Range(MODELS_MIN, MODELS_MAX, MODELS_STEP)
  , ::testing::Values(true)
  , ::testing::Values(true)
  , ::testing::Values(false)
  ));

INSTANTIATE_TEST_CASE_P(DartSliders, OneDofTest,
  ::testing::Combine(::testing::Values("dart")
  , ::testing::Values(7.0e-4)
  , ::testing::Values(50)
  , ::testing::Range(MODELS_MIN, MODELS_MAX, MODELS_STEP)
  , ::testing::Values(true)
  , ::testing::Values(true)
  , ::testing::Values(false)
  ));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
