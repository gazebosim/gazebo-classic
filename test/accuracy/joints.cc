/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
// include this for ODE implicit spring flag
#include "gazebo/physics/ode/ODEJoint.hh"
#include "test/ServerFixture.hh"
#include "test/integration/helper_physics_generator.hh"

using namespace gazebo;

// string
//  physics engine
//  joint type
// double
//  dt
//  natural frequency (Hz)
//  magnitude of disturbance
// int
//  number of iterations
//  number of models to spawn
// bool
//  ode implicit spring on / off
typedef std::tr1::tuple<const char *
                      , const char *
                      , double
                      , double
                      , double
                      , int
                      , int
                      , bool
                      > char2double3int2bool1;
class JointsTest : public ServerFixture,
                   public testing::WithParamInterface<char2double3int2bool1>
{
  /// \brief Test accuracy of unconstrained rigid body motion.
  /// \param[in] _physicsEngine Physics engine to use.
  /// \param[in] _jointType Type of joint to simulate.
  /// \param[in] _dt Max time step size.
  /// \param[in] _freq Natural frequency (Hz).
  /// \param[in] _disturbance Magnitude of disturbance.
  /// \param[in] _iterations Number of iterations.
  /// \param[in] _modelCount Number of models to spawn.
  /// \param[in] _implicit Flag for turning ODE implicit springs on / off.
  public: void OneDof(const std::string &_physicsEngine
                   , const std::string &_jointType
                   , double _dt
                   , double _freq
                   , double _disturbance
                   , int _iterations
                   , int _modelCount
                   , bool _implicit
                   );
};

/////////////////////////////////////////////////
// Joints:
// Spawn an array of single degree-of-freedom boxes connected
// to the world with a slider joint
// and record accuracy for momentum and enery conservation
void JointsTest::OneDof(const std::string &_physicsEngine
                        , const std::string &_jointType
                        , double _dt
                        , double _freq
                        , double _disturbance
                        , int _iterations
                        , int _modelCount
                        , bool _implicit
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

  // set gravity value to zero
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

  const math::Vector3 axis(1, 0, 0);

  // scalar moment of inertia along joint axis
  const double I = axis.Dot(math::Vector3(Ixx, Iyy, Izz));

  // Create box with inertia based on box of uniform density
  msgs::Model msgModel;
  msgs::AddBoxLink(msgModel, mass, math::Vector3(dx, dy, dz));
  msgModel.add_joint();
  {
    msgs::Joint *joint = msgModel.mutable_joint(0);
    joint->set_name("joint");
    joint->set_type(msgs::ConvertJointType(_jointType));
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

  // initial energy value
  double E0 = 0.0;

  // spring stiffness
  double stiffness = 0;
  const double damping = 0.0;
  const double springReference = 0.0;

  const double velMag = 2.5;
  double v0mag = 0.0;
  double w0mag = 0.0;
  const double radiansPerSec = 2 * M_PI * _freq;
  if (_jointType == "revolute")
  {
    v0mag = 0.0;
    w0mag = velMag;
    stiffness = I * radiansPerSec * radiansPerSec;
    E0 = 1.302084;
  }
  else if (_jointType == "prismatic")
  {
    v0mag = velMag;
    w0mag = 0.0;
    stiffness = mass * radiansPerSec * radiansPerSec;
    E0 = 31.25;
  }
  E0 += 0.5 * stiffness * springReference * springReference;

  // initial linear velocity in global frame
  const math::Vector3 v0(v0mag * axis);

  // initial angular velocity in global frame
  const math::Vector3 w0(w0mag * axis);

  for (int i = 0; i < _modelCount; ++i)
  {
    // give models unique names
    msgModel.set_name(this->GetUniqueString("model"));
    // give models unique positions
    msgs::Set(msgModel.mutable_pose()->mutable_position(),
              math::Vector3(0.0, dz*2*i, 0.0));

    model = this->SpawnModel(msgModel);
    ASSERT_TRUE(model != NULL);

    link = model->GetLink();
    ASSERT_TRUE(link != NULL);

    joint = model->GetJoint("joint");
    ASSERT_TRUE(joint != NULL);

    // Set stiffness
    joint->SetStiffnessDamping(0, stiffness, damping, springReference);

    // Set initial conditions
    joint->SetVelocity(0, velMag);
    if (_physicsEngine == "ode")
    {
      if (_jointType == "revolute")
      {
        link->SetAngularVel(w0);
      }
      else if (_jointType == "prismatic")
      {
        link->SetLinearVel(v0);
      }

      // Set ODE implicit spring flag
      physics::ODEJointPtr odeJoint =
        boost::dynamic_pointer_cast<physics::ODEJoint>(joint);
      odeJoint->UseImplicitSpringDamper(_implicit);
      ASSERT_EQ(_implicit, odeJoint->UsesImplicitSpringDamper());
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
    {
      int pulse = (i/10) % 2;
      if (_jointType == "revolute")
      {
        math::Vector3 force = _disturbance * pulse * math::Vector3(1, 1, 1);
        force -= axis * axis.Dot(force);
        link->SetForce(force);
      }
      else if (_jointType == "prismatic")
      {
        link->SetTorque(_disturbance * pulse * math::Vector3(1, 1, 1));
      }
    }
    world->Step(1);

    // current time
    double t = (world->GetSimTime() - t0).Double();

    // linear velocity measurement
    math::Vector3 v = link->GetWorldCoGLinearVel();

    // linear position measurement
    math::Vector3 p = link->GetWorldInertialPose().pos;

    // angular position measurement
    math::Vector3 a = link->GetWorldInertialPose().rot.GetAsEuler();

    // angular momentum measurement
    math::Vector3 H = link->GetWorldInertiaMatrix()*link->GetWorldAngularVel();

    if (math::equal(_freq, 0.0))
    {
      // constant velocity / angular momentum
      linearVelocityError.InsertData(v - v0);
      angularMomentumError.InsertData(H - H0);

      // linear position / angle
      linearPositionError.InsertData(p - (p0 + v0 * t));
      math::Quaternion angleTrue(w0 * t);
      angularPositionError.InsertData(a - angleTrue.GetAsEuler());
    }
    else
    {
      // nonlinear trajectory
      double theta = radiansPerSec * t;
      linearVelocityError.InsertData(v - v0 * cos(theta));
      angularMomentumError.InsertData(H - H0 * cos(theta));

      linearPositionError.InsertData(
        p - (p0 + v0 / radiansPerSec * sin(theta)));
      math::Quaternion angleTrue(w0 / radiansPerSec * sin(theta));
      angularPositionError.InsertData(a - angleTrue.GetAsEuler());
    }

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
  this->Record("angPositionErr", angularPositionError.Mag());
  this->Record("angMomentumErr", angularMomentumError.Mag());
  this->Record("linPositionErr", linearPositionError.Mag());
  this->Record("linVelocityErr", linearVelocityError.Mag());
}

/////////////////////////////////////////////////
TEST_P(JointsTest, OneDof)
{
  std::string physicsEngine = std::tr1::get<0>(GetParam());
  std::string jointType     = std::tr1::get<1>(GetParam());
  double dt                 = std::tr1::get<2>(GetParam());
  double freq               = std::tr1::get<3>(GetParam());
  double disturbance        = std::tr1::get<4>(GetParam());
  int iterations            = std::tr1::get<5>(GetParam());
  int modelCount            = std::tr1::get<6>(GetParam());
  bool implicit             = std::tr1::get<7>(GetParam());
  gzdbg << physicsEngine
        << ", " << jointType
        << ", dt: " << dt
        << ", freq: " << freq
        << ", disturbance: " << disturbance
        << ", iters: " << iterations
        << ", modelCount: " << modelCount
        << ", implicit: " << implicit
        << std::endl;
  RecordProperty("engine", physicsEngine);
  RecordProperty("jointType", jointType);
  this->Record("dt", dt);
  this->Record("freq", freq);
  this->Record("disturbance", disturbance);
  RecordProperty("iters", iterations);
  RecordProperty("modelCount", modelCount);
  RecordProperty("implicit", implicit);
  OneDof(physicsEngine
      , jointType
      , dt
      , freq
      , disturbance
      , iterations
      , modelCount
      , implicit
      );
}

#define JOINT_TYPES ::testing::Values("revolute", "prismatic")
// #define DT_MIN 1e-4
// #define DT_MAX 1.01e-3
// #define DT_STEP 2.0e-4
// #define DT_VALUES ::testing::Range(DT_MIN, DT_MAX, DT_STEP)
//#define DT_VALUES ::testing::Values(1e-4, 4e-4, 8e-4, 1.6e-3, 3.2e-3, 6.4e-3)
#define DT_VALUES ::testing::Values(4e-4, 8e-4, 1.6e-3, 3.2e-3, 6.4e-3, 1.28e-2)
// #define ITERS_MIN 5
// #define ITERS_MAX 51
// #define ITERS_STEP 5
// #define ITERS_VALUES ::testing::Range(ITERS_MIN, ITERS_MAX, ITERS_STEP)
#define ITERS_VALUES ::testing::Values(5, 10, 20, 50)

// Linear trajectory with disturbances
// Use multiple dt's (time steps)
// Use multiple iterations for ODE
// frequency = 0
INSTANTIATE_TEST_CASE_P(LinearDtItersOde, JointsTest,
    ::testing::Combine(::testing::Values("ode")
  , JOINT_TYPES
  , DT_VALUES
  , ::testing::Values(0.0)
  , ::testing::Values(99.0)
  , ITERS_VALUES
  , ::testing::Values(1)
  , ::testing::Values(false)
  ));
INSTANTIATE_TEST_CASE_P(LinearDt, JointsTest,
    ::testing::Combine(::testing::Values("dart", "bullet", "simbody")
  , JOINT_TYPES
  , DT_VALUES
  , ::testing::Values(0.0)
  , ::testing::Values(99.0)
  , ::testing::Values(50)
  , ::testing::Values(1)
  , ::testing::Values(false)
  ));

// Nonlinear trajectory with disturbances
// dt = 1e-3
// Use multiple iterations for ODE
// frequency = 10 Hz
#define FREQ_VALUE ::testing::Values(50.0)
INSTANTIATE_TEST_CASE_P(NonlinearDtItersOde, JointsTest,
    ::testing::Combine(::testing::Values("ode")
  , JOINT_TYPES
  , DT_VALUES
  , FREQ_VALUE
  , ::testing::Values(99.0)
  , ITERS_VALUES
  , ::testing::Values(1)
  , ::testing::Bool()
  ));
INSTANTIATE_TEST_CASE_P(NonlinearDt, JointsTest,
    ::testing::Combine(::testing::Values("dart", "bullet", "simbody")
  , JOINT_TYPES
  , DT_VALUES
  , FREQ_VALUE
  , ::testing::Values(99.0)
  , ::testing::Values(50)
  , ::testing::Values(1)
  , ::testing::Values(false)
  ));

// #define MODELS_MIN 1
// #define MODELS_MAX 105
// #define MODELS_STEP 25
// #define MODELS_DT 3.2e-3
// INSTANTIATE_TEST_CASE_P(OneDofsTorqueOde, JointsTest,
//     ::testing::Combine(::testing::Values("ode")
//   , ::testing::Values("prismatic")
//   , ::testing::Values(MODELS_DT)
//   , ::testing::Values(0.0)
//   , ITERS_VALUES
//   , ::testing::Range(MODELS_MIN, MODELS_MAX, MODELS_STEP)
//   , ::testing::Values(false)
//   , ::testing::Values(true)
//   ));
// 
// INSTANTIATE_TEST_CASE_P(OneDofsTorque, JointsTest,
//     ::testing::Combine(::testing::Values("dart", "bullet", "simbody")
//   , ::testing::Values("prismatic")
//   , ::testing::Values(MODELS_DT)
//   , ::testing::Values(0.0)
//   , ::testing::Values(50)
//   , ::testing::Range(MODELS_MIN, MODELS_MAX, MODELS_STEP)
//   , ::testing::Values(false)
//   , ::testing::Values(true)
//   ));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
