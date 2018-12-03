/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/ode/ODESurfaceParams.hh"
#include "gazebo/physics/ode/ODETypes.hh"

#include "test_config.h"

using namespace gazebo;

class WheelSlipTest : public ServerFixture
{
  /// \brief Class to hold parameters for tire tests.
  public: class WheelSlipState
  {
    /// \brief Constructor.
    public: WheelSlipState()
    {
    }

    /// \brief Destructor.
    public: ~WheelSlipState() = default;

    /// \brief Axel force in lateral direction to expect.
    public: double axelForceLateral = 0.0;

    /// \brief Axel force in lateral direction to expect.
    public: double axelForceLongitudinal = 0.0;

    /// \brief Description to print during test loop.
    public: std::string description;

    /// \brief Drum spin speed in rad/s.
    public: double drumSpeed = 0.0;

    /// \brief Steer angle to apply.
    public: ignition::math::Angle steer;

    /// \brief Suspension force to apply in N.
    public: double suspForce = 0.0;

    /// \brief Wheel slip compliance in lateral direction;
    public: double wheelSlipComplianceLateral = 0.01;

    /// \brief Wheel slip compliance in longitudinal direction;
    public: double wheelSlipComplianceLongitudinal = 0.01;

    /// \brief Wheel spin speed in rad/s.
    public: double wheelSpeed = 0.0;

    /// \brief P gain with wheel spin speed.
    public: double wheelSpeedGain = 0.0;

    /// \brief Wheel torque in Nm.
    public: double wheelTorque = 0.0;
  };

  /// \brief Class to hold parameters for each model in the TriballDrift test.
  public: class TriballDriftMeasurement
  {
    /// \brief Reset stats and set reference pose.
    public: void Reset()
    {
      this->pose0 = this->model->WorldPose();
      this->statsPositionX.Reset();
      this->statsVelocityX.Reset();
    }

    /// \brief Take new measurements.
    public: void Update()
    {
      this->statsPositionX.InsertData(
        this->pose0.Pos().X() - this->model->WorldPose().Pos().X());
      this->statsVelocityX.InsertData(
        this->model->WorldLinearVel().X());
    }

    /// \brief Model pointer.
    public: physics::ModelPtr model;

    /// \brief Reference pose.
    public: ignition::math::Pose3d pose0;

    /// \brief Statistics on position error in X.
    public: ignition::math::SignalMaxAbsoluteValue statsPositionX;

    /// \brief Statistics on velocity error in X.
    public: ignition::math::SignalMaxAbsoluteValue statsVelocityX;
  };

  /// \brief Set joint commands for tire testrig.
  /// \param[in] _wheelSpeed Wheel spin speed in rad/s.
  /// \param[in] _drumSpeed Drum spin speed in rad/s.
  /// \param[in] _suspForce Suspension force to apply in N.
  /// \param[in] _steer Steer angle to apply.
  public: void SetCommands(const WheelSlipState &_state);

  /// \brief Publisher of joint commands for the tire model.
  protected: transport::PublisherPtr tireJointCmdPub;

  /// \brief Publisher of joint commands for the drum model.
  protected: transport::PublisherPtr drumJointCmdPub;

  /// \brief Publisher of lateral wheel slip compliance.
  protected: transport::PublisherPtr slipLateralPub;

  /// \brief Publisher of longitudinal wheel slip compliance.
  protected: transport::PublisherPtr slipLongitudinalPub;

  /// \brief Joint pointer for drum spin joint.
  protected: physics::JointPtr drumJoint;

  /// \brief Joint pointer for spin joint.
  protected: physics::JointPtr spinJoint;

  /// \brief Joint pointer for steering joint.
  protected: physics::JointPtr steerJoint;
};

/////////////////////////////////////////////////
// This test simulates a single wheel with the WheelSlipPlugin using
// a friction pyramid model rolling on a dynamometer.
// The testrig controls the rotational speed of the tire and drum
// as well as the wheel steering angle and normal load.
// Test conditions are specified in the WheelSlipState class
// and a force-torque sensor is used to measure the longitudinal
// and lateral friction forces.
TEST_F(WheelSlipTest, TireDrum)
{
  const double metersPerMile = 1609.34;
  const double secondsPerHour = 3600.0;

  Load("worlds/tire_drum_test.world", true);

  // joint command publishers
  this->tireJointCmdPub = node->Advertise<msgs::JointCmd>("~/tire/joint_cmd");
  this->drumJointCmdPub = node->Advertise<msgs::JointCmd>("~/drum/joint_cmd");

  // slip compliance publishers
  this->slipLateralPub = node->Advertise<msgs::GzString>(
      "~/tire/wheel_slip/lateral_compliance");
  this->slipLongitudinalPub = node->Advertise<msgs::GzString>(
      "~/tire/wheel_slip/longitudinal_compliance");

  sensors::ForceTorqueSensorPtr sensor =
    std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(
        sensors::get_sensor("default::tire::axel_wheel::force_torque"));

  ASSERT_NE(sensor, nullptr);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_NE(world, nullptr);

  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_NE(physics, nullptr);

  auto g = world->Gravity();

  physics::ModelPtr wheelModel = world->ModelByName("tire");
  ASSERT_NE(wheelModel, nullptr);

  physics::LinkPtr wheelLink = wheelModel->GetLink("wheel");
  ASSERT_NE(wheelLink, nullptr);

  double wheelMass = 0.0;
  double wheelRadius = 0.0;
  double wheelStiffness = 0.0;
  {
    wheelMass = wheelLink->GetInertial()->Mass();

    physics::CollisionPtr wheelCollision = wheelLink->GetCollision("collision");
    ASSERT_NE(wheelCollision, nullptr);

    physics::ShapePtr shape = wheelCollision->GetShape();
    ASSERT_NE(shape, nullptr);
    ASSERT_TRUE(shape->HasType(physics::Base::CYLINDER_SHAPE)
             || shape->HasType(physics::Base::SPHERE_SHAPE));
    if (shape->HasType(physics::Base::CYLINDER_SHAPE))
    {
      auto cyl = boost::static_pointer_cast<physics::CylinderShape>(shape);
      wheelRadius = cyl->GetRadius();
    }
    else if (shape->HasType(physics::Base::SPHERE_SHAPE))
    {
      auto sph = boost::static_pointer_cast<physics::SphereShape>(shape);
      wheelRadius = sph->GetRadius();
    }

    physics::ODESurfaceParamsPtr surface =
      boost::dynamic_pointer_cast<physics::ODESurfaceParams>(
        wheelCollision->GetSurface());
    ASSERT_NE(surface, nullptr);
    wheelStiffness = surface->kp;
  }

  double modelMass = 0.0;
  {
    for (const auto &link : wheelModel->GetLinks())
    {
      modelMass += link->GetInertial()->Mass();
    }
  }

  double drumRadius = 0.0;
  {
    physics::ModelPtr drumModel = world->ModelByName("drum");
    ASSERT_NE(drumModel, nullptr);

    this->drumJoint =  drumModel->GetJoint("joint");
    ASSERT_NE(this->drumJoint, nullptr);

    physics::LinkPtr drumLink = drumModel->GetLink("link");
    ASSERT_NE(drumLink, nullptr);

    physics::CollisionPtr drumCollision = drumLink->GetCollision("collision");
    ASSERT_NE(drumCollision, nullptr);

    physics::ShapePtr shape = drumCollision->GetShape();
    ASSERT_NE(shape, nullptr);
    ASSERT_TRUE(shape->HasType(physics::Base::CYLINDER_SHAPE));
    auto cyl = boost::static_pointer_cast<physics::CylinderShape>(shape);
    drumRadius = cyl->GetRadius();
  }

  this->spinJoint =  wheelModel->GetJoint("axel_wheel");
  ASSERT_NE(this->spinJoint, nullptr);

  this->steerJoint =  wheelModel->GetJoint("steer");
  ASSERT_NE(this->steerJoint, nullptr);

  WheelSlipState state0;
  // speed in miles / hour, convert to rad/s
  state0.drumSpeed = -25.0 * metersPerMile / secondsPerHour /  drumRadius;
  state0.wheelSpeed = 25.0 * metersPerMile / secondsPerHour / wheelRadius;
  state0.wheelSpeedGain = 1e2;
  state0.suspForce = 1000.0;

  std::vector<WheelSlipState> states;
  {
    WheelSlipState state = state0;
    state.description = "Zero slip";
    state.steer.Degree(0.0);
    state.axelForceLateral = 0.0;
    state.axelForceLongitudinal = 0.0;
    states.push_back(state);
  }
  {
    WheelSlipState state = state0;
    state.description = "Lateral slip: low";
    state.steer.Degree(3.0);
    state.wheelSlipComplianceLateral = 0.1;
    state.axelForceLateral = -state.suspForce *
        sin(state.steer.Radian()) / state.wheelSlipComplianceLateral;
    state.axelForceLongitudinal = 0.0;
    states.push_back(state);
  }
  {
    WheelSlipState state = state0;
    state.description = "Lateral slip: high";
    state.steer.Degree(10);
    state.wheelSpeed *= cos(state.steer.Radian());
    state.axelForceLateral = -state.suspForce;
    state.axelForceLongitudinal = 0.0;
    states.push_back(state);
  }
  {
    WheelSlipState state = state0;
    state.description = "Longitudinal torque control: low";
    state.wheelSpeed = -1.055 * state.drumSpeed * drumRadius / wheelRadius;
    state.wheelTorque = 0.25 * state.suspForce * wheelRadius;
    state.steer.Degree(0.0);
    state.wheelSlipComplianceLateral = 0.1;
    state.axelForceLateral = 0.0;
    state.axelForceLongitudinal = -250.0;
    states.push_back(state);
  }
  {
    WheelSlipState state = state0;
    state.description = "Longitudinal torque control: moderate";
    state.wheelSpeed = -1.12 * state.drumSpeed * drumRadius / wheelRadius;
    state.wheelTorque = 0.5 * state.suspForce * wheelRadius;
    state.steer.Degree(0.0);
    state.wheelSlipComplianceLateral = 0.1;
    state.axelForceLateral = 0.0;
    state.axelForceLongitudinal = -600.0;
    states.push_back(state);
  }

  for (const auto &state : states)
  {
    gzdbg << "Loading state: " << state.description << std::endl;
    this->SetCommands(state);
    common::Time::MSleep(100);
    world->Step(250);

    // Measure certain quantities
    ignition::math::SignalMaxAbsoluteValue statsDrumSpeed;
    ignition::math::SignalMaxAbsoluteValue statsForceLateral;
    ignition::math::SignalMaxAbsoluteValue statsForceLongitudinal;
    ignition::math::SignalMaxAbsoluteValue statsForceVertical;
    ignition::math::SignalMaxAbsoluteValue statsHeight;
    ignition::math::SignalMaxAbsoluteValue statsSteer;
    ignition::math::SignalMaxAbsoluteValue statsWheelSpeed;

    for (int i = 0; i < 1e3; ++i)
    {
      world->Step(1);
      statsDrumSpeed.InsertData(drumJoint->GetVelocity(0) - state.drumSpeed);
      statsHeight.InsertData(wheelLink->WorldPose().Pos().Z()
        - (wheelRadius - state.suspForce / wheelStiffness));
      statsSteer.InsertData(
        (this->steerJoint->Position(0) - state.steer.Radian()));
      statsForceLateral.InsertData(
        sensor->Force().Y() - state.axelForceLateral);
      statsForceLongitudinal.InsertData(
        sensor->Force().X() - state.axelForceLongitudinal);
      statsForceVertical.InsertData(
        sensor->Force().Z() - (state.suspForce - (modelMass-wheelMass)*g.Z()));
      statsWheelSpeed.InsertData(spinJoint->GetVelocity(0) - state.wheelSpeed);
    }
    EXPECT_LT(statsDrumSpeed.Value(), 0.5);
    EXPECT_LT(statsHeight.Value(), 2e-3);
    EXPECT_LT(statsSteer.Value(), 1e-2);
    if (state.description.compare("Longitudinal torque control: moderate") != 0)
    {
      // Lateral forces are really noisy on that test
      EXPECT_LT(statsForceLateral.Value(), state.suspForce * 6.2e-2);
    }
    EXPECT_LT(statsForceLongitudinal.Value(), state.suspForce * 9e-2);
    EXPECT_LT(statsForceVertical.Value(), state.suspForce * 9e-2);
    EXPECT_LT(statsWheelSpeed.Value(), 6e-1);
  }
}

/////////////////////////////////////////////////
// Set the control inputs for the testrig:
// * drum spin speed
// * wheel spin speed
// * suspension force
// * steering angle
// * wheel slip compliance parameters (lateral and longitudinal)
void WheelSlipTest::SetCommands(const WheelSlipState &_state)
{
  // PID gains for joint controllers
  const double wheelSpinI = 0.0;
  const double wheelSpinD = 0.0;
  const double drumSpinP = 1e4;
  const double drumSpinI = 0.0;
  const double drumSpinD = 0.0;
  const double drumLimit = 1e6;

  {
    msgs::JointCmd msg;
    msg.set_name("drum::joint");

    msgs::PID *pid = msg.mutable_velocity();
    pid->set_target(_state.drumSpeed);
    pid->set_p_gain(drumSpinP);
    pid->set_i_gain(drumSpinI);
    pid->set_d_gain(drumSpinD);
    pid->set_limit(drumLimit);

    this->drumJointCmdPub->Publish(msg);
  }

  {
    msgs::JointCmd msg;
    msg.set_name("tire::axel_wheel");

    msgs::PID *pid = msg.mutable_velocity();
    pid->set_target(_state.wheelSpeed);
    pid->set_p_gain(_state.wheelSpeedGain);
    pid->set_i_gain(wheelSpinI);
    pid->set_d_gain(wheelSpinD);

    msg.set_force(_state.wheelTorque);

    this->tireJointCmdPub->Publish(msg);
  }

  {
    msgs::JointCmd msg;
    msg.set_name("tire::world_upright");
    msg.set_force(-_state.suspForce);

    this->tireJointCmdPub->Publish(msg);
  }

  {
    msgs::GzString msg;
    msg.set_data(std::to_string(_state.wheelSlipComplianceLateral));

    this->slipLateralPub->Publish(msg);
  }

  {
    msgs::GzString msg;
    msg.set_data(std::to_string(_state.wheelSlipComplianceLongitudinal));

    this->slipLongitudinalPub->Publish(msg);
  }

  this->steerJoint->SetUpperLimit(0, _state.steer.Radian());
  this->steerJoint->SetLowerLimit(0, _state.steer.Radian());
}

/////////////////////////////////////////////////
// This test measures whether a 3-wheeled "vehicle"
// with locked wheels can hold position on a slope,
// simulated as a ground plane with lateral gravity.
// The model variations include:
// * triball_lumped: Lumped rigid body with fixed collisions for each wheel.
// * triball_fixed: Chassis connected to spherical wheels with fixed joints.
// * triball_revolute: Chassis connected to spherical wheels with revolute
//   joints and using joint friction to keep the wheels locked.
// * triball_wheel_slip: Same as triball_revolute but with the WheelSlipPlugin
//   applied to each wheel.
TEST_F(WheelSlipTest, TriballDrift)
{
  gazebo::common::SystemPaths::Instance()->AddModelPaths(
    PROJECT_SOURCE_PATH "/test/models");

  Load("worlds/triball_drift.world", true);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_NE(world, nullptr);

  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_NE(physics, nullptr);

  physics->SetRealTimeUpdateRate(0);

  auto g = world->Gravity();
  EXPECT_EQ(g, ignition::math::Vector3d(1, 0, -9.8));

  // allow some settling time
  world->Step(100);

  std::map<std::string, TriballDriftMeasurement> measures;
  measures["triball_lumped"].model = world->ModelByName("triball_lumped");
  measures["triball_fixed"].model = world->ModelByName("triball_fixed");
  measures["triball_revolute"].model = world->ModelByName("triball_revolute");
  measures["triball_wheel_slip"].model =
      world->ModelByName("triball_wheel_slip");
  for (auto &measure : measures)
  {
    ASSERT_NE(measure.second.model, nullptr);
    measure.second.Reset();
  }

  for (int i = 0; i < 3e3; ++i)
  {
    world->Step(1);
    for (auto &measure : measures)
    {
      measure.second.Update();
    }
  }

  EXPECT_NEAR(0.0, measures["triball_lumped"].statsPositionX.Value(), 1e-18);
  EXPECT_NEAR(0.0, measures["triball_lumped"].statsVelocityX.Value(), 1e-11);
  EXPECT_NEAR(0.0, measures["triball_fixed"].statsPositionX.Value(), 1e-5);
  EXPECT_NEAR(0.0, measures["triball_fixed"].statsVelocityX.Value(), 1e-5);
  EXPECT_NEAR(0.0, measures["triball_revolute"].statsPositionX.Value(), 2e-4);
  EXPECT_NEAR(0.0, measures["triball_revolute"].statsVelocityX.Value(), 7e-5);
  EXPECT_NEAR(0.0, measures["triball_wheel_slip"].statsPositionX.Value(), 3e-4);
  EXPECT_NEAR(0.0, measures["triball_wheel_slip"].statsVelocityX.Value(), 7e-5);

  // try it again with more ODE solver iterations
  physics->SetParam("iters", 200);
  world->Step(100);

  for (auto &measure : measures)
  {
    ASSERT_NE(measure.second.model, nullptr);
    measure.second.Reset();
  }

  for (int i = 0; i < 3e3; ++i)
  {
    world->Step(1);
    for (auto &measure : measures)
    {
      measure.second.Update();
    }
  }

  EXPECT_NEAR(0.0, measures["triball_lumped"].statsPositionX.Value(), 3e-19);
  EXPECT_NEAR(0.0, measures["triball_lumped"].statsVelocityX.Value(), 1e-11);
  EXPECT_NEAR(0.0, measures["triball_fixed"].statsPositionX.Value(), 1e-8);
  EXPECT_NEAR(0.0, measures["triball_fixed"].statsVelocityX.Value(), 1e-6);
  EXPECT_NEAR(0.0, measures["triball_revolute"].statsPositionX.Value(), 1e-7);
  EXPECT_NEAR(0.0, measures["triball_revolute"].statsVelocityX.Value(), 1e-6);
  EXPECT_NEAR(0.0, measures["triball_wheel_slip"].statsPositionX.Value(), 2e-7);
  EXPECT_NEAR(0.0, measures["triball_wheel_slip"].statsVelocityX.Value(), 1e-6);
}

/////////////////////////////////////////////////
// This test shows tricycles with spherical wheels
// driving under load with and without slip compliance.
// The actual slip should match the predicted value.
TEST_F(WheelSlipTest, TricyclesUphill)
{
  Load("worlds/trisphere_cycle_wheel_slip.world", true);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_NE(world, nullptr);

  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_NE(physics, nullptr);

  physics->SetRealTimeUpdateRate(0);

  auto g = world->Gravity();
  EXPECT_EQ(g, ignition::math::Vector3d(-2, 0, -9.8));

  std::map<physics::ModelPtr, double> modelSlips;

  physics::ModelPtr modelSlip0 = world->ModelByName("trisphere_cycle_slip0");
  ASSERT_NE(nullptr, modelSlip0);
  modelSlips[modelSlip0] = 0.0;

  physics::ModelPtr modelSlip1 = world->ModelByName("trisphere_cycle_slip1");
  ASSERT_NE(nullptr, modelSlip1);
  modelSlips[modelSlip1] = 1.0;

  double wheelRadius = 0;
  {
    physics::LinkPtr wheel = modelSlip0->GetLink("wheel_rear_left");
    ASSERT_NE(nullptr, wheel);
    auto collisions = wheel->GetCollisions();
    ASSERT_FALSE(collisions.empty());
    ASSERT_EQ(1u, collisions.size());
    auto collision = collisions.front();
    ASSERT_NE(nullptr, collision);
    auto shape = collision->GetShape();
    ASSERT_NE(nullptr, shape);
    ASSERT_TRUE(shape->HasType(physics::Base::SPHERE_SHAPE));
    auto sphere = boost::dynamic_pointer_cast<physics::SphereShape>(shape);
    ASSERT_NE(nullptr, sphere);
    wheelRadius = sphere->GetRadius();
  }
  ASSERT_GT(wheelRadius, 0);

  const common::PID wheelSpeed(9, 0, 0);
  const common::PID steerPosition(9, 0, 0.1);
  const double feedforwardTorque = 2.15;
  const double angularSpeed = 6.0;
  for (auto modelSlip : modelSlips)
  {
    auto model = modelSlip.first;
    auto jc = model->GetJointController();
    jc->SetForce(model->GetScopedName() + "::wheel_rear_left_spin",
        feedforwardTorque);
    jc->SetForce(model->GetScopedName() + "::wheel_rear_right_spin",
        feedforwardTorque);
    jc->SetVelocityPID(model->GetScopedName() + "::wheel_rear_left_spin",
        wheelSpeed);
    jc->SetVelocityPID(model->GetScopedName() + "::wheel_rear_right_spin",
        wheelSpeed);
    jc->SetPositionPID(model->GetScopedName() + "::wheel_front_steer",
        steerPosition);
    jc->SetVelocityTarget(model->GetScopedName() + "::wheel_rear_left_spin",
        angularSpeed);
    jc->SetVelocityTarget(model->GetScopedName() + "::wheel_rear_right_spin",
        angularSpeed);
    jc->SetPositionTarget(model->GetScopedName() + "::wheel_front_steer", 0.0);
  }

  world->Step(2000);

  // compute expected slip
  // normal force as passed to WheelSlipPlugin in test world
  const double wheelNormalForce = 32;
  const double mass = 14.5;
  const double forceRatio = (mass/2) * std::abs(g.X()) / wheelNormalForce;
  const double noSlipLinearSpeed = wheelRadius * angularSpeed;
  for (auto modelSlip : modelSlips)
  {
    auto model = modelSlip.first;
    gzdbg << "checking model slip for " << model->GetName() << std::endl;
    auto jointLeft = model->GetJoint("wheel_rear_left_spin");
    ASSERT_NE(nullptr, jointLeft);
    auto jointRight = model->GetJoint("wheel_rear_right_spin");
    ASSERT_NE(nullptr, jointRight);
    EXPECT_NEAR(angularSpeed, jointLeft->GetVelocity(0), 3e-3);
    EXPECT_NEAR(angularSpeed, jointRight->GetVelocity(0), 3e-3);
    // expected slip
    EXPECT_NEAR(noSlipLinearSpeed - model->WorldLinearVel().X(),
                noSlipLinearSpeed * modelSlip.second * forceRatio, 5e-3);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
