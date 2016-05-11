/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include <ignition/math/Rand.hh>

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "gazebo/test/helper_physics_generator.hh"

// How tightly to compare for deterministic values
#define IMU_TOL 1e-5

using namespace gazebo;
class ImuTest : public ServerFixture,
                public testing::WithParamInterface<const char*>
{
  /// \brief start imu_sensor_test.world, which contains a pendulum,
  /// a sphere with frictional contact, a sphere with frictionless
  /// contact and a ramp.  Each model has an IMU attached.
  /// This test check results to make sure the readings adhere to
  /// each simple model under gravity.
  public: void ImuSensorTestWorld(const std::string &_physicsEngine);

  /// \brief Spawn a static model with an ImuSensor attached
  /// in the empty world.  Test basic IMU outputs.
  public: void StationaryEmptyWorld(const std::string &_physicsEngine);

  /// \brief Spawn a static model with an ImuSensor attached
  /// in the empty world.  Test basic IMU outputs with noise enabled.
  public: void StationaryEmptyWorldNoise(const std::string &_physicsEngine);

  /// \brief Spawn a static model with an ImuSensor attached
  /// in the empty world.  Test basic IMU outputs with bias enabled.
  public: void StationaryEmptyWorldBias(const std::string &_physicsEngine);

  /// \breif Return gravity rotated by some orientation
  /// \param[in] _rot User specified rotation
  /// \param[out] _g gravity in user specified orientation
  private: void Gravity(
               const ignition::math::Quaterniond &_rot,
               ignition::math::Vector3d &_g);

  /// \breif Collect a number of samples and return the average
  /// rate and accel values
  /// \param[in] _imu Pointer to sensor
  /// \param[in] _cnt number of samples to tak
  /// \param[out] _rateMean average angular rates over samples
  /// \param[out] _accelMean average accelerations over samples
  /// \param[out] _orientation orientation of the imu at the end of sample
  /// period
  private: void ImuData(sensors::ImuSensorPtr _imu, unsigned int _cnt,
               ignition::math::Vector3d &_rateMean,
               ignition::math::Vector3d &_accelMean,
               ignition::math::Quaterniond &_orientation);
};

void ImuTest::Gravity(const ignition::math::Quaterniond &_rot,
    ignition::math::Vector3d &_g)
{
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  // Rotate into IMU's frame
  _g = _rot.Inverse().RotateVector(physics->Gravity());
}

void ImuTest::ImuData(sensors::ImuSensorPtr _imu,
                      unsigned int _cnt,
                      ignition::math::Vector3d &_rateMean,
                      ignition::math::Vector3d &_accelMean,
                      ignition::math::Quaterniond &_orientation)
{
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  // Collect a number of samples and return the average rate and accel values
  ignition::math::Vector3d rateSum, accelSum;
  for (unsigned int i = 0; i < _cnt; ++i)
  {
    world->Step(1);

    int j = 0;
    while (_imu->LastMeasurementTime() == gazebo::common::Time::Zero &&
        j < 100)
    {
      _imu->Update(true);
      gazebo::common::Time::MSleep(100);
      ++j;
    }

    EXPECT_LT(j, 100);

    rateSum += _imu->AngularVelocity();
    accelSum += _imu->LinearAcceleration();
  }
  _rateMean = rateSum / _cnt;
  _accelMean = accelSum / _cnt;
  _orientation = _imu->Orientation();
}

void ImuTest::ImuSensorTestWorld(const std::string &_physicsEngine)
{
  if (_physicsEngine != "ode")
  {
    gzerr << "not working yet for anything other than ode. see issue #9999.\n";
    return;
  }

  Load("worlds/imu_sensor_test.world", true, _physicsEngine);

  // get world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // get physics engine
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);

  // get pendulum
  std::string pendulumName = "model_pendulum";
  physics::ModelPtr pendulumModel = world->ModelByName(pendulumName);
  ASSERT_TRUE(pendulumModel != NULL);

  std::string pendulumSensorName = "pendulum_imu_sensor";
  sensors::ImuSensorPtr pendulumImu =
    std::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(pendulumSensorName));
  ASSERT_TRUE(pendulumImu != NULL);
  pendulumImu->Init();

  // get friction ball
  std::string ballFrictionName = "model_ball";
  physics::ModelPtr ballFrictionModel = world->ModelByName(ballFrictionName);
  ASSERT_TRUE(ballFrictionModel != NULL);

  std::string ballFrictionSensorName = "ball_imu_sensor";
  sensors::ImuSensorPtr ballFrictionImu =
    std::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(ballFrictionSensorName));
  ASSERT_TRUE(ballFrictionImu != NULL);
  ballFrictionImu->Init();

  // get frictionless ball
  std::string ballNoFrictionName = "model_ball_no_friction";
  physics::ModelPtr ballNoFrictionModel = world->ModelByName(ballNoFrictionName);
  ASSERT_TRUE(ballNoFrictionModel != NULL);

  std::string ballNoFrictionSensorName = "ball_no_friction_imu_sensor";
  sensors::ImuSensorPtr ballNoFrictionImu =
    std::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(ballNoFrictionSensorName));
  ASSERT_TRUE(ballNoFrictionImu != NULL);
  ballNoFrictionImu->Init();

  // get gravity
  ignition::math::Vector3d g = physics->Gravity();

  // run for 1900 steps (Step(1) each time), or 1.9 seconds, enough
  // to capture what we need from this experiment.
  for (unsigned n = 0; n < 1900; ++n)
  {
    world->Step(1);
    // gzdbg << "time: " << world->SimTime().Double() << "\n";

    // pendulum
    // on startup
    //   sensor linear accel [0 0 0]
    //   Link::RelativeLinearAccel() [0 0 -9.81]
    //   Link::WorldLinearAccel() [0 0 -9.81]
    // @T=1.872 sec, at max lowest position
    //   sensor linear accel [-0 -0.041216 29.4258]
    //   Link::RelativeLinearAccel() [-0 -0.008923 19.6159]
    //   Link::WorldLinearAccel() [-0 0.055649 19.6158]
    {
      // get states from imu sensor
      ignition::math::Vector3d imuLinearAccel =
        pendulumImu->LinearAcceleration();
      // get states from link
      ignition::math::Vector3d relativeLinearAccel =
        pendulumModel->RelativeLinearAccel();
      ignition::math::Vector3d worldLinearAccel =
        pendulumModel->WorldLinearAccel();

      if (world->SimTime().Double() == 1.872)
      {
        // initial values
        EXPECT_NEAR(imuLinearAccel.X(), 0, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.Y(), -0.041216, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.Z(), 29.42581726, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.X(), 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.Y(), -0.036397, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.Z(), 19.6158848, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.X(), 0, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.Y(), -0.0267709, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.Z(), 19.6159003, IMU_TOL);
      }
      else
      {
        // initial values
        EXPECT_LE(imuLinearAccel.Z(), 29.4259);
        EXPECT_LE(relativeLinearAccel.Z(), 19.616);
        EXPECT_LE(worldLinearAccel.Z(), 19.616);
      }
    }

    // friction ball
    // before contact
    //   sensor linear accel [0 0 0]
    //   Link::RelativeLinearAccel() [0 0 -9.81]
    //   Link::WorldLinearAccel() [0 0 -9.81]
    //
    // @T=1.2 sec, on ramp - varies, e.g.
    //   sensor linear accel [-7.81558 0 3.71003]
    //   Link::RelativeLinearAccel() [1.98569 0 3.29613]
    //   Link::WorldLinearAccel() [3.37698 0 -1.84485]
    //
    // @T=1.849 sec, on ground - sensor vector rotates
    //   sensor linear accel [-2.93844 0 9.35958]
    //   Link::RelativeLinearAccel() [0 0 0]
    //   Link::WorldLinearAccel() [0 0 0]
    {
      // get states from imu sensor
      ignition::math::Vector3d imuLinearAccel =
        ballFrictionImu->LinearAcceleration();
      // get states from link
      ignition::math::Vector3d relativeLinearAccel =
        ballFrictionModel->RelativeLinearAccel();
      ignition::math::Vector3d worldLinearAccel =
        ballFrictionModel->WorldLinearAccel();

      if (world->SimTime().Double() <= 1.0)
      {
        // freefall
        EXPECT_NEAR(imuLinearAccel.X(), 0, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.Y(), 0, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.Z(), 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.X(), g.X(), IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.Y(), g.Y(), IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.Z(), g.Z(), IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.X(), g.X(), IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.Y(), g.Y(), IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.Z(), g.Z(), IMU_TOL);
      }
      // should use contact detector for these timing stuff
      else if (world->SimTime().Double() >= 1.2 &&
               world->SimTime().Double() <= 1.84)
      {
        // on ramp
        // ...hm, not much can be said in simple terms, leave out for now.
      }
      else if (world->SimTime().Double() >= 1.85)
      {
        // on the ground
        double imuMag = imuLinearAccel.Length();
        double gMag = g.Length();
        EXPECT_NEAR(imuMag, gMag, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.X(), 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.Y(), 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.Z(), 0, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.X(), 0, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.Y(), 0, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.Z(), 0, IMU_TOL);
      }
    }

    // frictionless ball
    // before contact
    //   sensor linear accel [0 0 0]
    //   Link::RelativeLinearAccel() [0 0 -9.81]
    //   Link::WorldLinearAccel() [0 0 -9.81]
    // @T=1.2 sec, on ramp - constant
    //   sensor linear accel [4.12742 0 7.55518]
    //   Link::RelativeLinearAccel() [4.12742 0 -2.25482]
    //   Link::WorldLinearAccel() [4.12742 0 -2.25482]
    // @T=1.8 sec, on ground - constant
    //   sensor linear accel [0 0 9.81]
    //   Link::RelativeLinearAccel() [0 0 0]
    //   Link::WorldLinearAccel() [0 0 0]
    {
      // get states from imu sensor
      ignition::math::Vector3d imuLinearAccel =
        ballNoFrictionImu->LinearAcceleration();
      // get states from link
      ignition::math::Vector3d relativeLinearAccel =
        ballNoFrictionModel->RelativeLinearAccel();
      ignition::math::Vector3d worldLinearAccel =
        ballNoFrictionModel->WorldLinearAccel();

      if (world->SimTime().Double() <= 1.0)
      {
        // freefall
        EXPECT_NEAR(imuLinearAccel.X(), 0, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.Y(), 0, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.Z(), 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.X(), g.X(), IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.Y(), g.Y(), IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.Z(), g.Z(), IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.X(), g.X(), IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.Y(), g.Y(), IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.Z(), g.Z(), IMU_TOL);
      }
      else if (world->SimTime().Double() >= 1.3 &&
               world->SimTime().Double() <= 1.751)
      {
        // on the ramp
        const double rampAngle = 0.5;
        double gMag = g.Length();
        double imuMag = imuLinearAccel.Length();
        EXPECT_NEAR(imuMag, gMag*cos(rampAngle), IMU_TOL);

        double relMag = relativeLinearAccel.Length();
        EXPECT_NEAR(relMag, gMag*sin(rampAngle), IMU_TOL);
        double worMag = worldLinearAccel.Length();
        EXPECT_NEAR(worMag, gMag*sin(rampAngle), IMU_TOL);
      }
      else if (world->SimTime().Double() >= 1.8)
      {
        // on the ground
        double imuMag = imuLinearAccel.Length();
        double gMag = g.Length();
        EXPECT_NEAR(imuMag, gMag, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.X(), 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.Y(), 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.Z(), 0, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.X(), 0, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.Y(), 0, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.Z(), 0, IMU_TOL);
      }
    }
  }
}

TEST_P(ImuTest, ImuSensorTestWorld)
{
  ImuSensorTestWorld(GetParam());
}

void ImuTest::StationaryEmptyWorld(const std::string &_physicsEngine)
{
  // static models not fully working in simbody yet
  if (_physicsEngine == "simbody")
  {
    gzerr << "Aborting test for Simbody, see issue #860.\n";
    return;
  }

  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "imu_model";
  std::string imuSensorName = "imu_sensor";
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0, 0, 0.05),
      ignition::math::Quaterniond(0.5, -1.0, 0.2));

  SpawnImuSensor(modelName, imuSensorName, testPose.Pos(),
      testPose.Rot().Euler());

  sensors::ImuSensorPtr imu =
    std::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(imuSensorName));

  ASSERT_TRUE(imu != NULL);
  imu->Init();
  ignition::math::Vector3d rateMean, accelMean;
  ignition::math::Quaterniond orientation;
  this->ImuData(imu, 1, rateMean, accelMean, orientation);

  EXPECT_NEAR(rateMean.X(), 0.0, IMU_TOL);
  EXPECT_NEAR(rateMean.Y(), 0.0, IMU_TOL);
  EXPECT_NEAR(rateMean.Z(), 0.0, IMU_TOL);

  ignition::math::Vector3d g;
  this->Gravity(testPose.Rot(), g);
  EXPECT_NEAR(accelMean.X(), -g.X(), IMU_TOL);
  EXPECT_NEAR(accelMean.Y(), -g.Y(), IMU_TOL);
  EXPECT_NEAR(accelMean.Z(), -g.Z(), IMU_TOL);

  // Orientation should be identity, since it is reported relative
  // to reference pose.
  EXPECT_NEAR(orientation.X(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.Y(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.Z(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.W(), 1, IMU_TOL);
}

TEST_P(ImuTest, EmptyWorld)
{
  StationaryEmptyWorld(GetParam());
}

void ImuTest::StationaryEmptyWorldNoise(const std::string &_physicsEngine)
{
  // static models not fully working in simbody yet
  if (_physicsEngine == "simbody")
  {
    gzerr << "Aborting test for Simbody, see issue #860.\n";
    return;
  }

  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "imu_model";
  std::string imuSensorName = "imu_sensor";
  ignition::math::Pose3d testPose(
      ignition::math::Vector3d(0, 0, 0.05),
      ignition::math::Quaterniond(0.3, -1.4, 2.0));

  double rateNoiseMean = 1.0;
  double rateNoiseStddev = 0.1;
  double rateBiasMean = 0.0;
  double rateBiasStddev = 0.0;
  double accelNoiseMean = -10.0;
  double accelNoiseStddev = 0.1;
  double accelBiasMean = 0.0;
  double accelBiasStddev = 0.0;
  SpawnImuSensor(modelName, imuSensorName, testPose.Pos(),
      testPose.Rot().Euler(), "gaussian",
      rateNoiseMean, rateNoiseStddev,
      rateBiasMean, rateBiasStddev,
      accelNoiseMean, accelNoiseStddev,
      accelBiasMean, accelBiasStddev);

  sensors::ImuSensorPtr imu =
    std::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(imuSensorName));

  ASSERT_TRUE(imu != NULL);
  imu->Init();
  ignition::math::Vector3d rateMean, accelMean;
  ignition::math::Quaterniond orientation;
  this->ImuData(imu, 1000, rateMean, accelMean, orientation);

  double d1, d2;
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(rateMean.X() - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.X() - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.Y() - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.Y() - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.Z() - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.Z() - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);

  ignition::math::Vector3d g;
  this->Gravity(testPose.Rot(), g);
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(accelMean.X() - (accelNoiseMean + accelBiasMean) + g.X());
  d2 = fabs(accelMean.X() - (accelNoiseMean - accelBiasMean) + g.X());
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.Y() - (accelNoiseMean + accelBiasMean) + g.Y());
  d2 = fabs(accelMean.Y() - (accelNoiseMean - accelBiasMean) + g.Y());
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.Z() - (accelNoiseMean + accelBiasMean) + g.Z());
  d2 = fabs(accelMean.Z() - (accelNoiseMean - accelBiasMean) + g.Z());
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);

  // Orientation should be identity, since it is reported relative
  // to reference pose.
  EXPECT_NEAR(orientation.X(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.Y(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.Z(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.W(), 1, IMU_TOL);
}

TEST_P(ImuTest, EmptyWorldNoise)
{
  StationaryEmptyWorldNoise(GetParam());
}

void ImuTest::StationaryEmptyWorldBias(const std::string &_physicsEngine)
{
  // static models not fully working in simbody yet
  if (_physicsEngine == "simbody")
  {
    gzerr << "Aborting test for Simbody, see issue #860.\n";
    return;
  }

  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "imu_model";
  std::string imuSensorName = "imu_sensor";
  ignition::math::Pose3d testPose(
      ignition::math::Vector3d(0, 0, 0.05),
      ignition::math::Quaterniond(-0.3, 0.5, 1.0));

  double rateNoiseMean = 0.0;
  double rateNoiseStddev = 0.0;
  double rateBiasMean = 1.0;
  double rateBiasStddev = 0.1;
  double accelNoiseMean = 0.0;
  double accelNoiseStddev = 0.0;
  double accelBiasMean = 5.0;
  double accelBiasStddev = 0.1;
  SpawnImuSensor(modelName, imuSensorName, testPose.Pos(),
      testPose.Rot().Euler(), "gaussian",
      rateNoiseMean, rateNoiseStddev,
      rateBiasMean, rateBiasStddev,
      accelNoiseMean, accelNoiseStddev,
      accelBiasMean, accelBiasStddev);

  sensors::ImuSensorPtr imu =
    std::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(imuSensorName));

  ASSERT_TRUE(imu != NULL);
  imu->Init();
  ignition::math::Vector3d rateMean, accelMean;
  ignition::math::Quaterniond orientation;
  this->ImuData(imu, 1000, rateMean, accelMean, orientation);

  double d1, d2;
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(rateMean.X() - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.X() - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.Y() - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.Y() - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.Z() - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.Z() - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);

  ignition::math::Vector3d g;
  this->Gravity(testPose.Rot(), g);
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(accelMean.X() - (accelNoiseMean + accelBiasMean) + g.X());
  d2 = fabs(accelMean.X() - (accelNoiseMean - accelBiasMean) + g.X());
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.Y() - (accelNoiseMean + accelBiasMean) + g.Y());
  d2 = fabs(accelMean.Y() - (accelNoiseMean - accelBiasMean) + g.Y());
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.Z() - (accelNoiseMean + accelBiasMean) + g.Z());
  d2 = fabs(accelMean.Z() - (accelNoiseMean - accelBiasMean) + g.Z());
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);

  // Orientation should be identity, since it is reported relative
  // to reference pose.
  EXPECT_NEAR(orientation.X(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.Y(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.Z(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.W(), 1, IMU_TOL);
}

TEST_P(ImuTest, EmptyWorldBias)
{
  StationaryEmptyWorldBias(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, ImuTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ignition::math::Rand::Seed(42);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
