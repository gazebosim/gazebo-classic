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
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "helper_physics_generator.hh"

// How tightly to compare for deterministic values
#define IMU_TOL 1e-5

using namespace gazebo;
class ImuTest : public ServerFixture,
                public testing::WithParamInterface<const char*>
{
  public: void Stationary_EmptyWorld(const std::string &_physicsEngine);
  public: void Stationary_EmptyWorld_Noise(const std::string &_physicsEngine);
  public: void Stationary_EmptyWorld_Bias(const std::string &_physicsEngine);
  private: void GetGravity(const math::Quaternion& _rot, math::Vector3 &_g);
  private: void GetImuData(sensors::ImuSensorPtr _imu, unsigned int _cnt,
                           math::Vector3 &_rateMean,
                           math::Vector3 &_accelMean,
                           math::Quaternion &_orientation);
};

void ImuTest::GetGravity(const math::Quaternion &_rot, math::Vector3 &_g)
{
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world);
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics);
  // Rotate into IMU's frame
  _g = _rot.GetInverse().RotateVector(physics->GetGravity());
}

void ImuTest::GetImuData(sensors::ImuSensorPtr _imu,
                         unsigned int _cnt,
                         math::Vector3 &_rateMean,
                         math::Vector3 &_accelMean,
                         math::Quaternion& _orientation)
{
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world);
  // Collect a number of samples and return the average rate and accel values
  math::Vector3 rateSum, accelSum;
  for (unsigned int i = 0; i < _cnt; ++i)
  {
    world->Step(1);

    int j = 0;
    while (_imu->GetLastMeasurementTime() == gazebo::common::Time::Zero &&
        j < 100)
    {
      _imu->Update(true);
      gazebo::common::Time::MSleep(100);
      ++j;
    }

    EXPECT_LT(j, 100);

    rateSum += _imu->GetAngularVelocity();
    accelSum += _imu->GetLinearAcceleration();
  }
  _rateMean = rateSum / _cnt;
  _accelMean = accelSum / _cnt;
  _orientation = _imu->GetOrientation();
}

void ImuTest::Stationary_EmptyWorld(const std::string &_physicsEngine)
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
  math::Pose testPose(math::Vector3(0, 0, 0.05),
      math::Quaternion(0.5, -1.0, 0.2));

  SpawnImuSensor(modelName, imuSensorName, testPose.pos,
      testPose.rot.GetAsEuler());

  sensors::ImuSensorPtr imu =
    boost::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(imuSensorName));

  ASSERT_TRUE(imu);
  imu->Init();
  math::Vector3 rateMean, accelMean;
  math::Quaternion orientation;
  this->GetImuData(imu, 1, rateMean, accelMean, orientation);

  EXPECT_NEAR(rateMean.x, 0.0, IMU_TOL);
  EXPECT_NEAR(rateMean.y, 0.0, IMU_TOL);
  EXPECT_NEAR(rateMean.z, 0.0, IMU_TOL);

  math::Vector3 g;
  this->GetGravity(testPose.rot, g);
  EXPECT_NEAR(accelMean.x, -g.x, IMU_TOL);
  EXPECT_NEAR(accelMean.y, -g.y, IMU_TOL);
  EXPECT_NEAR(accelMean.z, -g.z, IMU_TOL);

  // Orientation should be identity, since it is reported relative
  // to reference pose.
  EXPECT_NEAR(orientation.x, 0, IMU_TOL);
  EXPECT_NEAR(orientation.y, 0, IMU_TOL);
  EXPECT_NEAR(orientation.z, 0, IMU_TOL);
  EXPECT_NEAR(orientation.w, 1, IMU_TOL);
}

TEST_P(ImuTest, EmptyWorld)
{
  Stationary_EmptyWorld(GetParam());
}

void ImuTest::Stationary_EmptyWorld_Noise(const std::string &_physicsEngine)
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
  math::Pose testPose(math::Vector3(0, 0, 0.05),
      math::Quaternion(0.3, -1.4, 2.0));

  double rateNoiseMean = 1.0;
  double rateNoiseStddev = 0.1;
  double rateBiasMean = 0.0;
  double rateBiasStddev = 0.0;
  double accelNoiseMean = -10.0;
  double accelNoiseStddev = 0.1;
  double accelBiasMean = 0.0;
  double accelBiasStddev = 0.0;
  SpawnImuSensor(modelName, imuSensorName, testPose.pos,
      testPose.rot.GetAsEuler(), "gaussian",
      rateNoiseMean, rateNoiseStddev,
      rateBiasMean, rateBiasStddev,
      accelNoiseMean, accelNoiseStddev,
      accelBiasMean, accelBiasStddev);

  sensors::ImuSensorPtr imu =
    boost::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(imuSensorName));

  ASSERT_TRUE(imu);
  imu->Init();
  math::Vector3 rateMean, accelMean;
  math::Quaternion orientation;
  this->GetImuData(imu, 1000, rateMean, accelMean, orientation);

  double d1, d2;
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(rateMean.x - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.x - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.y - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.y - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.z - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.z - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);

  math::Vector3 g;
  this->GetGravity(testPose.rot, g);
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(accelMean.x - (accelNoiseMean + accelBiasMean) + g.x);
  d2 = fabs(accelMean.x - (accelNoiseMean - accelBiasMean) + g.x);
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.y - (accelNoiseMean + accelBiasMean) + g.y);
  d2 = fabs(accelMean.y - (accelNoiseMean - accelBiasMean) + g.y);
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.z - (accelNoiseMean + accelBiasMean) + g.z);
  d2 = fabs(accelMean.z - (accelNoiseMean - accelBiasMean) + g.z);
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);

  // Orientation should be identity, since it is reported relative
  // to reference pose.
  EXPECT_NEAR(orientation.x, 0, IMU_TOL);
  EXPECT_NEAR(orientation.y, 0, IMU_TOL);
  EXPECT_NEAR(orientation.z, 0, IMU_TOL);
  EXPECT_NEAR(orientation.w, 1, IMU_TOL);
}

TEST_P(ImuTest, EmptyWorldNoise)
{
  Stationary_EmptyWorld_Noise(GetParam());
}

void ImuTest::Stationary_EmptyWorld_Bias(const std::string &_physicsEngine)
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
  math::Pose testPose(math::Vector3(0, 0, 0.05),
      math::Quaternion(-0.3, 0.5, 1.0));

  double rateNoiseMean = 0.0;
  double rateNoiseStddev = 0.0;
  double rateBiasMean = 1.0;
  double rateBiasStddev = 0.1;
  double accelNoiseMean = 0.0;
  double accelNoiseStddev = 0.0;
  double accelBiasMean = 5.0;
  double accelBiasStddev = 0.1;
  SpawnImuSensor(modelName, imuSensorName, testPose.pos,
      testPose.rot.GetAsEuler(), "gaussian",
      rateNoiseMean, rateNoiseStddev,
      rateBiasMean, rateBiasStddev,
      accelNoiseMean, accelNoiseStddev,
      accelBiasMean, accelBiasStddev);

  sensors::ImuSensorPtr imu =
    boost::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(imuSensorName));

  ASSERT_TRUE(imu);
  imu->Init();
  math::Vector3 rateMean, accelMean;
  math::Quaternion orientation;
  this->GetImuData(imu, 1000, rateMean, accelMean, orientation);

  double d1, d2;
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(rateMean.x - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.x - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.y - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.y - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.z - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.z - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);

  math::Vector3 g;
  this->GetGravity(testPose.rot, g);
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(accelMean.x - (accelNoiseMean + accelBiasMean) + g.x);
  d2 = fabs(accelMean.x - (accelNoiseMean - accelBiasMean) + g.x);
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.y - (accelNoiseMean + accelBiasMean) + g.y);
  d2 = fabs(accelMean.y - (accelNoiseMean - accelBiasMean) + g.y);
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.z - (accelNoiseMean + accelBiasMean) + g.z);
  d2 = fabs(accelMean.z - (accelNoiseMean - accelBiasMean) + g.z);
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);

  // Orientation should be identity, since it is reported relative
  // to reference pose.
  EXPECT_NEAR(orientation.x, 0, IMU_TOL);
  EXPECT_NEAR(orientation.y, 0, IMU_TOL);
  EXPECT_NEAR(orientation.z, 0, IMU_TOL);
  EXPECT_NEAR(orientation.w, 1, IMU_TOL);
}

TEST_P(ImuTest, EmptyWorldBias)
{
  Stationary_EmptyWorld_Bias(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, ImuTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  math::Rand::SetSeed(42);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
