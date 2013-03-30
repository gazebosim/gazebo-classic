/*
 * Copyright 2013 Open Source Robotics Foundation
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
#include "sensors/sensors.hh"
#include "common/common.hh"

#define IMU_TOL 1e-6
#define GRAVITY 9.8

using namespace gazebo;
class ImuTest : public ServerFixture
{
  public: void Stationary_EmptyWorld(const std::string &_physicsEngine);
  public: void Stationary_EmptyWorld_Noise(const std::string &_physicsEngine);
  public: void Stationary_EmptyWorld_Bias(const std::string &_physicsEngine);
};

void ImuTest::Stationary_EmptyWorld(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "imu_model";
  std::string imuSensorName = "imu_sensor";
  math::Pose testPose(math::Vector3(0, 0, 0.05),
      math::Quaternion(0, 0, 0));

  SpawnImuSensor(modelName, imuSensorName, testPose.pos,
      testPose.rot.GetAsEuler());

  sensors::ImuSensorPtr imu =
    boost::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(imuSensorName));

  ASSERT_TRUE(imu);
  imu->Init();
  imu->Update(true);

  math::Vector3 rates = imu->GetAngularVelocity();
  EXPECT_NEAR(rates.x, 0.0, IMU_TOL);
  EXPECT_NEAR(rates.y, 0.0, IMU_TOL);
  EXPECT_NEAR(rates.z, 0.0, IMU_TOL);

  math::Vector3 accels = imu->GetLinearAcceleration();
  EXPECT_NEAR(accels.x, 0.0, IMU_TOL);
  EXPECT_NEAR(accels.y, 0.0, IMU_TOL);
  EXPECT_NEAR(accels.z, GRAVITY, IMU_TOL);
}

TEST_F(ImuTest, EmptyWorldODE)
{
  Stationary_EmptyWorld("ode");
}

#ifdef HAVE_BULLET
TEST_F(ImuTest, EmptyWorldBullet)
{
  Stationary_EmptyWorld("bullet");
}
#endif

void ImuTest::Stationary_EmptyWorld_Noise(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "imu_model";
  std::string imuSensorName = "imu_sensor";
  math::Pose testPose(math::Vector3(0, 0, 0.05),
      math::Quaternion(0, 0, 0));

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
  imu->Update(true);

  math::Vector3 rates = imu->GetAngularVelocity();
  double d1, d2;
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(rates.x - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rates.x - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rates.y - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rates.y - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rates.z - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rates.z - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);

  math::Vector3 accels = imu->GetLinearAcceleration();
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(accels.x - (accelNoiseMean + accelBiasMean));
  d2 = fabs(accels.x - (accelNoiseMean - accelBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accels.y - (accelNoiseMean + accelBiasMean));
  d2 = fabs(accels.y - (accelNoiseMean - accelBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accels.z - (accelNoiseMean + accelBiasMean));
  d2 = fabs(accels.z - (accelNoiseMean - accelBiasMean));
  EXPECT_NEAR(GRAVITY, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
}

TEST_F(ImuTest, EmptyWorldNoiseODE)
{
  Stationary_EmptyWorld_Noise("ode");
}

#ifdef HAVE_BULLET
TEST_F(ImuTest, EmptyWorldNoiseBullet)
{
  Stationary_EmptyWorld_Noise("bullet");
}
#endif

void ImuTest::Stationary_EmptyWorld_Bias(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "imu_model";
  std::string imuSensorName = "imu_sensor";
  math::Pose testPose(math::Vector3(0, 0, 0.05),
      math::Quaternion(0, 0, 0));

  double rateNoiseMean = 0.0;
  double rateNoiseStddev = 0.0;
  double rateBiasMean = 1.0;
  double rateBiasStddev = 0.1;
  double accelNoiseMean = 0.0;
  double accelNoiseStddev = 0.0;
  double accelBiasMean = 10.0;
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
  imu->Update(true);

  math::Vector3 rates = imu->GetAngularVelocity();
  double d1, d2;
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(rates.x - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rates.x - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rates.y - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rates.y - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rates.z - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rates.z - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);

  math::Vector3 accels = imu->GetLinearAcceleration();
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(accels.x - (accelNoiseMean + accelBiasMean));
  d2 = fabs(accels.x - (accelNoiseMean - accelBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accels.y - (accelNoiseMean + accelBiasMean));
  d2 = fabs(accels.y - (accelNoiseMean - accelBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accels.z - (accelNoiseMean + accelBiasMean));
  d2 = fabs(accels.z - (accelNoiseMean - accelBiasMean));
  EXPECT_NEAR(GRAVITY, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
}

TEST_F(ImuTest, EmptyWorldBiasODE)
{
  Stationary_EmptyWorld_Bias("ode");
}

#ifdef HAVE_BULLET
TEST_F(ImuTest, EmptyWorldBiasBullet)
{
  Stationary_EmptyWorld_Bias("bullet");
}
#endif

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
