/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <ignition/math/Helpers.hh>
#include <ignition/math/Rand.hh>

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "scans_cmp.h"
#include "gazebo/test/helper_physics_generator.hh"

#define LASER_TOL 1e-5
#define DOUBLE_TOL 1e-6

using namespace gazebo;
class LaserTest : public ServerFixture,
                  public testing::WithParamInterface<const char*>
{
  public: void Stationary_EmptyWorld(const std::string &_physicsEngine);
  public: void GroundPlane(const std::string &_physicsEngine);
  public: void LaserUnitBox(const std::string &_physicsEngine);
  public: void LaserUnitNoise(const std::string &_physicsEngine);
  public: void LaserVertical(const std::string &_physicsEngine);
  public: void LaserScanResolution(const std::string &_physicsEngine);
};

void LaserTest::Stationary_EmptyWorld(const std::string &_physicsEngine)
{
  if (_physicsEngine == "dart")
  {
    gzerr << "Abort test since dart does not support ray shape, "
          << "Please see issue #911. "
          << "(https://bitbucket.org/osrf/gazebo/issue/911).\n";
    return;
  }

  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "ray_model";
  std::string raySensorName = "ray_sensor";
  double hMinAngle = -2.27;
  double hMaxAngle = 2.27;
  double minRange = 0.0;
  double maxRange = 10.0;
  double rangeResolution = 0.01;
  unsigned int samples = 640;
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0, 0, 0.5),
      ignition::math::Quaterniond::Identity);

  SpawnRaySensor(modelName, raySensorName, testPose.Pos(),
      testPose.Rot().Euler(), hMinAngle, hMaxAngle, 0, 0,
      minRange, maxRange, rangeResolution, samples, 1, 1, 1);

  sensors::RaySensorPtr laser =
    std::static_pointer_cast<sensors::RaySensor>(
        sensors::SensorManager::Instance()->GetSensor(raySensorName));

  ASSERT_TRUE(laser != NULL);
  laser->Init();
  laser->Update(true);

  EXPECT_EQ(640, laser->RayCount());
  EXPECT_EQ(640, laser->RangeCount());
  EXPECT_NEAR(laser->AngleMin().Radian(), -2.27, DOUBLE_TOL);
  EXPECT_NEAR(laser->AngleMax().Radian(), 2.27, DOUBLE_TOL);
  EXPECT_NEAR(laser->RangeMin(), 0, DOUBLE_TOL);
  EXPECT_NEAR(laser->RangeMax(), 10, DOUBLE_TOL);
  EXPECT_NEAR(laser->RangeResolution(), 0.01, DOUBLE_TOL);

  for (int i = 0; i < laser->RangeCount(); ++i)
  {
    EXPECT_DOUBLE_EQ(ignition::math::INF_D, laser->Range(i));
  }

  // Spawn a box and test for proper laser scan
  {
    SpawnBox("test_box", ignition::math::Vector3d(1, 1, 1),
        ignition::math::Vector3d(2, 0, 0.5),
        ignition::math::Vector3d::Zero);
    common::Time::MSleep(1000);

    laser->Update(true);

    std::vector<double> scan;
    laser->Ranges(scan);

    // run test against pre-recorded range data only in ode
    if (_physicsEngine == "ode")
    {
      double diffMax, diffSum, diffAvg;
      DoubleCompare(box_scan, &scan[0], 640, diffMax, diffSum, diffAvg);
      EXPECT_LT(diffMax, 2e-6);
      EXPECT_LT(diffSum, 1e-4);
      EXPECT_LT(diffAvg, 2e-6);
    }

    // This line will print the current scan. Use this to generate
    // a new test scan sample
    // PrintScan("box_scan", &scan[0], 640);
  }

  // Move the laser to point down on the ground plane,
  {
    common::Time prevTime;
    physics::WorldPtr world = physics::get_world("default");
    ASSERT_TRUE(world != NULL);

    physics::ModelPtr model = world->ModelByName(modelName);

    prevTime = laser->LastUpdateTime();
    model->SetWorldPose(ignition::math::Pose3d(0, 0, 1.0, 0, M_PI*0.5, 0));

    double diffMax, diffSum, diffAvg;

    std::vector<double> scan, scan2;

    laser->Update(false);
    for (unsigned int j = 0; j < 5; ++j)
    {
      laser->Update(true);
      laser->Ranges(scan);
      laser->Update(true);
      laser->Ranges(scan2);

      DoubleCompare(&scan[0], &scan2[0], 640, diffMax, diffSum, diffAvg);
      EXPECT_LT(diffMax, 1e-6);
      EXPECT_LT(diffSum, 1e-6);
      EXPECT_LT(diffAvg, 1e-6);
    }
    laser->Update(true);

    // run test against pre-recorded range data only in ode
    if (_physicsEngine == "ode")
    {
      DoubleCompare(plane_scan, &scan[0], 640, diffMax, diffSum, diffAvg);
      EXPECT_LT(diffMax, 1e-6);
      EXPECT_LT(diffSum, 1e-6);
      EXPECT_LT(diffAvg, 1e-6);
    }

    // This line will print the current scan. Use this to generate
    // a new test scan sample
    // PrintScan("plane_scan", &scan[0], 640);
  }
}

TEST_P(LaserTest, EmptyWorld)
{
  Stationary_EmptyWorld(GetParam());
}

void LaserTest::LaserUnitBox(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody")
  {
    gzerr << "Abort test since simbody does not support ray sensor, "
          << "Please see issue #867.\n";
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr << "Abort test since dart does not support ray shape and sensor, "
          << "Please see issue #911. "
          << "(https://bitbucket.org/osrf/gazebo/issue/911).\n";
    return;
  }

  // Test ray sensor with 3 boxes in the world.
  // First place 2 of 3 boxes within range and verify range values, one of them
  // being a static model to verify collision filtering is working,
  // then move all 3 boxes out of range and verify range values

  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "ray_model";
  std::string raySensorName = "ray_sensor";
  double hMinAngle = -M_PI/2.0;
  double hMaxAngle = M_PI/2.0;
  double minRange = 0.1;
  double maxRange = 5.0;
  double rangeResolution = 0.02;
  unsigned int samples = 320;
  ignition::math::Pose3d testPose(ignition::math::Vector3d::Zero,
      ignition::math::Quaterniond::Identity);
  if (_physicsEngine == "bullet" && LIBBULLET_VERSION >= 2.82)
  {
    testPose.Pos().Z() = 0.1;
    gzwarn << "Raising sensor for bullet as workaround for #934" << std::endl;
  }

  SpawnRaySensor(modelName, raySensorName, testPose.Pos(),
      testPose.Rot().Euler(), hMinAngle, hMaxAngle, 0, 0, minRange, maxRange,
      rangeResolution, samples, 1, 1, 1);

  std::string box01 = "box_01";
  std::string box02 = "box_02";
  std::string box03 = "box_03";

  // box in front of ray sensor
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(1, 0, 0.5),
                                   ignition::math::Quaterniond::Identity);
  // box on the right of ray sensor
  ignition::math::Pose3d box02Pose(ignition::math::Vector3d(0, -1, 0.5),
                                   ignition::math::Quaterniond::Identity);
  // box on the left of the ray sensor but out of range
  ignition::math::Pose3d box03Pose(
      ignition::math::Vector3d(0, maxRange + 1, 0.5),
      ignition::math::Quaterniond::Identity);

  SpawnBox(box01, ignition::math::Vector3d(1, 1, 1), box01Pose.Pos(),
      box01Pose.Rot().Euler());

  // box02 is static
  SpawnBox(box02, ignition::math::Vector3d(1, 1, 1), box02Pose.Pos(),
      box02Pose.Rot().Euler(), true);

  SpawnBox(box03, ignition::math::Vector3d(1, 1, 1), box03Pose.Pos(),
      box03Pose.Rot().Euler());

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::RaySensorPtr raySensor =
    std::dynamic_pointer_cast<sensors::RaySensor>(sensor);

  raySensor->Init();
  raySensor->Update(true);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  EXPECT_TRUE(world->ModelByName(box02)->IsStatic());

  int mid = samples / 2;
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPoint = box01Pose.Pos().X() - unitBoxSize/2;

  EXPECT_NEAR(raySensor->Range(mid), expectedRangeAtMidPoint, LASER_TOL);
  EXPECT_NEAR(raySensor->Range(0), expectedRangeAtMidPoint, LASER_TOL);

  EXPECT_DOUBLE_EQ(raySensor->Range(samples-1), ignition::math::INF_D);

  // Move all boxes out of range
  world->ModelByName(box01)->SetWorldPose(
      ignition::math::Pose3d(
        ignition::math::Vector3d(maxRange + 1, 0, 0),
        ignition::math::Quaterniond::Identity));
  world->ModelByName(box02)->SetWorldPose(
      ignition::math::Pose3d(ignition::math::Vector3d(0, -(maxRange + 1), 0),
      ignition::math::Quaterniond::Identity));
  world->Step(1);
  raySensor->Update(true);

  for (int i = 0; i < raySensor->RayCount(); ++i)
  {
    EXPECT_DOUBLE_EQ(raySensor->Range(i), ignition::math::INF_D);
  }
}

TEST_P(LaserTest, LaserBox)
{
  LaserUnitBox(GetParam());
}

void LaserTest::LaserVertical(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody")
  {
    gzerr << "Abort test since simbody does not support ray sensor, "
          << "Please see issue #867.\n";
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr << "Abort test since dart does not support ray shape and sensor, "
          << "Please see issue #911. "
          << "(https://bitbucket.org/osrf/gazebo/issue/911).\n";
    return;
  }

  // Test a ray sensor that has a vertical range component.
  // Place a box within range and verify range values,
  // then move the box out of range and verify range values

  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "ray_model";
  std::string raySensorName = "ray_sensor";
  double hMinAngle = -M_PI/2.0;
  double hMaxAngle = M_PI/2.0;
  double vMinAngle = -0.1;
  double vMaxAngle = 0.1;
  double minRange = 0.0;
  double maxRange = 5.0;
  double rangeResolution = 0.02;
  unsigned int samples = 640;
  unsigned int vSamples = 3;
  double vAngleStep = (vMaxAngle - vMinAngle) / (vSamples-1);
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0.25, 0, 0.5),
      ignition::math::Quaterniond::Identity);

  SpawnRaySensor(modelName, raySensorName, testPose.Pos(),
      testPose.Rot().Euler(), hMinAngle, hMaxAngle, vMinAngle, vMaxAngle,
      minRange, maxRange, rangeResolution, samples, vSamples, 1, 1);

  std::string box01 = "box_01";

  // box in front of ray sensor
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(1, 0, 0.5),
                                   ignition::math::Quaterniond::Identity);
  SpawnBox(box01, ignition::math::Vector3d(1, 1, 1), box01Pose.Pos(),
      box01Pose.Rot().Euler());

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::RaySensorPtr raySensor =
    std::dynamic_pointer_cast<sensors::RaySensor>(sensor);

  raySensor->Init();
  raySensor->Update(true);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  unsigned int mid = samples / 2;
  double unitBoxSize = 1.0;

  double angleStep = vMinAngle;
  // all vertical laser planes should sense box
  for (unsigned int i = 0; i < vSamples; ++i)
  {
    double expectedRangeAtMidPoint = box01Pose.Pos().X() - unitBoxSize/2
        - testPose.Pos().X();
    expectedRangeAtMidPoint = expectedRangeAtMidPoint / cos(angleStep);

    EXPECT_NEAR(raySensor->Range(i*samples + mid),
        expectedRangeAtMidPoint, LASER_TOL);

    angleStep += vAngleStep;

    EXPECT_DOUBLE_EQ(raySensor->Range(i*samples), ignition::math::INF_D);
    EXPECT_DOUBLE_EQ(raySensor->Range(i*samples + samples-1),
                     ignition::math::INF_D);
  }

  // Move box out of range
  world->ModelByName(box01)->SetWorldPose(
      ignition::math::Pose3d(
        ignition::math::Vector3d(maxRange + 1, 0, 0),
        ignition::math::Quaterniond::Identity));

  world->Step(1);
  raySensor->Update(true);

  for (int j = 0; j < raySensor->VerticalRayCount(); ++j)
  {
    for (int i = 0; i < raySensor->RayCount(); ++i)
    {
      EXPECT_DOUBLE_EQ(raySensor->Range(j*raySensor->RayCount() + i),
          ignition::math::INF_D);
    }
  }
}

TEST_P(LaserTest, LaserVertical)
{
  LaserVertical(GetParam());
}

void LaserTest::LaserScanResolution(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody")
  {
    gzerr << "Abort test since simbody does not support ray sensor, "
          << "Please see issue #867.\n";
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr << "Abort test since dart does not support ray shape and sensor, "
          << "Please see issue #911. "
          << "(https://bitbucket.org/osrf/gazebo/issue/911).\n";
    return;
  }

  // Test ray sensor scan resolution.
  // Orient the sensor to face downwards and verify that the interpolated
  // range values all intersect with ground plane at z = 0;

  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "ray_model";
  std::string raySensorName = "ray_sensor";
  // use asymmetric horizontal angles to make test more difficult
  double hMinAngle = -M_PI/4.0;
  double hMaxAngle = M_PI/8.0;
  double vMinAngle = -0.1;
  double vMaxAngle = 0.1;
  double vMidAngle = M_PI/2.0;
  double minRange = 0.0;
  double maxRange = 5.0;
  double rangeResolution = 0.02;
  unsigned int hSamples = 641;
  unsigned int vSamples = 5;
  double hResolution = 3;
  double vResolution = 2;
  double hAngleStep = (hMaxAngle - hMinAngle) / (hSamples*hResolution-1);
  double vAngleStep = (vMaxAngle - vMinAngle) / (vSamples*vResolution-1);
  double z0 = 0.5;
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0.25, 0, z0),
      ignition::math::Quaterniond(0, vMidAngle, 0));

  SpawnRaySensor(modelName, raySensorName, testPose.Pos(),
      testPose.Rot().Euler(), hMinAngle, hMaxAngle, vMinAngle, vMaxAngle,
      minRange, maxRange, rangeResolution, hSamples, vSamples,
      hResolution, vResolution);

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::RaySensorPtr raySensor =
    std::dynamic_pointer_cast<sensors::RaySensor>(sensor);

  raySensor->Init();
  raySensor->Update(true);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  unsigned int h, v;

  for (v = 0; v < vSamples*vResolution; ++v)
  {
    for (h = 0; h < hSamples*hResolution; ++h)
    {
      // pitch angle
      double p = vMinAngle + v*vAngleStep;
      // yaw angle
      double y = hMinAngle + h*hAngleStep;
      double R = raySensor->Range(v*hSamples*hResolution + h);

      ignition::math::Quaterniond rot(0.0, -p, y);
      ignition::math::Vector3d axis =
        testPose.Rot() * rot * ignition::math::Vector3d::UnitX;
      ignition::math::Vector3d intersection = (axis * R) + testPose.Pos();
      EXPECT_NEAR(intersection.Z(), 0.0, rangeResolution);
    }
  }
}

TEST_P(LaserTest, LaserScanResolution)
{
  LaserScanResolution(GetParam());
}

void LaserTest::GroundPlane(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody")
  {
    gzerr << "Abort test since simbody does not support ray sensor, "
          << "Please see issue #867.\n";
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr << "Abort test since dart does not support ray shape and sensor, "
          << "Please see issue #911. "
          << "(https://bitbucket.org/osrf/gazebo/issue/911).\n";
    return;
  }

  // Test a ray sensor that has a vertical range component.
  // Aim the sensor toward the ground and verify correct ranges.

  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "ray_model";
  std::string raySensorName = "ray_sensor";
  // use asymmetric horizontal angles to make test more difficult
  double hMinAngle = -M_PI/4.0;
  double hMaxAngle = M_PI/8.0;
  double vMinAngle = -0.1;
  double vMaxAngle = 0.1;
  double vMidAngle = 0.3;
  double minRange = 0.0;
  double maxRange = 5.0;
  double rangeResolution = 0.02;
  unsigned int hSamples = 641;
  unsigned int vSamples = 5;
  double hAngleStep = (hMaxAngle - hMinAngle) / (hSamples-1);
  double vAngleStep = (vMaxAngle - vMinAngle) / (vSamples-1);
  double z0 = 0.5;
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0.25, 0, z0),
      ignition::math::Quaterniond(0, vMidAngle, 0));

  SpawnRaySensor(modelName, raySensorName, testPose.Pos(),
      testPose.Rot().Euler(), hMinAngle, hMaxAngle, vMinAngle, vMaxAngle,
      minRange, maxRange, rangeResolution, hSamples, vSamples, 1, 1);

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::RaySensorPtr raySensor =
    std::dynamic_pointer_cast<sensors::RaySensor>(sensor);

  raySensor->Init();
  raySensor->Update(true);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  unsigned int h, v;

  for (v = 0; v < vSamples; ++v)
  {
    for (h = 0; h < hSamples; ++h)
    {
      // pitch angle
      double p = vMinAngle + v*vAngleStep;
      // yaw angle
      double y = hMinAngle + h*hAngleStep;
      double R = raySensor->Range(v*hSamples + h);

      ignition::math::Quaterniond rot(0.0, -p, y);
      ignition::math::Vector3d axis =
        testPose.Rot() * rot * ignition::math::Vector3d::UnitX;
      ignition::math::Vector3d intersection = (axis * R) + testPose.Pos();
      EXPECT_NEAR(intersection.Z(), 0.0, rangeResolution);
    }
  }
}

TEST_P(LaserTest, GroundPlane)
{
  GroundPlane(GetParam());
}

void LaserTest::LaserUnitNoise(const std::string &_physicsEngine)
{
  if (_physicsEngine == "dart")
  {
    gzerr << "Abort test since dart does not support ray shape and sensor, "
          << "Please see issue #911. "
          << "(https://bitbucket.org/osrf/gazebo/issue/911).\n";
    return;
  }

  // Test ray sensor with noise applied

  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "ray_model";
  std::string raySensorName = "ray_sensor";
  double hMinAngle = -M_PI/2.0;
  double hMaxAngle = M_PI/2.0;
  double minRange = 0.0;
  double maxRange = 5.0;
  double rangeResolution = 0.02;
  unsigned int samples = 320;
  std::string noiseType = "gaussian";
  // Give negative bias so that we can see the effect (positive bias
  // would be removed by clamp(minRange,maxRange).
  double noiseMean = -1.0;
  double noiseStdDev = 0.01;
  ignition::math::Pose3d testPose(ignition::math::Vector3d::Zero,
      ignition::math::Quaterniond::Identity);

  SpawnRaySensor(modelName, raySensorName, testPose.Pos(),
      testPose.Rot().Euler(), hMinAngle, hMaxAngle, 0, 0,
      minRange, maxRange, rangeResolution, samples, 1, 1, 1,
      noiseType, noiseMean, noiseStdDev);

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::RaySensorPtr raySensor =
    std::dynamic_pointer_cast<sensors::RaySensor>(sensor);

  EXPECT_TRUE(raySensor != NULL);

  raySensor->Init();
  raySensor->Update(true);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  bool foundNoise = false;
  for (int i = 0; i < raySensor->RayCount(); ++i)
  {
    if (fabs(raySensor->Range(i) - maxRange) > LASER_TOL)
      foundNoise = true;
  }
  EXPECT_TRUE(foundNoise);
}

TEST_P(LaserTest, LaserNoise)
{
  LaserUnitNoise(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, LaserTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ignition::math::Rand::Seed(42);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
