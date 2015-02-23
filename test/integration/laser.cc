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
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "scans_cmp.h"
#include "helper_physics_generator.hh"

#define LASER_TOL 1e-5
#define DOUBLE_TOL 1e-6

using namespace gazebo;
class LaserTest : public ServerFixture,
                  public testing::WithParamInterface<const char*>
{
  public: void Stationary_EmptyWorld(const std::string &_physicsEngine);
  public: void LaserUnitBox(const std::string &_physicsEngine);
  public: void LaserUnitNoise(const std::string &_physicsEngine);
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
  math::Pose testPose(math::Vector3(0, 0, 0.5),
      math::Quaternion(0, 0, 0));

  SpawnRaySensor(modelName, raySensorName, testPose.pos,
      testPose.rot.GetAsEuler(), hMinAngle, hMaxAngle, minRange, maxRange,
      rangeResolution, samples);

  sensors::RaySensorPtr laser =
    boost::static_pointer_cast<sensors::RaySensor>(
        sensors::SensorManager::Instance()->GetSensor(raySensorName));

  ASSERT_TRUE(laser);
  laser->Init();
  laser->Update(true);

  EXPECT_EQ(640, laser->GetRayCount());
  EXPECT_EQ(640, laser->GetRangeCount());
  EXPECT_NEAR(laser->GetAngleMin().Radian(), -2.27, DOUBLE_TOL);
  EXPECT_NEAR(laser->GetAngleMax().Radian(), 2.27, DOUBLE_TOL);
  EXPECT_NEAR(laser->GetRangeMin(), 0, DOUBLE_TOL);
  EXPECT_NEAR(laser->GetRangeMax(), 10, DOUBLE_TOL);
  EXPECT_NEAR(laser->GetRangeResolution(), 0.01, DOUBLE_TOL);


  for (int i = 0; i < laser->GetRangeCount(); ++i)
  {
    EXPECT_NEAR(10, laser->GetRange(i), DOUBLE_TOL);
  }

  // Spawn a box and test for proper laser scan
  {
    SpawnBox("test_box", math::Vector3(1, 1, 1),
        math::Vector3(2, 0, 0.5), math::Vector3(0, 0, 0));
    common::Time::MSleep(1000);

    laser->Update(true);

    std::vector<double> scan;
    laser->GetRanges(scan);

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
    ASSERT_TRUE(world);

    physics::ModelPtr model = world->GetModel(modelName);

    prevTime = laser->GetLastUpdateTime();
    model->SetWorldPose(math::Pose(0, 0, 1.0, 0, M_PI*0.5, 0));

    double diffMax, diffSum, diffAvg;

    std::vector<double> scan, scan2;

    laser->Update(false);
    for (unsigned int j = 0; j < 5; ++j)
    {
      laser->Update(true);
      laser->GetRanges(scan);
      laser->Update(true);
      laser->GetRanges(scan2);

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
  math::Pose testPose(math::Vector3(0, 0, 0),
      math::Quaternion(0, 0, 0));
  if (_physicsEngine == "bullet" && LIBBULLET_VERSION >= 2.82)
  {
    testPose.pos.z = 0.1;
    gzwarn << "Raising sensor for bullet as workaround for #934" << std::endl;
  }

  SpawnRaySensor(modelName, raySensorName, testPose.pos,
      testPose.rot.GetAsEuler(), hMinAngle, hMaxAngle, minRange, maxRange,
      rangeResolution, samples);

  std::string box01 = "box_01";
  std::string box02 = "box_02";
  std::string box03 = "box_03";

  // box in front of ray sensor
  math::Pose box01Pose(math::Vector3(1, 0, 0.5), math::Quaternion(0, 0, 0));
  // box on the right of ray sensor
  math::Pose box02Pose(math::Vector3(0, -1, 0.5), math::Quaternion(0, 0, 0));
  // box on the left of the ray sensor but out of range
  math::Pose box03Pose(math::Vector3(0, maxRange + 1, 0.5),
      math::Quaternion(0, 0, 0));

  SpawnBox(box01, math::Vector3(1, 1, 1), box01Pose.pos,
      box01Pose.rot.GetAsEuler());

  // box02 is static
  SpawnBox(box02, math::Vector3(1, 1, 1), box02Pose.pos,
      box02Pose.rot.GetAsEuler(), true);

  SpawnBox(box03, math::Vector3(1, 1, 1), box03Pose.pos,
      box03Pose.rot.GetAsEuler());

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::RaySensorPtr raySensor =
    boost::dynamic_pointer_cast<sensors::RaySensor>(sensor);

  raySensor->Init();
  raySensor->Update(true);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  EXPECT_TRUE(world->GetModel(box02)->IsStatic());

  int mid = samples / 2;
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPoint = box01Pose.pos.x - unitBoxSize/2;

  EXPECT_NEAR(raySensor->GetRange(mid), expectedRangeAtMidPoint, LASER_TOL);
  EXPECT_NEAR(raySensor->GetRange(0), expectedRangeAtMidPoint, LASER_TOL);

  // WARNING: for readings of no return, gazebo returns max range rather
  // than +inf. issue #124
  EXPECT_NEAR(raySensor->GetRange(samples-1), maxRange, LASER_TOL);

  // Move all boxes out of range
  world->GetModel(box01)->SetWorldPose(
      math::Pose(math::Vector3(maxRange + 1, 0, 0), math::Quaternion(0, 0, 0)));
  world->GetModel(box02)->SetWorldPose(
      math::Pose(math::Vector3(0, -(maxRange + 1), 0),
      math::Quaternion(0, 0, 0)));
  world->StepWorld(1);
  raySensor->Update(true);

  for (int i = 0; i < raySensor->GetRayCount(); ++i)
  {
    EXPECT_NEAR(raySensor->GetRange(i), maxRange, LASER_TOL);
  }
}

TEST_P(LaserTest, LaserBox)
{
  LaserUnitBox(GetParam());
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
  math::Pose testPose(math::Vector3(0, 0, 0),
      math::Quaternion(0, 0, 0));

  SpawnRaySensor(modelName, raySensorName, testPose.pos,
      testPose.rot.GetAsEuler(), hMinAngle, hMaxAngle, minRange, maxRange,
      rangeResolution, samples,
      noiseType, noiseMean, noiseStdDev);

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::RaySensorPtr raySensor =
    boost::dynamic_pointer_cast<sensors::RaySensor>(sensor);

  EXPECT_TRUE(raySensor != NULL);

  raySensor->Init();
  raySensor->Update(true);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Expect at least one value to be non-max (empty world), and expect the
  // mean to be close to the max+noiseMean
  double total = 0.0;
  bool foundNoise = false;
  for (int i = 0; i < raySensor->GetRayCount(); ++i)
  {
    if (fabs(raySensor->GetRange(i) - maxRange) > LASER_TOL)
      foundNoise = true;
    total += raySensor->GetRange(i);
  }
  EXPECT_TRUE(foundNoise);
  double mean = total / raySensor->GetRayCount();
  // The mean should be well within 3-sigma
  EXPECT_NEAR(mean, maxRange + noiseMean, 3*noiseStdDev);
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
  math::Rand::SetSeed(42);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
