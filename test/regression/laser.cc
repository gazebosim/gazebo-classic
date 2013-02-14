/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "physics/physics.hh"
#include "sensors/sensors.hh"
#include "common/common.hh"
#include "scans_cmp.h"

#define LASER_TOL 1e-2
#define DOUBLE_TOL 1e-6

using namespace gazebo;
class LaserTest : public ServerFixture
{
  public: void Stationary_EmptyWorld(const std::string &_physicsEngine);
  public: void LaserUnitCylinder(const std::string &_physicsEngine);
};

void LaserTest::Stationary_EmptyWorld(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", false, _physicsEngine);
  std::ostringstream laserModel;
    laserModel << "<sdf version='" << SDF_VERSION << "'>"
      "<model name='box'>"
        "<static>true</static>"
        "<link name='link'>"
        "<pose>0 0 0.5 0 0 0</pose>"
        "<sensor name='laser' type='ray'>"
          "<always_on>true</always_on>"
          "<update_rate>10</update_rate>"
          "<visualize>true</visualize>"
          "<topic>~/laser_scan</topic>"
          "<ray>"
            "<scan>"
              "<horizontal>"
                "<samples>640</samples>"
                "<resolution>1</resolution>"
                "<min_angle>-2.27</min_angle>"
                "<max_angle>2.27</max_angle>"
              "</horizontal>"
            "</scan>"
            "<range>"
              "<min>0.0</min>"
              "<max>10</max>"
              "<resolution>0.01</resolution>"
            "</range>"
          "</ray>"
        "</sensor>"
      "</link>"
    "</model>"
  "</sdf>";

  SpawnSDF(laserModel.str());
  while (!HasEntity("box"))
    common::Time::MSleep(10);

  sensors::RaySensorPtr laser =
    boost::shared_static_cast<sensors::RaySensor>(
        sensors::SensorManager::Instance()->GetSensor(
          "default::box::link::laser"));
  EXPECT_TRUE(laser);
  laser->Update(true);
  while (laser->GetRange(0) == 0)
    common::Time::MSleep(100);


  EXPECT_EQ(640, laser->GetRayCount());
  EXPECT_EQ(640, laser->GetRangeCount());
  EXPECT_NEAR(laser->GetAngleMin().Radian(), -2.27, DOUBLE_TOL);
  EXPECT_NEAR(laser->GetAngleMax().Radian(), 2.27, DOUBLE_TOL);
  EXPECT_NEAR(laser->GetRangeMin(), 0, DOUBLE_TOL);
  EXPECT_NEAR(laser->GetRangeMax(), 10, DOUBLE_TOL);
  EXPECT_NEAR(laser->GetRangeResolution(), 0.01, DOUBLE_TOL);

  for (int i = 0; i < laser->GetRangeCount(); ++i)
  {
    EXPECT_EQ(10, laser->GetRange(i));
  }

  // Spawn a box and test for proper laser scan
  {
    SpawnBox("test_box", math::Vector3(1, 1, 1),
        math::Vector3(2, 0, 0.5), math::Vector3(0, 0, 0));
    common::Time::MSleep(1000);

    laser->Update(true);

    double diffMax, diffSum, diffAvg;
    std::vector<double> scan;
    laser->GetRanges(scan);

    DoubleCompare(box_scan, &scan[0], 640, diffMax, diffSum, diffAvg);
    EXPECT_LT(diffMax, 1e-6);
    EXPECT_LT(diffSum, 1e-5);
    EXPECT_LT(diffAvg, 1e-6);

    // This line will print the current scan. Use this to generate
    // a new test scan sample
    // PrintScan("box_scan", &scan[0], 640);
  }

  // Move the laser to point down on the ground plane,
  {
    common::Time prevTime;
    physics::WorldPtr world = physics::get_world("default");
    EXPECT_TRUE(world);

    physics::ModelPtr model = world->GetModel("box");

    prevTime = laser->GetLastUpdateTime();
    model->SetWorldPose(math::Pose(0, 0, 1.0, 0, M_PI*0.5, 0));
    while (laser->GetLastUpdateTime() <= prevTime)
      common::Time::MSleep(10);

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

    DoubleCompare(plane_scan, &scan[0], 640, diffMax, diffSum, diffAvg);
    EXPECT_LT(diffMax, 1e-6);
    EXPECT_LT(diffSum, 1e-6);
    EXPECT_LT(diffAvg, 1e-6);

    // This line will print the current scan. Use this to generate
    // a new test scan sample
    // PrintScan("plane_scan", &scan[0], 640);
  }
}

TEST_F(LaserTest, EmptyWorldODE)
{
  Stationary_EmptyWorld("ode");
}

TEST_F(LaserTest, EmptyWorldBullet)
{
  Stationary_EmptyWorld("bullet");
}

void LaserTest::LaserUnitCylinder(const std::string &_physicsEngine)
{
  // Test ray sensor with 3 cylinders in the world.
  // First place 2 of 3 cylinders within range and verify range values,
  // then move all 3 cylinder out of range and verify range values

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

  SpawnRaySensor(modelName, raySensorName, testPose.pos,
      testPose.rot.GetAsEuler(), hMinAngle, hMaxAngle, minRange, maxRange,
      rangeResolution, samples);

  std::string cylinder01 = "cylinder_01";
  std::string cylinder02 = "cylinder_02";
  std::string cylinder03 = "cylinder_03";

  // cylinder in front of ray sensor
  math::Pose cylinder01Pose(math::Vector3(1, 0, 0),
      math::Quaternion(0, 0, 0));
  // cylinder on the right of ray sensor
  math::Pose cylinder02Pose(math::Vector3(0, -1, 0),
      math::Quaternion(0, 0, 0));
  // cylinder on the left of the ray sensor but out of range
  math::Pose cylinder03Pose(math::Vector3(0, maxRange + 1, 0),
      math::Quaternion(0, 0, 0));

  SpawnCylinder(cylinder01, cylinder01Pose.pos,
      cylinder01Pose.rot.GetAsEuler());

  SpawnCylinder(cylinder02, cylinder02Pose.pos,
      cylinder02Pose.rot.GetAsEuler());

  SpawnCylinder(cylinder03, cylinder03Pose.pos,
      cylinder03Pose.rot.GetAsEuler());

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::RaySensorPtr raySensor =
    boost::shared_dynamic_cast<sensors::RaySensor>(sensor);

  raySensor->Init();
  raySensor->Update(true);

  int mid = samples / 2;
  double unitCylinderRadius = 0.5;
  double expectedRangeAtMidPoint = cylinder01Pose.pos.x - unitCylinderRadius;

  // WARNING: gazebo returns distance to object from min range
  // issue #503
  expectedRangeAtMidPoint -= minRange;

  EXPECT_NEAR(raySensor->GetRange(mid), expectedRangeAtMidPoint, LASER_TOL);
  EXPECT_NEAR(raySensor->GetRange(0), expectedRangeAtMidPoint, LASER_TOL);

  // WARNING: for readings of no return, gazebo returns max range rather
  // than +inf. issue #124
  EXPECT_NEAR(raySensor->GetRange(samples-1), maxRange, LASER_TOL);

  // Move all cylinders out of range
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world !=NULL);

  world->GetModel(cylinder01)->SetWorldPose(
        math::Pose(math::Vector3(maxRange + 1, 0, 0),
        math::Quaternion(0, 0, 0)));
  world->GetModel(cylinder02)->SetWorldPose(
        math::Pose(math::Vector3(0, -(maxRange + 1), 0),
        math::Quaternion(0, 0, 0)));
  world->StepWorld(1);
  raySensor->Update(true);

  for (int i = 0; i < raySensor->GetRayCount(); ++i)
  {
    EXPECT_NEAR(raySensor->GetRange(i), maxRange, LASER_TOL);
  }
}

TEST_F(LaserTest, LaserCylindereODE)
{
  LaserUnitCylinder("ode");
}

TEST_F(LaserTest, LaserCylinderBullet)
{
  LaserUnitCylinder("bullet");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
