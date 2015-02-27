/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/common/Console.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/sensors/SensorsIface.hh"
#include "gazebo/sensors/sensors.hh"

#define LASER_TOL 1e-5
#define DOUBLE_TOL 1e-6

using namespace gazebo;
class GPURaySensorTest : public ServerFixture
{
};

void OnNewLaserFrame(int *_scanCounter, float *_scanDest,
                  const float *_scan,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth,
                  const std::string &/*_format*/)
{
  memcpy(_scanDest, _scan, _width * _height * _depth);
  *_scanCounter += 1;
}

/////////////////////////////////////////////////
/// \brief Test GPU ray sensor range values,
/// Adapted from LaserUnitBox test in laser.cc
TEST_F(GPURaySensorTest, LaserUnitBox)
{
  // Test GPU ray sensors with 3 boxes in the world.
  // First GPU ray sensor at identity orientation, second at 90 degree roll
  // First place 2 of 3 boxes within range and verify range values.
  // then move all 3 boxes out of range and verify range values
  Load("worlds/empty_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run gpu laser test\n";
    return;
  }

  std::string modelName = "gpu_ray_model";
  std::string raySensorName = "gpu_ray_sensor";
  double hMinAngle = -M_PI/2.0;
  double hMaxAngle = M_PI/2.0;
  double minRange = 0.1;
  double maxRange = 5.0;
  double rangeResolution = 0.02;
  unsigned int samples = 320;
  math::Pose testPose(math::Vector3(0, 0, 0.1),
      math::Quaternion(0, 0, 0));

  // Spawn another gpu ray sensor at 90 degree roll
  std::string modelName2 = "gpu_ray_model_roll";
  std::string raySensorName2 = "gpu_ray_sensor_roll";
  math::Pose testPose2(math::Vector3(0, 0, 0.1),
      math::Quaternion(M_PI/2.0, 0, 0));

  SpawnGpuRaySensor(modelName, raySensorName, testPose.pos,
      testPose.rot.GetAsEuler(), hMinAngle, hMaxAngle, minRange, maxRange,
      rangeResolution, samples);

  SpawnGpuRaySensor(modelName2, raySensorName2, testPose2.pos,
      testPose2.rot.GetAsEuler(), hMinAngle, hMaxAngle, minRange, maxRange,
      rangeResolution, samples);

  std::string box01 = "box_01";
  std::string box02 = "box_02";
  std::string box03 = "box_03";

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->GetPhysicsEngine()->SetGravity(math::Vector3(0, 0, 0));

  // box in front of ray sensor 1 and 2
  math::Pose box01Pose(math::Vector3(1, 0, 0.5), math::Quaternion(0, 0, 0));
  // box on the right of ray sensor 1
  math::Pose box02Pose(math::Vector3(0, -1, 0.5), math::Quaternion(0, 0, 0));
  // box on the left of the ray sensor 1 but out of range
  math::Pose box03Pose(math::Vector3(0, maxRange + 1, 0.5),
      math::Quaternion(0, 0, 0));

  SpawnBox(box01, math::Vector3(1, 1, 1), box01Pose.pos,
      box01Pose.rot.GetAsEuler());

  SpawnBox(box02, math::Vector3(1, 1, 1), box02Pose.pos,
      box02Pose.rot.GetAsEuler());

  SpawnBox(box03, math::Vector3(1, 1, 1), box03Pose.pos,
      box03Pose.rot.GetAsEuler());

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::GpuRaySensorPtr raySensor =
    boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensor);

  sensors::SensorPtr sensor2 = sensors::get_sensor(raySensorName2);
  sensors::GpuRaySensorPtr raySensor2 =
    boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensor2);

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(raySensor != NULL);
  EXPECT_TRUE(raySensor2 != NULL);

  raySensor->SetActive(true);
  raySensor2->SetActive(true);

  // Verify ray sensor 1 range readings
  // listen to new laser frames
  float *scan = new float[raySensor->GetRayCount()
      * raySensor->GetVerticalRayCount() * 3];
  int scanCount = 0;
  event::ConnectionPtr c =
    raySensor->ConnectNewLaserFrame(
        boost::bind(&::OnNewLaserFrame, &scanCount, scan,
          _1, _2, _3, _4, _5));

  // wait for a few laser scans
  int i = 0;
  while (scanCount < 10 && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  int mid = samples / 2;
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPoint = box01Pose.pos.x - unitBoxSize/2;

  // ray sensor 1 should see box01 and box02
  EXPECT_NEAR(raySensor->GetRange(mid), expectedRangeAtMidPoint, LASER_TOL);
  EXPECT_NEAR(raySensor->GetRange(0), expectedRangeAtMidPoint, LASER_TOL);

  // WARNING: for readings of no return, gazebo returns max range rather
  // than +inf. issue #124
  EXPECT_NEAR(raySensor->GetRange(samples-1), maxRange, LASER_TOL);

  // Verify ray sensor 2 range readings
  // listen to new laser frames
  float *scan2 = new float[raySensor2->GetRayCount()
      * raySensor2->GetVerticalRayCount() * 3];
  int scanCount2 = 0;
  event::ConnectionPtr c2 =
    raySensor->ConnectNewLaserFrame(
        boost::bind(&::OnNewLaserFrame, &scanCount2, scan2,
          _1, _2, _3, _4, _5));

  // wait for a few laser scans
  i = 0;
  scanCount2 = 0;
  while (scanCount2 < 10 && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  // Only box01 should be visible to ray sensor 2
  EXPECT_NEAR(raySensor2->GetRange(mid), expectedRangeAtMidPoint, LASER_TOL);
  EXPECT_NEAR(raySensor2->GetRange(0), maxRange, LASER_TOL);
  EXPECT_NEAR(raySensor->GetRange(samples-1), maxRange, LASER_TOL);

  // Move all boxes out of range
  world->GetModel(box01)->SetWorldPose(
      math::Pose(math::Vector3(maxRange + 1, 0, 0), math::Quaternion(0, 0, 0)));
  world->GetModel(box02)->SetWorldPose(
      math::Pose(math::Vector3(0, -(maxRange + 1), 0),
      math::Quaternion(0, 0, 0)));

  // wait for a few more laser scans
  i = 0;
  scanCount = 0;
  scanCount2 = 0;
  while ((scanCount < 10 ||scanCount2 < 10) && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  for (i = 0; i < raySensor->GetRayCount(); ++i)
    EXPECT_NEAR(raySensor->GetRange(i), maxRange, LASER_TOL);

  for (i = 0; i < raySensor->GetRayCount(); ++i)
    EXPECT_NEAR(raySensor2->GetRange(i), maxRange, LASER_TOL);

  raySensor->DisconnectNewLaserFrame(c);
  raySensor2->DisconnectNewLaserFrame(c2);

  delete [] scan;
  delete [] scan2;
}

/////////////////////////////////////////////////
/// \brief Spawn multiple GPU ray sensors with same name.
/// Verify that it doesn't crash.
TEST_F(GPURaySensorTest, NameCollision)
{
  // Test GPU ray sensors with 3 boxes in the world.
  // First GPU ray sensor at identity orientation, second at 90 degree roll
  // First place 2 of 3 boxes within range and verify range values.
  // then move all 3 boxes out of range and verify range values
  Load("worlds/empty_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run gpu laser test\n";
    return;
  }

  std::string modelName = "gpu_ray_model";
  std::string raySensorName = "gpu_ray_sensor";
  double hMinAngle = -M_PI/2.0;
  double hMaxAngle = M_PI/2.0;
  double minRange = 0.1;
  double maxRange = 5.0;
  double rangeResolution = 0.02;
  unsigned int samples = 320;
  math::Pose testPose(math::Vector3(0, 0, 0.1),
      math::Quaternion(0, 0, 0));

  // Spawn another gpu ray sensor at 90 degree roll
  std::string modelName2 = "gpu_ray_model_roll";
  std::string raySensorName2 = "gpu_ray_sensor";
  math::Pose testPose2(math::Vector3(0, 0, 0.1),
      math::Quaternion(M_PI/2.0, 0, 0));

  SpawnGpuRaySensor(modelName, raySensorName, testPose.pos,
      testPose.rot.GetAsEuler(), hMinAngle, hMaxAngle, minRange, maxRange,
      rangeResolution, samples);

  SpawnGpuRaySensor(modelName2, raySensorName2, testPose2.pos,
      testPose2.rot.GetAsEuler(), hMinAngle, hMaxAngle, minRange, maxRange,
      rangeResolution, samples);

  std::string box01 = "box_01";
  std::string box02 = "box_02";
  std::string box03 = "box_03";

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->GetPhysicsEngine()->SetGravity(math::Vector3(0, 0, 0));

  // box in front of ray sensor 1 and 2
  math::Pose box01Pose(math::Vector3(1, 0, 0.5), math::Quaternion(0, 0, 0));
  // box on the right of ray sensor 1
  math::Pose box02Pose(math::Vector3(0, -1, 0.5), math::Quaternion(0, 0, 0));
  // box on the left of the ray sensor 1 but out of range
  math::Pose box03Pose(math::Vector3(0, maxRange + 1, 0.5),
      math::Quaternion(0, 0, 0));

  SpawnBox(box01, math::Vector3(1, 1, 1), box01Pose.pos,
      box01Pose.rot.GetAsEuler());

  SpawnBox(box02, math::Vector3(1, 1, 1), box02Pose.pos,
      box02Pose.rot.GetAsEuler());

  SpawnBox(box03, math::Vector3(1, 1, 1), box03Pose.pos,
      box03Pose.rot.GetAsEuler());

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::GpuRaySensorPtr raySensor =
    boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensor);

  sensors::SensorPtr sensor2 = sensors::get_sensor(raySensorName2);
  sensors::GpuRaySensorPtr raySensor2 =
    boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensor2);

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(raySensor != NULL);
  EXPECT_TRUE(raySensor2 != NULL);
}

/////////////////////////////////////////////////
/// \brief Test GPU ray sensor interaction with terrain
TEST_F(GPURaySensorTest, Heightmap)
{
  Load("worlds/gpu_laser_heightmap.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run gpu laser test\n";
    return;
  }

  // Get a pointer to the gpu laser sensor
  std::string gpuLaserName = "gpu_laser_sensor";
  int t = 0;
  while (sensors::get_sensor(gpuLaserName) == NULL && t < 100)
  {
    common::Time::MSleep(100);
    ++t;
  }
  ASSERT_LT(t, 100);
  sensors::SensorPtr sensor = sensors::get_sensor(gpuLaserName);
  sensors::GpuRaySensorPtr raySensor =
    boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensor);

  EXPECT_TRUE(raySensor != NULL);

  // listen to new laser frames
  float *scan = new float[raySensor->GetRayCount()
      * raySensor->GetVerticalRayCount() * 3];
  int scanCount = 0;
  event::ConnectionPtr c =
    raySensor->ConnectNewLaserFrame(
        boost::bind(&::OnNewLaserFrame, &scanCount, scan,
          _1, _2, _3, _4, _5));

  // wait for a few laser scans
  int i = 0;
  while (scanCount < 10 && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  // Verify initial laser range readings. Nothing should be intersecting
  double maxRange = 10;
  EXPECT_NEAR(raySensor->GetRangeMax(), maxRange, LASER_TOL);

  for (i = 0; i < raySensor->GetRayCount(); ++i)
    EXPECT_NEAR(raySensor->GetRange(i), maxRange, LASER_TOL);

  // Move laser model very close to terrain, it should now returns range values
  // that are less than half the max range
  std::string gpuLaserModelName = "gpu_laser";
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->GetModel(gpuLaserModelName)->SetWorldPose(
      math::Pose(math::Vector3(13.2, 0, 0.035), math::Quaternion(0, 0, 0)));

  // wait for a few laser scans
  i = 0;
  scanCount = 0;
  while (scanCount < 10 && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  for (i = 0; i < raySensor->GetRayCount(); ++i)
    EXPECT_TRUE(raySensor->GetRange(i) < maxRange / 2.0);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
