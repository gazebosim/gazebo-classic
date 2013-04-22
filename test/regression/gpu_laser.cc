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
#include "sensors/sensors.hh"

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
  // Test GPU ray sensor with 3 boxes in the world.
  // First place 2 of 3 boxes within range and verify range values.
  // then move all 3 boxes out of range and verify range values

  Load("worlds/empty.world");

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

  SpawnGpuRaySensor(modelName, raySensorName, testPose.pos,
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

  // FIXME: Later part of the test calls SetWorldPose, which doesn't seem to
  // change a static model's visual pose and causes the test to fail!
  SpawnBox(box02, math::Vector3(1, 1, 1), box02Pose.pos,
      box02Pose.rot.GetAsEuler()/*, true*/);

  SpawnBox(box03, math::Vector3(1, 1, 1), box03Pose.pos,
      box03Pose.rot.GetAsEuler());

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // EXPECT_TRUE(world->GetModel(box02)->IsStatic());

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::GpuRaySensorPtr raySensor =
    boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensor);

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(raySensor != NULL);

  raySensor->SetActive(true);

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

  // wait for a few more laser scans
  i = 0;
  scanCount = 0;
  while (scanCount < 10 && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  for (int i = 0; i < raySensor->GetRayCount(); ++i)
  {
    EXPECT_NEAR(raySensor->GetRange(i), maxRange, LASER_TOL);
    break;
  }

  delete [] scan;
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
