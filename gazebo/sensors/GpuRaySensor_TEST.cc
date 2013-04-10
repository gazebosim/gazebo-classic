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

#include <gtest/gtest.h>
#include "gazebo/math/Angle.hh"
#include "test/ServerFixture.hh"

using namespace gazebo;
class GPURaySensor_TEST : public ServerFixture
{
};

/////////////////////////////////////////////////
/// \brief Test Creation of a Ray sensor
TEST_F(GPURaySensor_TEST, CreateLaser)
{
  Load("worlds/gpu_laser.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  // Create the Ray sensor
  std::string sensorName = "default::model_1::link_1::laser_sensor";

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the Ray sensor
  sensors::GpuRaySensorPtr sensor =
    boost::shared_dynamic_cast<sensors::GpuRaySensor>
    (mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != NULL);

  double angleRes = (sensor->GetAngleMax() - sensor->GetAngleMin()).Radian() /
                    sensor->GetRayCount();
  EXPECT_EQ(sensor->GetAngleMin(), math::Angle(-1.396263));
  EXPECT_EQ(sensor->GetAngleMax(), math::Angle(1.396263));
  EXPECT_NEAR(sensor->GetRangeMin(), 0.08, 1e-6);
  EXPECT_NEAR(sensor->GetRangeMax(), 10.0, 1e-6);
  EXPECT_NEAR(sensor->GetAngleResolution(), angleRes, 1e-3);
  EXPECT_NEAR(sensor->GetRangeResolution(), 0.01, 1e-3);
  EXPECT_EQ(sensor->GetRayCount(), 640);
  EXPECT_EQ(sensor->GetRangeCount(), 640);

  EXPECT_EQ(sensor->GetVerticalRayCount(), 1);
  EXPECT_EQ(sensor->GetVerticalRangeCount(), 1);
  EXPECT_EQ(sensor->GetVerticalAngleMin(), -0.7554);
  EXPECT_EQ(sensor->GetVerticalAngleMax(), 0.7554);

  EXPECT_TRUE(sensor->IsActive());
  EXPECT_TRUE(sensor->IsHorizontal());

/// \TODO: this interface needs debugging, currently, only gazebo_ros_gpu_laser
/// accessor works well for now.
/*
  // Update the sensor
  sensor->Update(true);

  // Get all the range values
  std::vector<double> ranges;
  sensor->GetRanges(ranges);
  EXPECT_EQ(ranges.size(), static_cast<size_t>(640));

  // Check that all the range values
  for (unsigned int i = 0; i < ranges.size(); ++i)
  {
    EXPECT_NEAR(ranges[i], sensor->GetRangeMax(), 1e-6);
    EXPECT_NEAR(sensor->GetRange(i), ranges[i], 1e-6);
    EXPECT_NEAR(sensor->GetRetro(i), 0, 1e-6);
    EXPECT_EQ(sensor->GetFiducial(i), -1);
  }
*/
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
/*
/////////////////////////////////////////////////
//                                             //
//  GPU Laser World has a few objects,         //
//  spot check if GPULaser is return correct   //
//  values.                                    //
//                                             //
/////////////////////////////////////////////////
TEST_F(GpuLaser_TEST, GpuLaserWorldTest)
{
  Load("worlds/gpu_laser.world");

  // Get sensors
  GpuRaySensor gpuRaySensor =
    boost::shared_dynamic_cast<sensors::GpuRaySensor>
      (sensors::SensorManager::Instance()->GetSensor(
        "default::laser_sensor"));

  gazebo::rendering::GpuLaserPtr laserCam =
    scene->CreateGpuLaser("test_laser", false);

  EXPECT_TRUE(laserCam != NULL);

  // The following tests all the getters and setters
  {
    laserCam->SetNearClip(0.1);
    EXPECT_NEAR(laserCam->GetNearClip(), 0.1, 1e-6);

    laserCam->SetFarClip(100.0);
    EXPECT_NEAR(laserCam->GetFarClip(), 100, 1e-6);

    laserCam->SetHorzHalfAngle(1.2);
    EXPECT_NEAR(laserCam->GetHorzHalfAngle(), 1.2, 1e-6);

    laserCam->SetVertHalfAngle(0.5);
    EXPECT_NEAR(laserCam->GetVertHalfAngle(), 0.5, 1e-6);

    laserCam->SetIsHorizontal(false);
    EXPECT_FALSE(laserCam->IsHorizontal());

    laserCam->SetHorzFOV(2.4);
    EXPECT_NEAR(laserCam->GetHorzFOV(), 2.4, 1e-6);

    laserCam->SetVertFOV(1.0);
    EXPECT_NEAR(laserCam->GetVertFOV(), 1.0, 1e-6);

    laserCam->SetCosHorzFOV(0.2);
    EXPECT_NEAR(laserCam->GetCosHorzFOV(), 0.2, 1e-6);

    laserCam->SetCosVertFOV(0.1);
    EXPECT_NEAR(laserCam->GetCosVertFOV(), 0.1, 1e-6);

    laserCam->SetRayCountRatio(0.344);
    EXPECT_NEAR(laserCam->GetRayCountRatio(), 0.344, 1e-6);

    laserCam->SetCameraCount(4);
    EXPECT_EQ(laserCam->GetCameraCount(), 4);
  }
}
*/
