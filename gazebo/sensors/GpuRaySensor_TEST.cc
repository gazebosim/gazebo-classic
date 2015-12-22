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

#include <boost/bind.hpp>
#include <gtest/gtest.h>
#include <ignition/math/Angle.hh>
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class GPURaySensor_TEST : public ServerFixture
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
/// \brief Test Creation of a Ray sensor
TEST_F(GPURaySensor_TEST, CreateLaser)
{
  Load("worlds/gpu_laser2.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  // Create the Ray sensor
  std::string sensorName = "default::model_1::link_1::laser_sensor";

  // Get a pointer to the Ray sensor
  sensors::GpuRaySensorPtr sensor =
     boost::dynamic_pointer_cast<sensors::GpuRaySensor>
     (mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != NULL);

  double angleRes = (sensor->AngleMax() - sensor->AngleMin()).Radian() /
                    sensor->GetRayCount();
  EXPECT_EQ(sensor->AngleMin(), ignition::math::Angle(-1.396263));
  EXPECT_EQ(sensor->AngleMax(), ignition::math::Angle(1.396263));
  EXPECT_NEAR(sensor->GetRangeMin(), 0.08, 1e-6);
  EXPECT_NEAR(sensor->GetRangeMax(), 10.0, 1e-6);
  EXPECT_NEAR(sensor->GetAngleResolution(), angleRes, 1e-3);
  EXPECT_NEAR(sensor->GetRangeResolution(), 0.01, 1e-3);
  EXPECT_EQ(sensor->GetRayCount(), 640);
  EXPECT_EQ(sensor->GetRangeCount(), 640);

  EXPECT_EQ(sensor->GetVerticalRayCount(), 1);
  EXPECT_EQ(sensor->GetVerticalRangeCount(), 1);
  EXPECT_EQ(sensor->VerticalAngleMin(), 0.0);
  EXPECT_EQ(sensor->VerticalAngleMax(), 0.0);

  EXPECT_TRUE(sensor->IsActive());
  EXPECT_TRUE(sensor->IsHorizontal());

  // listen to new laser frames
  float *scan = new float[sensor->GetRayCount()
      * sensor->GetVerticalRayCount() * 3];
  int scanCount = 0;
  event::ConnectionPtr c =
    sensor->ConnectNewLaserFrame(
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

  // Get all the range values
  std::vector<double> ranges;
  sensor->GetRanges(ranges);
  EXPECT_EQ(ranges.size(), static_cast<size_t>(640));

  // Check that all the range values
  for (unsigned int i = 0; i < ranges.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(ranges[i], GZ_DBL_INF);
    EXPECT_DOUBLE_EQ(sensor->GetRange(i), ranges[i]);
    EXPECT_NEAR(sensor->GetRetro(i), 0, 1e-6);
    EXPECT_EQ(sensor->GetFiducial(i), -1);
  }

  delete [] scan;
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
