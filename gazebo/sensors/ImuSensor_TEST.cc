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

#include <sys/time.h>
#include <gtest/gtest.h>

#include "test/ServerFixture.hh"
#include "gazebo/sensors/ImuSensor.hh"

using namespace gazebo;
class ImuSensor_TEST : public ServerFixture
{
};

static std::string imuSensorString =
"<sdf version='1.3'>"
"  <sensor name='imu' type='imu'>"
"    <always_on>1</always_on>"
"    <update_rate>20.000000</update_rate>"
"    <imu>"
"      <topic>/test_imu</topic>"
"    </imu>"
"  </sensor>"
"</sdf>";

TEST_F(ImuSensor_TEST, BasicImuSensorCheck)
{
  Load("worlds/empty.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(imuSensorString, sdf);

  // Create the IMU sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "ground_plane::link");

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName, std::string("default::ground_plane::link::imu"));

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the Ray sensor
  sensors::ImuSensorPtr sensor = boost::shared_dynamic_cast<sensors::ImuSensor>
    (mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != NULL);

  EXPECT_EQ(sensor->GetAngularVelocity(), math::Vector3::Zero);
  EXPECT_EQ(sensor->GetLinearAcceleration(), math::Vector3::Zero);
  EXPECT_EQ(sensor->GetOrientation(), math::Quaternion());
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
