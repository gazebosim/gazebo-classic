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

#include <gtest/gtest.h>
#include "test/ServerFixture.hh"

using namespace gazebo;
class SonarSensor_TEST : public ServerFixture
{
};

static std::string sonarSensorString =
"<sdf version='1.4'>"
"  <sensor name='sonar' type='sonar'>"
"    <always_on>1</always_on>"
"    <visualize>1</visualize>"
"    <update_rate>20.000000</update_rate>"
"    <sonar>"
"      <min>0</min>"
"      <max>1</max>"
"      <radius>0.3</radius>"
"    </sonar>"
"  </sensor>"
"</sdf>";

/////////////////////////////////////////////////
/// \brief Test Creation of a Sonar sensor
TEST_F(SonarSensor_TEST, CreateSonar)
{
  Load("worlds/empty.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(sonarSensorString, sdf);

  // Create the Ray sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "ground_plane::link");

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName, std::string("default::ground_plane::link::sonar"));

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the sonar sensor
  sensors::SonarSensorPtr sensor =
    boost::dynamic_pointer_cast<sensors::SonarSensor>(
        mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != NULL);

  EXPECT_DOUBLE_EQ(sensor->GetRangeMin(), 0.0);
  EXPECT_DOUBLE_EQ(sensor->GetRangeMax(), 1.0);
  EXPECT_DOUBLE_EQ(sensor->GetRadius(), 0.3);
  EXPECT_DOUBLE_EQ(sensor->GetRange(), 0.0);

  EXPECT_TRUE(sensor->IsActive());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
