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
#include "gazebo/sdf/sdf.hh"
#include "test/ServerFixture.hh"

using namespace gazebo;
class RaySensor_TEST : public ServerFixture
{
};

static std::string raySensorString =
"<sdf version='1.3'>"
"  <sensor name='laser' type='ray'>"
"    <always_on>1</always_on>"
"    <visualize>1</visualize>"
"    <update_rate>20.000000</update_rate>"
"    <ray>"
"      <scan>"
"        <horizontal>"
"          <samples>640</samples>"
"          <resolution>1.000000</resolution>"
"          <min_angle>-2.268900</min_angle>"
"          <max_angle>2.268900</max_angle>"
"        </horizontal>"
"      </scan>"
"      <range>"
"        <min>0.080000</min>"
"        <max>10.000000</max>"
"        <resolution>0.010000</resolution>"
"      </range>"
"    </ray>"
"  </sensor>"
"</sdf>";

/////////////////////////////////////////////////
/// \brief Test Creation of a Ray sensor
TEST_F(RaySensor_TEST, CreateLaser)
{
  Load("worlds/empty.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(raySensorString, sdf);

  // Create the Ray sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "ground_plane::link");

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName, std::string("default::ground_plane::link::laser"));

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the Ray sensor
  sensors::RaySensorPtr sensor = boost::shared_dynamic_cast<sensors::RaySensor>
    (mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != NULL);

  EXPECT_EQ(sensor->GetAngleMin(), );
  EXPECT_EQ(sensor->GetAngleMax(), );
  EXPECT_EQ(sensor->GetRangeMin(), );
  EXPECT_EQ(sensor->GetRangeMax(), );
  EXPECT_EQ(sensor->GetAngleResolution(), );
  EXPECT_EQ(sensor->GetRangeResolution(), );
  EXPECT_EQ(sensor->GetRayCount(), );
  EXPECT_EQ(sensor->GetRangeCount(), );
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
