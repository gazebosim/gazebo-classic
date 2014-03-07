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

#include <gtest/gtest.h>
#include "test/ServerFixture.hh"

using namespace gazebo;
class GpsSensor_TEST : public ServerFixture
{
};

static std::string gpsSensorString =
"<sdf version='1.4'>"
"  <sensor name='gps' type='gps'>"
"    <always_on>1</always_on>"
"    <update_rate>10.000000</update_rate>"
"    <gps>"
"      <position_sensing>"
"        <horizontal>"
"          <noise type='gaussian_quantized'>"
"            <mean>0</mean>"
"            <stddev>1</stddev>"
"            <bias_mean>3</bias_mean>"
"            <bias_stddev>1</bias_stddev>"
"            <precision>0.5</precision>"
"          </noise>"
"        </horizontal>"
"        <vertical>"
"          <noise type='gaussian_quantized'>"
"            <mean>0</mean>"
"            <stddev>1</stddev>"
"            <bias_mean>3</bias_mean>"
"            <bias_stddev>1</bias_stddev>"
"            <precision>1.0</precision>"
"          </noise>"
"        </vertical>"
"      </position_sensing>"
"      <velocity_sensing>"
"        <horizontal>"
"          <noise type='gaussian_quantized'>"
"            <mean>0</mean>"
"            <stddev>0.1</stddev>"
"            <bias_mean>0.1</bias_mean>"
"            <bias_stddev>0.1</bias_stddev>"
"            <precision>0.1</precision>"
"          </noise>"
"        </horizontal>"
"        <vertical>"
"          <noise type='gaussian_quantized'>"
"            <mean>0</mean>"
"            <stddev>0.2</stddev>"
"            <bias_mean>0.2</bias_mean>"
"            <bias_stddev>0.2</bias_stddev>"
"            <precision>0.2</precision>"
"          </noise>"
"        </vertical>"
"      </velocity_sensing>"
"    </gps>"
"  </sensor>"
"</sdf>";

/////////////////////////////////////////////////
/// \brief Test Creation of a Gps sensor
TEST_F(GpsSensor_TEST, CreateGps)
{
  Load("worlds/empty.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(gpsSensorString, sdf);

  // Create the Ray sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "ground_plane::link", 0);

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName, std::string("default::ground_plane::link::gps"));

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the gps sensor
  sensors::GpsSensorPtr sensor =
    boost::dynamic_pointer_cast<sensors::GpsSensor>(
        mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != NULL);

  EXPECT_DOUBLE_EQ(sensor->GetLatitude().Radian(), 0.0);
  EXPECT_DOUBLE_EQ(sensor->GetLongitude().Radian(), 0.0);
  EXPECT_DOUBLE_EQ(sensor->GetAltitude(), 0.0);

  EXPECT_TRUE(sensor->IsActive());

  physics::WorldPtr world = physics::get_world();
  ASSERT_TRUE(world != NULL);
  world->Step(100);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
