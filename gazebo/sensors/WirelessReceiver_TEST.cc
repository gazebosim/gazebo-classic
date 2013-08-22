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
#include "test/ServerFixture.hh"

using namespace gazebo;
class WirelessReceiver_TEST : public ServerFixture
{
};

static std::string receiverSensorString =
"<sdf version='1.4'>"
"  <sensor name='wirelessReceiver' type='wireless_receiver'>"
"    <always_on>1</always_on>"
"    <visualize>0</visualize>"
"    <update_rate>1.0</update_rate>"
"    <transceiver>"
"      <min_frequency>2412.0</min_frequency>"
"      <max_frequency>2484.0</max_frequency>"
"      <power>14.5</power>"
"      <gain>2.5</gain>"
"      <sensitivity>-90.0</sensitivity>"
"    </transceiver>"
"  </sensor>"
"</sdf>";

/////////////////////////////////////////////////
/// \brief Test Creation of a wireless receiver sensor
TEST_F(WirelessReceiver_TEST, CreateWirelessReceiver)
{
  Load("worlds/empty.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(receiverSensorString, sdf);

  // Create the wireless receiver sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "ground_plane::link");

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName,
      std::string("default::ground_plane::link::wirelessReceiver"));

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the wireless receiver sensor
  sensors::WirelessReceiverPtr sensor =
    boost::dynamic_pointer_cast<sensors::WirelessReceiver>(
        mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != NULL);

  EXPECT_DOUBLE_EQ(sensor->GetMinFreqFiltered(), 2412.0);
  EXPECT_DOUBLE_EQ(sensor->GetMaxFreqFiltered(), 2484.0);
  EXPECT_DOUBLE_EQ(sensor->GetSensitivity(), -90);

  EXPECT_TRUE(sensor->IsActive());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
