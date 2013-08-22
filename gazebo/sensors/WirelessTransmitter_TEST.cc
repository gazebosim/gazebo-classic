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
#include <boost/algorithm/string/replace.hpp>
#include "test/ServerFixture.hh"

using namespace gazebo;
class WirelessTransmitter_TEST : public ServerFixture
{
    public: WirelessTransmitter_TEST();
    public: void TestCreateWirelessTransmitter();
    public: void TestSignalStrength();
    public: void TestUpdateImpl();
    private: void TxMsg(const ConstPropagationGridPtr &_msg);

    private: boost::mutex mutex;
    private: bool receivedMsg;
    private: boost::shared_ptr<msgs::PropagationGrid const> gridMsg;
    private: sensors::WirelessTransmitterPtr tx;
};

static std::string transmitterSensorString =
"<sdf version='1.4'>"
"  <sensor name='wirelessTransmitter' type='wireless_transmitter'>"
"    <always_on>1</always_on>"
"    <visualize>true</visualize>"
"    <update_rate>1.0</update_rate>"
"    <transceiver>"
"      <essid>GzTest</essid>"
"      <frequency>2442.0</frequency>"
"      <power>14.5</power>"
"      <gain>2.6</gain>"
"    </transceiver>"
"  </sensor>"
"</sdf>";

WirelessTransmitter_TEST::WirelessTransmitter_TEST()
{
  Load("worlds/empty.world");

  std::string txModelName = "tx";
  std::string txSensorName = "wirelessTransmitter";
  std::string txEssid = "GzTest";
  math::Pose txPose(math::Vector3(0.0, 0.0, 0.055), math::Quaternion(0, 0, 0));

  SpawnWirelessTransmitterSensor(txModelName, txSensorName, txPose.pos,
      txPose.rot.GetAsEuler(), txEssid, 2442.0, 14.5, 2.6);

  this->tx = boost::static_pointer_cast<sensors::WirelessTransmitter>(
      sensors::SensorManager::Instance()->GetSensor(txSensorName));

  this->receivedMsg = false;
}

/////////////////////////////////////////////////
/// \brief Test Creation of a wireless transmitter sensor
void WirelessTransmitter_TEST::TestCreateWirelessTransmitter()
{
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(transmitterSensorString, sdf);

  // Create the wireless transmitter sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "ground_plane::link");

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName,
      std::string("default::ground_plane::link::wirelessTransmitter"));

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the wireless receiver sensor
  sensors::WirelessTransmitterPtr sensor =
    boost::dynamic_pointer_cast<sensors::WirelessTransmitter>(
        mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != NULL);

  EXPECT_EQ("GzTest", sensor->GetESSID());
  EXPECT_DOUBLE_EQ(sensor->GetFreq(), 2442.0);

  EXPECT_TRUE(sensor->IsActive());
}

/////////////////////////////////////////////////
/// \brief Test Creation of a wireless transmitter sensor with an empty ESSID
void WirelessTransmitter_TEST::TestSignalStrength()
{
  int samples = 100;
  double signalStrengthAvg = 0.0;
  math::Pose txPose2(math::Vector3(3.0, 3.0, 0.055), math::Quaternion(0, 0, 0));


  for (int i = 0; i < samples; ++i)
  {
    signalStrengthAvg += tx->GetSignalStrength(txPose2, tx->GetGain());
  }
  signalStrengthAvg /= samples;
  std::cout << signalStrengthAvg << std::endl;

  EXPECT_NEAR(signalStrengthAvg, -62.0, 5.0);
}

/////////////////////////////////////////////////
void WirelessTransmitter_TEST::TxMsg(const ConstPropagationGridPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  // Just copy the message
  this->gridMsg = _msg;
  this->receivedMsg = true;
}

/////////////////////////////////////////////////
/// \brief Test Creation of a wireless transmitter sensor with an empty ESSID
void WirelessTransmitter_TEST::TestUpdateImpl()
{
  // Initialize gazebo transport layer
  transport::NodePtr node(new transport::Node());
  node->Init("default");

  std::string txTopic =
      "/gazebo/default/tx/link/wirelessTransmitter/transceiver";
  transport::SubscriberPtr sub = node->Subscribe(txTopic,
      &WirelessTransmitter_TEST::TxMsg, this);

  for (int i = 0; i < 10; ++i)
  {
    this->tx->Update(true);
    common::Time::MSleep(100); 
  }

  boost::mutex::scoped_lock lock(this->mutex);
  EXPECT_TRUE(this->receivedMsg);
}

/////////////////////////////////////////////////
TEST_F(WirelessTransmitter_TEST, TestSensorCreation)
{
  TestCreateWirelessTransmitter();
}

/////////////////////////////////////////////////
TEST_F(WirelessTransmitter_TEST, TestSignalStrength)
{
  TestSignalStrength();
}

/////////////////////////////////////////////////
TEST_F(WirelessTransmitter_TEST, TestUpdateImpl)
{
  TestUpdateImpl();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
