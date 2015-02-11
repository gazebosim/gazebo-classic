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

#include <gtest/gtest.h>
#include "test/ServerFixture.hh"

using namespace gazebo;
class WirelessTransmitter_TEST : public ServerFixture
{
    public: WirelessTransmitter_TEST();
    public: void TestCreateWirelessTransmitter();
    public: void TestSignalStrength();
    public: void TestUpdateImpl();
    public: void TestUpdateImplNoVisual();
    public: void TestInvalidFreq();
    private: void TxMsg(const ConstPropagationGridPtr &_msg);

    private: boost::mutex mutex;
    private: bool receivedMsg;
    private: boost::shared_ptr<msgs::PropagationGrid const> gridMsg;
    private: sensors::WirelessTransmitterPtr tx;
    private: sensors::WirelessTransmitterPtr txNoVisual;
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

/////////////////////////////////////////////////
/// \brief Constructor spawns a transmitter sensor
WirelessTransmitter_TEST::WirelessTransmitter_TEST()
{
  Load("worlds/empty.world");

  std::string txModelName = "tx";
  std::string txSensorName = "wirelessTransmitter";
  std::string txEssid = "GzTest";
  double freq = 2442.0;
  double power = 14.5;
  double gain = 2.6;
  math::Pose txPose(math::Vector3(0.0, 0.0, 0.055), math::Quaternion(0, 0, 0));

  // Spawn a wireless transmitter with sensor visualization
  SpawnWirelessTransmitterSensor(txModelName, txSensorName, txPose.pos,
      txPose.rot.GetAsEuler(), txEssid, freq, power, gain);

  this->tx = boost::static_pointer_cast<sensors::WirelessTransmitter>(
      sensors::SensorManager::Instance()->GetSensor(txSensorName));

  this->receivedMsg = false;
}

/////////////////////////////////////////////////
/// \brief Test creation of a wireless transmitter sensor
void WirelessTransmitter_TEST::TestCreateWirelessTransmitter()
{
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(transmitterSensorString, sdf);

  // Create the wireless transmitter sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "ground_plane::link", 0);

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
  ASSERT_TRUE(sensor != NULL);

  EXPECT_EQ("GzTest", sensor->GetESSID());
  EXPECT_DOUBLE_EQ(sensor->GetFreq(), 2442.0);
  EXPECT_DOUBLE_EQ(sensor->GetPower(), 14.5);
  EXPECT_DOUBLE_EQ(sensor->GetGain(), 2.6);

  EXPECT_TRUE(sensor->IsActive());
}

/////////////////////////////////////////////////
/// \brief Test an invalid frequency value
void WirelessTransmitter_TEST::TestInvalidFreq()
{
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);

  // Replace the essid by an empty value
  boost::regex re("<frequency>.*<\\/frequency>");
  std::string transmitterSensorStringCopy =
      boost::regex_replace(transmitterSensorString, re,
        "<frequency>-1.0</frequency>");

  sdf::readString(transmitterSensorStringCopy, sdf);

  // Create the wireless receiver sensor
  ASSERT_ANY_THROW(mgr->CreateSensor(sdf, "default", "ground_plane::link", 0));
}

/////////////////////////////////////////////////
/// \brief Test the signal strength function
void WirelessTransmitter_TEST::TestSignalStrength()
{
  int samples = 100;
  double signStrengthAvg = 0.0;
  math::Pose txPose(math::Vector3(3.0, 3.0, 0.055), math::Quaternion(0, 0, 0));
  math::Pose txPoseOccluded(math::Vector3(-3.0, -3.0, 0.055),
      math::Quaternion(0, 0, 0));

  // Take some samples and get the average signal strength
  for (int i = 0; i < samples; ++i)
  {
    this->tx->Update(true);
    signStrengthAvg += tx->GetSignalStrength(txPose, tx->GetGain());
  }
  signStrengthAvg /= samples;

  EXPECT_NEAR(signStrengthAvg, -62.0, this->tx->ModelStdDesv);
}

/////////////////////////////////////////////////
/// \brief Callback executed for every propagation grid message received
void WirelessTransmitter_TEST::TxMsg(const ConstPropagationGridPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  // Just copy the message
  this->gridMsg = _msg;
  this->receivedMsg = true;
}

/////////////////////////////////////////////////
/// \brief Test the publication of the propagation grid used for visualization
void WirelessTransmitter_TEST::TestUpdateImpl()
{
  // Initialize gazebo transport layer
  transport::NodePtr node(new transport::Node());
  node->Init("default");

  std::string txTopic =
      "/gazebo/default/tx/link/wirelessTransmitter/transceiver";
  transport::SubscriberPtr sub = node->Subscribe(txTopic,
      &WirelessTransmitter_TEST::TxMsg, this);

  // Make sure that the sensor is updated and some messages are published
  for (int i = 0; i < 10; ++i)
  {
    this->tx->Update(true);
    common::Time::MSleep(100);
  }

  boost::mutex::scoped_lock lock(this->mutex);
  EXPECT_TRUE(this->receivedMsg);
}

/////////////////////////////////////////////////
/// \brief Test the updateImpl method with the visualization disabled
void WirelessTransmitter_TEST::TestUpdateImplNoVisual()
{
  sensors::WirelessTransmitterPtr txNoVisual;
  std::string txModelName = "tx";
  std::string txNoVisualSensorName = "wirelessTransmitterNoVisual";
  std::string txEssid = "GzTest";
  double freq = 2442.0;
  double power = 14.5;
  double gain = 2.6;
  math::Pose txPose(math::Vector3(3.0, 3.0, 0.055), math::Quaternion(0, 0, 0));

  // Spawn a wireless transmitter without sensor visualization
  SpawnWirelessTransmitterSensor(txModelName + "NoVisual",
      txNoVisualSensorName, txPose.pos, txPose.rot.GetAsEuler(),
      txEssid + "NoVisual", freq, power, gain, false);

  txNoVisual = boost::static_pointer_cast<sensors::WirelessTransmitter>(
      sensors::SensorManager::Instance()->GetSensor(txNoVisualSensorName));

  // Initialize gazebo transport layer
  transport::NodePtr node(new transport::Node());
  node->Init("default");

  std::string txTopic =
      "/gazebo/default/txNoVisual/link/wirelessTransmitter/transceiver";
  transport::SubscriberPtr sub = node->Subscribe(txTopic,
      &WirelessTransmitter_TEST::TxMsg, this);

  // Make sure that the sensor is updated and some messages are published
  for (int i = 0; i < 10; ++i)
  {
    txNoVisual->Update(true);
    common::Time::MSleep(100);
  }

  boost::mutex::scoped_lock lock(this->mutex);
  EXPECT_FALSE(this->receivedMsg);
}

/////////////////////////////////////////////////
TEST_F(WirelessTransmitter_TEST, TestSensorCreation)
{
  TestCreateWirelessTransmitter();
}

/////////////////////////////////////////////////
TEST_F(WirelessTransmitter_TEST, TestInvalidFreq)
{
  TestInvalidFreq();
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
TEST_F(WirelessTransmitter_TEST, TestUpdateImplNoVisual)
{
  TestUpdateImplNoVisual();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
