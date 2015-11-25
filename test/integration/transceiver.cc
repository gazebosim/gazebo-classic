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
#include <ignition/math/Rand.hh>

#include <boost/foreach.hpp>
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

class TransceiverTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  public: TransceiverTest();
  public: void TxRxEmptySpace(const std::string &_physicsEngine);
  public: void TxRxObstacle(const std::string &_physicsEngine);
  public: void TxRxFreqOutOfBounds(const std::string &_physicsEngine);
  private: void RxMsg(const ConstWirelessNodesPtr &_msg);

  private: static const double MinFreq;
  private: static const double MaxFreq;
  private: static const double Gain;
  private: static const double Power;
  private: static const double Sensitivity;
  private: static const double MaxPos;

  private: boost::mutex mutex;
  private: std::vector<int> num_msgs;
  private: std::vector<common::Time> elapsed_time;
  private: boost::shared_ptr<msgs::WirelessNodes const> nodesMsg;
  private: bool receivedMsg;
};

const double TransceiverTest::MinFreq = 2412.0;
const double TransceiverTest::MaxFreq = 2484.0;
const double TransceiverTest::Gain = 2.6;
const double TransceiverTest::Power = 14.5;
const double TransceiverTest::Sensitivity = -90.0;
const double TransceiverTest::MaxPos = 10.0;

/////////////////////////////////////////////////
TransceiverTest::TransceiverTest()
{
  this->receivedMsg = false;
}

/////////////////////////////////////////////////
void TransceiverTest::RxMsg(const ConstWirelessNodesPtr &_msg)
{
  boost::mutex::scoped_lock lock(mutex);
  // Just copy the message
  this->nodesMsg = _msg;
  this->receivedMsg = true;
}

/////////////////////////////////////////////////
void TransceiverTest::TxRxEmptySpace(const std::string &_physicsEngine)
{
  typedef std::map<std::string, sensors::WirelessTransmitterPtr> trans_map_type;
  trans_map_type transmitters;

  Load("worlds/empty.world", true, _physicsEngine);

  // Generate a random number [1-10] of transmitters
  int nTransmitters = ignition::math::Rand::IntUniform(1, 10);

  for (int i = 0; i < nTransmitters; ++i)
  {
    double txFreq = ignition::math::Rand::DblUniform(this->MinFreq,
        this->MaxFreq);
    std::ostringstream convert;
    convert << i;
    std::string txModelName = "tx" + convert.str();
    std::string txSensorName = "wirelessTransmitter" + convert.str();
    std::string txEssid = "osrf" + convert.str();
    double x = ignition::math::Rand::DblUniform(-this->MaxPos, this->MaxPos);
    double y = ignition::math::Rand::DblUniform(-this->MaxPos, this->MaxPos);
    math::Pose txPose(math::Vector3(x, y, 0.055), math::Quaternion(0, 0, 0));

    SpawnWirelessTransmitterSensor(txModelName, txSensorName, txPose.pos,
        txPose.rot.GetAsEuler(), txEssid, txFreq, this->Power, this->Gain);

    sensors::WirelessTransmitterPtr tx =
        boost::static_pointer_cast<sensors::WirelessTransmitter>(
          sensors::SensorManager::Instance()->GetSensor(txSensorName));

    // Store the new transmitter sensor in the map
    transmitters[txEssid] = tx;

    ASSERT_TRUE(tx != NULL);
  }

  // Wireless Receiver - rx
  std::string rxModelName = "rx";
  std::string rxSensorName = "wirelessReceiver";
  math::Pose rxPose(math::Vector3(0, 2, 0.055),
      math::Quaternion(0, 0, 0));

  // Spawn rx
  SpawnWirelessReceiverSensor(rxModelName, rxSensorName, rxPose.pos,
      rxPose.rot.GetAsEuler(), this->MinFreq, this->MaxFreq, this->Power,
      this->Gain, this->Sensitivity);

  sensors::WirelessReceiverPtr rx =
    boost::static_pointer_cast<sensors::WirelessReceiver>(
        sensors::SensorManager::Instance()->GetSensor(rxSensorName));

  ASSERT_TRUE(rx != NULL);

  // Initialize gazebo transport layer
  transport::NodePtr node(new transport::Node());
  node->Init("default");

  std::string rxTopic = "/gazebo/default/rx/link/wirelessReceiver/transceiver";
  transport::SubscriberPtr sub = node->Subscribe(rxTopic,
      &TransceiverTest::RxMsg, this);
  this->receivedMsg = false;

  // Loop a max. of ~5 seconds
  for (int i = 0; i < 50; ++i)
  {
    // Update all the transmitter sensors
    BOOST_FOREACH(const trans_map_type::value_type& myPair, transmitters)
    {
      myPair.second->Update(true);
    }

    // Update the receiver sensor
    rx->Update(true);

    common::Time::MSleep(100);
    boost::mutex::scoped_lock lock(this->mutex);

    if (this->nodesMsg && this->receivedMsg)
    {
      this->receivedMsg = false;
      gazebo::msgs::WirelessNodes txNodes;
      int numTxNodes = nodesMsg->node_size();

      for (int i = 0; i < numTxNodes; ++i)
      {
        gazebo::msgs::WirelessNode txNode = nodesMsg->node(i);
        std::string essid = txNode.essid();
        EXPECT_EQ(transmitters[essid]->ESSID(), essid);
        EXPECT_EQ(transmitters[essid]->Freq(), txNode.frequency());
        EXPECT_LE(txNode.signal_level(), 0);
        EXPECT_GE(txNode.signal_level(), rx->Sensitivity());
      }
      return;
    }
  }
  FAIL();
}

/////////////////////////////////////////////////
void TransceiverTest::TxRxFreqOutOfBounds(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);

  double txFreq = this->MinFreq - 1.0;
  std::string tx1ModelName = "tx1";
  std::string tx1SensorName = "wirelessTransmitter1";
  std::string tx2ModelName = "tx2";
  std::string tx2SensorName = "wirelessTransmitter2";
  std::string txEssid = "osrf";
  double x = ignition::math::Rand::DblUniform(-this->MaxPos, this->MaxPos);
  double y = ignition::math::Rand::DblUniform(-this->MaxPos, this->MaxPos);
  math::Pose txPose(math::Vector3(x, y, 0.055), math::Quaternion(0, 0, 0));

  SpawnWirelessTransmitterSensor(tx1ModelName, tx1SensorName, txPose.pos,
      txPose.rot.GetAsEuler(), txEssid, txFreq, this->Power, this->Gain);

  sensors::WirelessTransmitterPtr tx1 =
      boost::static_pointer_cast<sensors::WirelessTransmitter>(
        sensors::SensorManager::Instance()->GetSensor(tx1SensorName));

  ASSERT_TRUE(tx1 != NULL);

  txFreq = this->MaxFreq + 1.0;
  SpawnWirelessTransmitterSensor(tx2ModelName, tx2SensorName, txPose.pos,
      txPose.rot.GetAsEuler(), txEssid, txFreq, this->Power, this->Gain);

  sensors::WirelessTransmitterPtr tx2 =
      boost::static_pointer_cast<sensors::WirelessTransmitter>(
        sensors::SensorManager::Instance()->GetSensor(tx2SensorName));

  ASSERT_TRUE(tx2 != NULL);

  // Wireless Receiver - rx
  std::string rxModelName = "rx";
  std::string rxSensorName = "wirelessReceiver";
  math::Pose rxPose(math::Vector3(0, 2, 0.055),
      math::Quaternion(0, 0, 0));

  // Spawn rx
  SpawnWirelessReceiverSensor(rxModelName, rxSensorName, rxPose.pos,
      rxPose.rot.GetAsEuler(), this->MinFreq, this->MaxFreq, this->Power,
      this->Gain, this->Sensitivity);

  sensors::WirelessReceiverPtr rx =
    boost::static_pointer_cast<sensors::WirelessReceiver>(
        sensors::SensorManager::Instance()->GetSensor(rxSensorName));

  ASSERT_TRUE(rx != NULL);

  // Initialize gazebo transport layer
  transport::NodePtr node(new transport::Node());
  node->Init("default");

  std::string rxTopic = "/gazebo/default/rx/link/wirelessReceiver/transceiver";
  transport::SubscriberPtr sub = node->Subscribe(rxTopic,
      &TransceiverTest::RxMsg, this);
  this->receivedMsg = false;

  // Loop a max. of ~5 seconds
  for (int i = 0; i < 50; ++i)
  {
    // Update the sensors
    tx1->Update(true);
    tx2->Update(true);
    rx->Update(true);

    common::Time::MSleep(100);
    boost::mutex::scoped_lock lock(this->mutex);
  }

  EXPECT_FALSE(this->receivedMsg);
}

/////////////////////////////////////////////////
void TransceiverTest::TxRxObstacle(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);

  double avgSignalLevelEmpty = 0.0;
  double avgSignalLevelObstacle = 0.0;
  int samples = 0;

  // Wireless Transmitter - tx
  std::string txModelName = "tx";
  std::string txSensorName = "wirelessTx";
  math::Pose txPose(math::Vector3(0, 0, 0.5),
      math::Quaternion(0, 0, 0));

  // Spawn tx
  SpawnWirelessTransmitterSensor(txModelName, txSensorName, txPose.pos,
      txPose.rot.GetAsEuler(), "osrf", 2450.0, this->Power, this->Gain);

  sensors::WirelessTransmitterPtr tx =
      boost::static_pointer_cast<sensors::WirelessTransmitter>(
        sensors::SensorManager::Instance()->GetSensor(txSensorName));

  ASSERT_TRUE(tx != NULL);

  // Wireless Receiver - rx1
  std::string rx1ModelName = "rx1";
  std::string rx1SensorName = "wirelessRx1";
  math::Pose rx1Pose(math::Vector3(3, 0, 0.5),
      math::Quaternion(0, 0, 0));

  // Spawn rx1
  SpawnWirelessReceiverSensor(rx1ModelName, rx1SensorName, rx1Pose.pos,
      rx1Pose.rot.GetAsEuler(), this->MinFreq, this->MaxFreq, this->Power,
      this->Gain, this->Sensitivity);

  sensors::WirelessReceiverPtr rx1 =
      boost::static_pointer_cast<sensors::WirelessReceiver>(
        sensors::SensorManager::Instance()->GetSensor(rx1SensorName));

  ASSERT_TRUE(rx1 != NULL);

  // Wireless Receiver - rx2
  std::string rx2ModelName = "rx2";
  std::string rx2SensorName = "wirelessRx2";
  math::Pose rx2Pose(math::Vector3(-2, 0, 0.5), math::Quaternion(0, 0, 0));

  // Spawn rx2
  SpawnWirelessReceiverSensor(rx2ModelName, rx2SensorName, rx2Pose.pos,
      rx2Pose.rot.GetAsEuler(), this->MinFreq, this->MaxFreq, this->Power,
      this->Gain, this->Sensitivity);

  sensors::WirelessReceiverPtr rx2 =
      boost::static_pointer_cast<sensors::WirelessReceiver>(
        sensors::SensorManager::Instance()->GetSensor(rx2SensorName));

  ASSERT_TRUE(rx2 != NULL);

  // Spawn an obstacle between the transmitter and the receiver
  SpawnBox("Box", math::Vector3(1, 1, 1), math::Vector3(-1, 0, 0.5),
      math::Vector3(0, 0, 0), true);

  // Initialize gazebo transport layer
  transport::NodePtr node(new transport::Node());
  node->Init("default");

  std::string rx1Topic = "/gazebo/default/rx1/link/wirelessRx1/transceiver";
  transport::SubscriberPtr sub = node->Subscribe(rx1Topic,
      &TransceiverTest::RxMsg, this);

  this->receivedMsg = false;

  // Loop until you have 10 samples or timeout after ~5secs
  int iters = 0;
  while (samples < 10 && iters < 50)
  {
    // Update all the sensors
    tx->Update(true);
    rx1->Update(true);

    common::Time::MSleep(100);
    boost::mutex::scoped_lock lock(this->mutex);

    if (this->nodesMsg && this->receivedMsg)
    {
      this->receivedMsg = false;
      gazebo::msgs::WirelessNodes txNodes;
      int numTxNodes = nodesMsg->node_size();

      for (int i = 0; i < numTxNodes; ++i)
      {
        gazebo::msgs::WirelessNode txNode = nodesMsg->node(i);
        double signalLevel = txNode.signal_level();

        ++samples;
        avgSignalLevelEmpty += signalLevel;
      }
    }
    ++iters;
  }

  samples = 0;
  iters = 0;

  std::string rx2Topic = "/gazebo/default/rx2/link/wirelessRx2/transceiver";
  sub = node->Subscribe(rx2Topic, &TransceiverTest::RxMsg, this);

  this->receivedMsg = false;

  // Loop until you have 10 samples or timeout after ~5secs
  while (samples < 10 && iters < 50)
  {
    // Update all the sensors
    tx->Update(true);
    rx2->Update(true);

    common::Time::MSleep(100);
    boost::mutex::scoped_lock lock(this->mutex);

    if (this->nodesMsg && this->receivedMsg)
    {
      this->receivedMsg = false;
      gazebo::msgs::WirelessNodes txNodes;
      int numTxNodes = nodesMsg->node_size();

      for (int i = 0; i < numTxNodes; ++i)
      {
        gazebo::msgs::WirelessNode txNode = nodesMsg->node(i);
        double signalLevel = txNode.signal_level();

        ++samples;
        avgSignalLevelObstacle += signalLevel;
      }
    }
    ++iters;
  }

  // Check that the signal level in the not-occluded receiver is higher than
  // the signal received by the occluded receiver
  EXPECT_GT(avgSignalLevelEmpty / samples, avgSignalLevelObstacle / samples);
}

/////////////////////////////////////////////////
TEST_P(TransceiverTest, EmptyWorld)
{
  TxRxEmptySpace(GetParam());
}

/////////////////////////////////////////////////
TEST_P(TransceiverTest, Obstacle)
{
  if (std::string(GetParam()) == "simbody")
  {
    gzerr << "Abort test since this test frequently fails with simbody, "
          << " see (issues #867)" << std::endl;
    return;
  }
  if (std::string(GetParam()) == "dart")
  {
    gzerr << "Abort test since this test frequently fails with dart, "
          << " see (issues #911)" << std::endl;
    return;
  }

  TxRxObstacle(GetParam());
}

/////////////////////////////////////////////////
TEST_P(TransceiverTest, FreqOutOfBounds)
{
  TxRxFreqOutOfBounds(GetParam());
}

/////////////////////////////////////////////////
INSTANTIATE_TEST_CASE_P(PhysicsEngines, TransceiverTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
