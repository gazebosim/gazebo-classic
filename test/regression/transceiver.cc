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

#include "ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include <boost/foreach.hpp>

using namespace gazebo;

class TransceiverTest : public ServerFixture
{
  public: void TxRxEmptySpace(const std::string &_physicsEngine);
  public: void TxRxObstacle(const std::string &_physicsEngine);
  private: void RxMsg(const ConstWirelessNodesPtr &_msg);

  private: static const double FREQ_FROM;
  private: static const double FREQ_TO;
  private: static const double GAIN;
  private: static const double POWER;
  private: static const double SENSITIVITY;

  private: boost::mutex mutex;
  private: std::vector<int> num_msgs;
  private: std::vector<common::Time> elapsed_time;
  private: boost::shared_ptr<msgs::WirelessNodes const> nodesMsg;
  private: bool receivedMsg;
};

const double TransceiverTest::FREQ_FROM = 2412.0;
const double TransceiverTest::FREQ_TO = 2484.0;
const double TransceiverTest::GAIN = 2.6;
const double TransceiverTest::POWER = 14.5;
const double TransceiverTest::SENSITIVITY = -90.0;

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

  srand (time(NULL));
  Load("worlds/empty.world", true, _physicsEngine);

  // Generate a random number [1-10] of transmitters
  int nTransmitters = (rand() % 10) + 1; 

  for (int i = 0; i < nTransmitters; ++i)
  {
    double txFreq = math::Rand::GetDblUniform(2412.0, 2472.0);
    std::ostringstream convert;
    convert << i;
    std::string txModelName = "tx" + convert.str();
    std::string txSensorName = "wirelessTransmitter" + convert.str();
    std::string txEssid = "osrf" + convert.str();
    double x = math::Rand::GetDblUniform(-10.0, 10.0);
    double y = math::Rand::GetDblUniform(-10.0, 10.0);
    math::Pose txPose(math::Vector3(x, y, 0.055),
        math::Quaternion(0, 0, 0));

    SpawnWirelessTransmitterSensor(txModelName, txSensorName, txPose.pos,
      txPose.rot.GetAsEuler(), txEssid, txFreq, this->POWER, this->GAIN);

    sensors::WirelessTransmitterPtr tx =
      boost::static_pointer_cast<sensors::WirelessTransmitter>(
        sensors::SensorManager::Instance()->GetSensor(txSensorName));
    
    // Store the new transmitter sensor in the map
    transmitters[txEssid] = tx;

    ASSERT_TRUE(tx);
  }

  // Wireless Receiver - rx1
  std::string rx1ModelName = "rx1";
  std::string rx1SensorName = "wirelessReceiver";
  math::Pose rx1Pose(math::Vector3(0, 2, 0.055),
      math::Quaternion(0, 0, 0));

  // Spawn rx1
  SpawnWirelessReceiverSensor(rx1ModelName, rx1SensorName, rx1Pose.pos,
      rx1Pose.rot.GetAsEuler(), this->FREQ_FROM, this->FREQ_TO, this->POWER,
      this->GAIN, this->SENSITIVITY);

  sensors::WirelessReceiverPtr rx1 =
    boost::static_pointer_cast<sensors::WirelessReceiver>(
        sensors::SensorManager::Instance()->GetSensor(rx1SensorName));

  ASSERT_TRUE(rx1);

  // Initialize gazebo transport layer
  transport::NodePtr node(new transport::Node());
  node->Init("default");

  std::string rx1Topic = "/gazebo/default/rx1/link/wirelessReceiver/transceiver";
  transport::SubscriberPtr sub = node->Subscribe(rx1Topic, 
                                                 &TransceiverTest::RxMsg,
                                                 this);
  this->receivedMsg = false;

  while (true)
  {
    // Update all the transmitter sensors
    BOOST_FOREACH(const trans_map_type::value_type& myPair, transmitters)
    {
      myPair.second->Update(true);
    }
    
    // Update the receiver sensor
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
        std::string essid = txNode.essid();
        EXPECT_EQ(transmitters[essid]->GetESSID(), essid);
        EXPECT_EQ(transmitters[essid]->GetFreq(), txNode.frequency());
        EXPECT_LE(txNode.signal_level(), 0);
        EXPECT_GE(txNode.signal_level(), rx1->GetSensitivity());
      }
      break;
    }
  }
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
  math::Pose txPose(math::Vector3(0, 0, 0.055),
      math::Quaternion(0, 0, 0));

  // Spawn tx
  SpawnWirelessTransmitterSensor(txModelName, txSensorName, txPose.pos,
      txPose.rot.GetAsEuler(), "osrf", 2450.0, this->POWER,
      this->GAIN);

  sensors::WirelessTransmitterPtr tx =
    boost::static_pointer_cast<sensors::WirelessTransmitter>(
        sensors::SensorManager::Instance()->GetSensor(txSensorName));

  ASSERT_TRUE(tx);

  // Wireless Receiver - rx1
  std::string rx1ModelName = "rx1";
  std::string rx1SensorName = "wirelessRx1";
  math::Pose rx1Pose(math::Vector3(3, 0, 0.055),
      math::Quaternion(0, 0, 0));

  // Spawn rx1
  SpawnWirelessReceiverSensor(rx1ModelName, rx1SensorName, rx1Pose.pos,
      rx1Pose.rot.GetAsEuler(), this->FREQ_FROM, this->FREQ_TO, this->POWER,
      this->GAIN, this->SENSITIVITY);

  sensors::WirelessReceiverPtr rx1 =
    boost::static_pointer_cast<sensors::WirelessReceiver>(
        sensors::SensorManager::Instance()->GetSensor(rx1SensorName));

  ASSERT_TRUE(rx1);

   // Wireless Receiver - rx2
  std::string rx2ModelName = "rx2";
  std::string rx2SensorName = "wirelessRx2";
  math::Pose rx2Pose(math::Vector3(-3, 0, 0.055),
      math::Quaternion(0, 0, 0));

  // Spawn rx2
  SpawnWirelessReceiverSensor(rx2ModelName, rx2SensorName, rx2Pose.pos,
      rx2Pose.rot.GetAsEuler(), this->FREQ_FROM, this->FREQ_TO, this->POWER,
      this->GAIN, this->SENSITIVITY);

  sensors::WirelessReceiverPtr rx2 =
    boost::static_pointer_cast<sensors::WirelessReceiver>(
        sensors::SensorManager::Instance()->GetSensor(rx2SensorName));

  ASSERT_TRUE(rx2);

  // Spawn an obstacle between the transmitter and the receiver
  SpawnBox("Box", math::Vector3(1, 1, 1), math::Vector3(-1.5, 0, 0.055),
                 math::Vector3(0, 0, 0), true);

  // Initialize gazebo transport layer
  transport::NodePtr node(new transport::Node());
  node->Init("default");
  
  std::string rx1Topic = "/gazebo/default/rx1/link/wirelessRx1/transceiver";
  transport::SubscriberPtr sub = node->Subscribe(rx1Topic, 
                                                 &TransceiverTest::RxMsg,
                                                 this);
  this->receivedMsg = false;

  while (samples < 10)
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
        std::string essid = txNode.essid();
        double signalLevel = txNode.signal_level();
        
        samples++;
        avgSignalLevelEmpty += signalLevel;
      }
    }
  }

  samples = 0;

  std::string rx2Topic = "/gazebo/default/rx2/link/wirelessRx2/transceiver";
  sub = node->Subscribe(rx2Topic, &TransceiverTest::RxMsg, this);

  this->receivedMsg = false;

  while (samples < 10)
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
        std::string essid = txNode.essid();
        double signalLevel = txNode.signal_level();

        samples++;
        avgSignalLevelObstacle += signalLevel;
      }
    }
  }
  EXPECT_GE(avgSignalLevelEmpty / samples, avgSignalLevelObstacle / samples);
}

/////////////////////////////////////////////////
TEST_F(TransceiverTest, EmptyWorldODE)
{
  TxRxEmptySpace("ode");
}

/////////////////////////////////////////////////
TEST_F(TransceiverTest, ObstacleODE)
{
  TxRxObstacle("ode");
}


#ifdef HAVE_BULLET
/////////////////////////////////////////////////
TEST_F(TransceiverTest, EmptyWorldBullet)
{
  TxRxEmptySpace("bullet");
}

/////////////////////////////////////////////////
TEST_F(TransceiverTest, ObstacleBullet)
{
  TxRxObstacle("bullet");
}
#endif

