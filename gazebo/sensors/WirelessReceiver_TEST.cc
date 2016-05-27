/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class WirelessReceiver_TEST : public ServerFixture
{
  public: static const std::string receiverSensorString;
  public: WirelessReceiver_TEST();
  public: void TestCreateWirelessReceiver();
  public: void TestIllegalTransceiver();
  public: void TestIllegalPower();
  public: void TestIllegalGain();
  public: void TestIllegalMinFreq();
  public: void TestIllegalMaxFreq();
  public: void TestIllegalMinMaxFreq();
  public: void TestIllegalSensitivity();
  public: void TestUpdateImpl();

  /// \brief Create a sensor with an illegal value and check that an exception
  /// is thrown
  /// \param[in] _sensorString Sensor SDF string
  private: void CheckIllegalValue(std::string _sensorString);

  /// \brief Create a sensor with legal values and check that an exception
  /// is not thrown
  /// \param[in] _sensorString Sensor SDF string
  private: void CheckLegalValue(std::string _sensorString);

  private: sensors::SensorManager *mgr;
  private: sdf::ElementPtr sdf;
};

const std::string WirelessReceiver_TEST::receiverSensorString =
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
WirelessReceiver_TEST::WirelessReceiver_TEST()
    :sdf(new sdf::Element)
{
  Load("worlds/empty.world");
  this->mgr = sensors::SensorManager::Instance();

  sdf::initFile("sensor.sdf", this->sdf);
}

/////////////////////////////////////////////////
/// \brief Test Creation of a wireless receiver sensor
void WirelessReceiver_TEST::TestCreateWirelessReceiver()
{
  sdf::readString(this->receiverSensorString, this->sdf);

  // Create the wireless receiver sensor
  std::string sensorName = this->mgr->CreateSensor(this->sdf, "default",
      "ground_plane::link", 0);

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName,
      std::string("default::ground_plane::link::wirelessReceiver"));

  // Update the sensor manager so that it can process new sensors.
  this->mgr->Update();

  // Get a pointer to the wireless receiver sensor
  sensors::WirelessReceiverPtr sensor =
    std::dynamic_pointer_cast<sensors::WirelessReceiver>(
        this->mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  ASSERT_TRUE(sensor != nullptr);

  EXPECT_DOUBLE_EQ(sensor->MinFreqFiltered(), 2412.0);
  EXPECT_DOUBLE_EQ(sensor->MaxFreqFiltered(), 2484.0);
  EXPECT_DOUBLE_EQ(sensor->Power(), 14.5);
  EXPECT_DOUBLE_EQ(sensor->Gain(), 2.5);
  EXPECT_DOUBLE_EQ(sensor->Sensitivity(), -90);

  EXPECT_TRUE(sensor->IsActive());
}

/////////////////////////////////////////////////
/// \brief Create a sensor with an illegal value and checks that an exception
/// is thrown
void WirelessReceiver_TEST::CheckIllegalValue(std::string _sensorString)
{
  sdf::readString(_sensorString, this->sdf);

  // Create the wireless receiver sensor
  ASSERT_ANY_THROW(this->mgr->CreateSensor(this->sdf,
      "default", "ground_plane::link", 0));
}

/////////////////////////////////////////////////
void WirelessReceiver_TEST::CheckLegalValue(std::string _sensorString)
{
  sdf::readString(_sensorString, this->sdf);

  // Create the wireless receiver sensor
  ASSERT_NO_THROW(this->mgr->CreateSensor(this->sdf,
      "default", "ground_plane::link", 0));
}

/////////////////////////////////////////////////
/// \brief Test Non-existent transceiver element
void WirelessReceiver_TEST::TestIllegalTransceiver()
{
  // Make a copy of the sdf string for avoid affecting other tests
  std::string receiverSensorStringCopy = this->receiverSensorString;
  boost::replace_first(receiverSensorStringCopy, "<transceiver>", "");
  boost::replace_first(receiverSensorStringCopy, "</transceiver>", "");

  this->CheckIllegalValue(receiverSensorStringCopy);
}

/////////////////////////////////////////////////
/// \brief Test wrong power value for the transceiver element
void WirelessReceiver_TEST::TestIllegalPower()
{
  // Replace the power by an incorrect value
  boost::regex re("<power>.*<\\/power>");
  std::string receiverSensorStringCopy =
      boost::regex_replace(this->receiverSensorString,
          re, "<power>-1.0</power>");

  this->CheckLegalValue(receiverSensorStringCopy);
}

/////////////////////////////////////////////////
/// \brief Test wrong gain value for the transceiver element
void WirelessReceiver_TEST::TestIllegalGain()
{
  // Replace the gain by an incorrect value
  boost::regex re("<gain>.*<\\/gain>");
  std::string receiverSensorStringCopy =
      boost::regex_replace(this->receiverSensorString, re, "<gain>-1.0</gain>");

  this->CheckLegalValue(receiverSensorStringCopy);
}

/////////////////////////////////////////////////
/// \brief Test wrong min_frequency value for the transceiver element
void WirelessReceiver_TEST::TestIllegalMinFreq()
{
  // Replace the min frequency by an incorrect value
  boost::regex re("<min_frequency>.*<\\/min_frequency>");
  std::string receiverSensorStringCopy =
      boost::regex_replace(this->receiverSensorString, re,
        "<min_frequency>-1.0</min_frequency>");

  this->CheckLegalValue(receiverSensorStringCopy);
}

/////////////////////////////////////////////////
/// \brief Test wrong max_frequency value for the transceiver element
void WirelessReceiver_TEST::TestIllegalMaxFreq()
{
  // Replace the max frequency by an incorrect value
  boost::regex re("<max_frequency>.*<\\/max_frequency>");
  std::string receiverSensorStringCopy =
      boost::regex_replace(this->receiverSensorString, re,
        "<max_frequency>-1.0</max_frequency>");

  this->CheckLegalValue(receiverSensorStringCopy);
}

/////////////////////////////////////////////////
/// \brief Test min_frequency value greater than max_frequency
void WirelessReceiver_TEST::TestIllegalMinMaxFreq()
{
  // Swap min_frequency and max_frequency
  boost::regex re("<max_frequency>.*<\\/max_frequency>");
  std::string receiverSensorStringCopy =
      boost::regex_replace(this->receiverSensorString, re,
        "<max_frequency>2412.0</max_frequency>");

  re = "<min_frequency>.*<\\/min_frequency>";
  receiverSensorStringCopy =
      boost::regex_replace(receiverSensorStringCopy, re,
        "<min_frequency>2484.0</min_frequency>");

  this->CheckLegalValue(receiverSensorStringCopy);
}

/////////////////////////////////////////////////
/// \brief Test wrong sensitivity value for the transceiver element
void WirelessReceiver_TEST::TestIllegalSensitivity()
{
  // Replace the sensitivity by an incorrect value
  boost::regex re("<sensitivity>.*<\\/sensitivity>");
  std::string receiverSensorStringCopy =
      boost::regex_replace(this->receiverSensorString, re,
        "<sensitivity>1.0</sensitivity>");

  this->CheckLegalValue(receiverSensorStringCopy);
}

/////////////////////////////////////////////////
/// \brief Test the updateImpl() method
void WirelessReceiver_TEST::TestUpdateImpl()
{
  sdf::readString(this->receiverSensorString, this->sdf);

  // Create the wireless receiver sensor
  std::string sensorName = this->mgr->CreateSensor(this->sdf, "default",
      "ground_plane::link", 0);

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName,
      std::string("default::ground_plane::link::wirelessReceiver"));

  // Update the sensor manager so that it can process new sensors.
  this->mgr->Update();

  // Get a pointer to the wireless receiver sensor
  sensors::WirelessReceiverPtr sensor =
    std::dynamic_pointer_cast<sensors::WirelessReceiver>(
        this->mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != nullptr);

  sensor->Update(true);
}

/////////////////////////////////////////////////
TEST_F(WirelessReceiver_TEST, TestCreateWilessReceiver)
{
  TestCreateWirelessReceiver();
}

/////////////////////////////////////////////////
TEST_F(WirelessReceiver_TEST, TestIllegalTransceiver)
{
  TestIllegalTransceiver();
}

/////////////////////////////////////////////////
TEST_F(WirelessReceiver_TEST, TestIllegalPower)
{
  TestIllegalPower();
}

/////////////////////////////////////////////////
TEST_F(WirelessReceiver_TEST, TestIllegalGain)
{
  TestIllegalGain();
}

/////////////////////////////////////////////////
TEST_F(WirelessReceiver_TEST, TestIllegalMinFreq)
{
  TestIllegalMinFreq();
}

/////////////////////////////////////////////////
TEST_F(WirelessReceiver_TEST, TestIllegalMaxFreq)
{
  TestIllegalMaxFreq();
}

/////////////////////////////////////////////////
TEST_F(WirelessReceiver_TEST, TestIllegalMinMaxFreq)
{
  TestIllegalMinMaxFreq();
}

/////////////////////////////////////////////////
TEST_F(WirelessReceiver_TEST, TestIllegalSensitivity)
{
  TestIllegalSensitivity();
}

/////////////////////////////////////////////////
TEST_F(WirelessReceiver_TEST, TestUpdateImpl)
{
  TestUpdateImpl();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
