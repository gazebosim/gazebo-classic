/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <ignition/math/Color.hh>

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;

// Information to record for the lights in the enviornment.
// duration: The actual duration time of the flashlight.
// interval: The actual interval time of the flashlight.
// lastUpdate: The time at which the light was last checked.
struct RecordInfo
{
  double duration;
  double interval;
  std::vector<ignition::math::Color> colors;
  common::Time lastUpdate;
};

class FlashLightPluginTest : public ServerFixture
{
  // Constructor.
  public: FlashLightPluginTest(): called(false)
  {
  }

  // Destructor.
  public: ~FlashLightPluginTest()
  {
  }

  // Callback for light/modify topic
  public: void lightCb(ConstLightPtr &_msg);

  // Initialize all records.
  protected: void InitRec(int _num);

  // Check the records regarding blinking.
  protected: void CheckRecBlinking();

  // Find the color in the records.
  protected: bool FindColorInRec(const ignition::math::Color &_color);

  // An array of records
  protected: std::vector<RecordInfo> flashLight;

  // True if the callback function has been called.
  protected: bool called;

  // The time to start recording.
  protected: common::Time startTime;

  // The time to end recording.
  protected: common::Time endTime;

  // Mutex to protect data from racing.
  protected: std::mutex mutex;

  // Gazebo transport subscriber.
  protected: transport::SubscriberPtr sub;
};

//////////////////////////////////////////////////
void FlashLightPluginTest::lightCb(ConstLightPtr &_msg)
{
  // Determine which light is to be updated
  int indx;
  std::string name = _msg->name();
  std::stringstream ss;
  ss << name.substr(name.length() - 1);
  ss >> indx;
  indx--;

  EXPECT_FALSE(indx < 0 || static_cast<int>(this->flashLight.size()) < indx);

  // Get the current time
  common::Time currentTime = physics::get_world()->SimTime();


  // Update to flash
  std::lock_guard<std::mutex> lk(this->mutex);
  if (_msg->range() > 0)
  {
    this->flashLight[indx].interval
      = currentTime.Double() - this->flashLight[indx].lastUpdate.Double();
  }
  // Update to dim
  else
  {
    this->flashLight[indx].duration
      = currentTime.Double() - this->flashLight[indx].lastUpdate.Double();
  }
  // Update for the color
  if (_msg->has_diffuse())
  {
    ignition::math::Color color = msgs::Convert(_msg->diffuse());
    auto it = std::find(
      this->flashLight[indx].colors.begin(),
      this->flashLight[indx].colors.end(),
      color);
    if (it == this->flashLight[indx].colors.end())
    {
      this->flashLight[indx].colors.push_back(color);
    }
  }

  // Update the last update time
  this->flashLight[indx].lastUpdate = currentTime;

  this->called = true;
}

//////////////////////////////////////////////////
void FlashLightPluginTest::InitRec(int _num)
{
  std::lock_guard<std::mutex> lk(this->mutex);
  this->startTime = physics::get_world()->SimTime();
  this->flashLight.resize(_num);
  for (unsigned int i = 0; i < this->flashLight.size(); ++i)
  {
    this->flashLight[i].duration = -1;
    this->flashLight[i].interval = -1;
    this->flashLight[i].colors.resize(0);
    this->flashLight[i].lastUpdate = this->startTime;
  }
  this->called = false;
}

//////////////////////////////////////////////////
void FlashLightPluginTest::CheckRecBlinking()
{
  std::lock_guard<std::mutex> lk(this->mutex);
  this->endTime = physics::get_world()->SimTime();
  double lastUpdate0 = this->flashLight[0].lastUpdate.Double();
  double lastUpdate1 = this->flashLight[1].lastUpdate.Double();
  double lastUpdate2 = this->flashLight[2].lastUpdate.Double();
  double lastUpdate3 = this->flashLight[3].lastUpdate.Double();

  // Make sure if the function was called
  EXPECT_TRUE(this->called);

  // Verify only the supposed lights are updated
  // NOTE: Taking some errors caused by callback functions into consideration.
  //       Here, it is considered to be passed if the error is less than or
  //       equal to 0.01 sec.
  // NOTE: The first and second must have been updated within their phases.
  EXPECT_GE(lastUpdate0, this->endTime.Double() - 0.41);
  EXPECT_GE(lastUpdate1, this->endTime.Double() - 0.06);
  // NOTE: It is supposed to stop updating the third and forth lights just
  //       after the beginning.
  EXPECT_LE(lastUpdate2, this->startTime.Double() + 0.01);
  EXPECT_LE(lastUpdate3, this->startTime.Double() + 0.01);

  // Verify the lights blinking at the supposed duration and interval.
  // NOTE: Taking some errors caused by callback functions into consideration.
  //       Here, it is considered to be passed if the error is less than or
  //       equal to 0.01 sec.
  EXPECT_NEAR(this->flashLight[0].duration, 0.1, 0.01);
  EXPECT_NEAR(this->flashLight[0].interval, 0.4, 0.01);
  EXPECT_NEAR(this->flashLight[1].duration, 0.05, 0.01);
  EXPECT_NEAR(this->flashLight[1].interval, 0.05, 0.01);
}

//////////////////////////////////////////////////
bool FlashLightPluginTest::FindColorInRec(const ignition::math::Color &_color)
{
  auto it = std::find(
    this->flashLight[0].colors.begin(),
    this->flashLight[0].colors.end(), _color);

  return it != this->flashLight[0].colors.end();
}

//////////////////////////////////////////////////
TEST_F(FlashLightPluginTest, blinkingCheck)
{
  this->Load("test/worlds/flash_light_blinking_test.world");
  auto world = physics::get_world();
  ASSERT_NE(nullptr, world);

  // Get models
  auto model = world->ModelByName("light_model");
  ASSERT_NE(nullptr, model);

  // Initialize the time in the records
  this->InitRec(4);

  // Let the plugin blink the lights for a while
  common::Time::MSleep(1000);

  {
    std::lock_guard<std::mutex> lk(this->mutex);
    // Make sure that the function has never been called
    EXPECT_FALSE(this->called);
    // Make sure that they has never been updated.
    EXPECT_EQ(
      this->flashLight[0].lastUpdate.Double(), this->startTime.Double());
    EXPECT_EQ(
      this->flashLight[1].lastUpdate.Double(), this->startTime.Double());
    EXPECT_EQ(
      this->flashLight[2].lastUpdate.Double(), this->startTime.Double());
    EXPECT_EQ(
      this->flashLight[3].lastUpdate.Double(), this->startTime.Double());
  }

  // Subscribe to plugin notifications
  this->sub
    = this->node->Subscribe("~/light/modify", &FlashLightPluginTest::lightCb,
                            dynamic_cast<FlashLightPluginTest*>(this));

  // Initialize the records
  this->InitRec(4);

  // Let the plugin blink the lights for a while
  common::Time::MSleep(2000);

  // Check the records
  this->CheckRecBlinking();

  // The second trial.
  this->InitRec(4);

  common::Time::MSleep(2000);

  this->CheckRecBlinking();
}

//////////////////////////////////////////////////
TEST_F(FlashLightPluginTest, multiBlockCheck)
{
  this->Load("test/worlds/flash_light_multiblock_test.world");
  auto world = physics::get_world();
  ASSERT_NE(nullptr, world);

  // Get models
  auto model = world->ModelByName("light_model");
  ASSERT_NE(nullptr, model);

  // Initialize the time in the records
  this->InitRec(1);

  // Subscribe to plugin notifications
  this->sub
    = this->node->Subscribe("~/light/modify", &FlashLightPluginTest::lightCb,
                            dynamic_cast<FlashLightPluginTest*>(this));

  // Let the plugin blink the lights for a while
  common::Time::MSleep(2000);

  // Expect red, green, and blue lights have flashed.
  EXPECT_TRUE(this->FindColorInRec(ignition::math::Color::Red))
    << "Red is not found in the records." << std::endl;

  EXPECT_TRUE(this->FindColorInRec(ignition::math::Color::Blue))
    << "Blue is not found in the records." << std::endl;

  EXPECT_TRUE(this->FindColorInRec(ignition::math::Color::Green))
    << "Green is not found in the records." << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
