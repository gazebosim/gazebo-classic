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

#include <string>
#include <sstream>
#include <cmath>

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class FlashLightPluginTest : public ServerFixture
{
};

// information to record for the lights in the enviornment
struct RecordInfo
{
  double duration;
  double interval;
  common::Time last_update;
};

struct RecordInfo flash_light[4];
bool f_called;

void InitRec()
{
  common::Time current_time = physics::get_world()->SimTime();
  flash_light[0].duration = -1;
  flash_light[1].duration = -1;
  flash_light[2].duration = -1;
  flash_light[3].duration = -1;
  flash_light[0].interval = -1;
  flash_light[1].interval = -1;
  flash_light[2].interval = -1;
  flash_light[3].interval = -1;
  flash_light[0].last_update = current_time;
  flash_light[1].last_update = current_time;
  flash_light[2].last_update = current_time;
  flash_light[3].last_update = current_time;
}

//////////////////////////////////////////////////
// Callback for light/modify topic
void lightCb(ConstLightPtr &_msg)
{
  // Determine which light is to be updated
  int indx;
  std::string name = _msg->name();
  std::stringstream ss;
  ss << name.substr(name.length() - 1);
  ss >> indx;
  indx--;

  bool flag = true;
  if (indx < 0 || 3 < indx)
  {
    flag = false;
  }
  EXPECT_TRUE(flag);

  // Get the current time
  common::Time current_time = physics::get_world()->SimTime();

  // Update to flash
  if (_msg->range() > 0)
  {
    flash_light[indx].interval
      = current_time.Double() - flash_light[indx].last_update.Double();
  }
  // Update to dim
  else
  {
    flash_light[indx].duration
      = current_time.Double() - flash_light[indx].last_update.Double();
  }

  // Update the last update time
  flash_light[indx].last_update = current_time;

  f_called = true;
}

//////////////////////////////////////////////////
TEST_F(FlashLightPluginTest, Blinking)
{
  this->Load("worlds/flash_light_plugin_demo.world", true);
  auto world = physics::get_world();
  ASSERT_NE(nullptr, world);

  // Get models
  auto model = world->ModelByName("light_model");
  ASSERT_NE(nullptr, model);

  // Initialize the time in the records
  InitRec();

  // Subscribe to plugin notifications
  f_called = false;
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  transport::SubscriberPtr sceneSub
    = node->Subscribe("~/light/modify", &lightCb);

  // Let the plugin blink the lights for a while
  common::Time s_time = world->SimTime();
  flash_light[0].last_update = s_time;
  flash_light[1].last_update = s_time;
  flash_light[2].last_update = s_time;
  flash_light[3].last_update = s_time;
  world->Step(5000);
  common::Time e_time = world->SimTime();
  double last_update0 = flash_light[0].last_update.Double();
  double last_update1 = flash_light[1].last_update.Double();
  double last_update2 = flash_light[2].last_update.Double();
  double last_update3 = flash_light[3].last_update.Double();

  // Make sure if the function was called
  EXPECT_TRUE(f_called);

  // Verify only the supposed lights are updated
  // NOTE: Taking some errors caused by callback functions into consideration.
  //       Here, it is considered to be passed if the error is less than or
  //       equal to 0.01 sec.
  // NOTE: The first and second must have been updated within their phases.
  EXPECT_TRUE(fabs(last_update0 - e_time.Double()) <= 0.41);
  EXPECT_TRUE(fabs(last_update1 - e_time.Double()) <= 0.06);
  // NOTE: It is supposed to stop updating the third and forth lights just
  //       after the beginning.
  EXPECT_TRUE(fabs(last_update2 - s_time.Double()) <= 0.01);
  EXPECT_TRUE(fabs(last_update3 - s_time.Double()) <= 0.01);

  // Verify the lights blinking at the supposed duration and interval.
  // NOTE: Taking some errors caused by callback functions into consideration.
  //       Here, it is considered to be passed if the error is less than or
  //       equal to 0.01 sec.
  EXPECT_TRUE(fabs(flash_light[0].duration - 0.1) <= 0.01);
  EXPECT_TRUE(fabs(flash_light[0].interval - 0.4) <= 0.01);
  EXPECT_TRUE(fabs(flash_light[1].duration - 0.05) <= 0.01);
  EXPECT_TRUE(fabs(flash_light[1].interval - 0.05) <= 0.01);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
