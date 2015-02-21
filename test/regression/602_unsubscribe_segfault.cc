/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/JointController.hh"
#include "gazebo/common/PID.hh"
#include "test/ServerFixture.hh"
#include "test_config.h"

using namespace gazebo;
class Issue602Test : public ServerFixture
{
  /// \brief Callback for sensor subscribers in MultipleSensors test.
  private: void Callback(const ConstWorldStatisticsPtr &_msg);
};

unsigned int g_messageCount = 0;

////////////////////////////////////////////////////////////////////////
void Issue602Test::Callback(const ConstWorldStatisticsPtr &/*_msg*/)
{
  g_messageCount++;
}

/////////////////////////////////////////////////
TEST_F(Issue602Test, Unsubscribe)
{
  Load("worlds/empty.world");

  auto topics = transport::getAdvertisedTopics("gazebo.msgs.WorldStatistics");
  for (auto const &topic: topics)
  {
    gzdbg << "Listening to " << topic << std::endl;
    transport::SubscriberPtr sub = this->node->Subscribe(topic,
      &Issue602Test::Callback, this);

    common::Time::MSleep(500);
    EXPECT_GE(g_messageCount, 2u);

    sub->Unsubscribe();
    common::Time::MSleep(50);
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
