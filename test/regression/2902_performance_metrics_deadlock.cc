/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;

class Issue2902Test : public ServerFixture
{
  /// \brief Test with PR2 world run with --lockstep.
  public: void PR2Test();

  /// \brief Callback for performance_metrics subscriber.
  /// \param[in] _msg Performance Metrics message.
  public: void Callback(const ConstPerformanceMetricsPtr &_msg);
};

unsigned int g_messageCount = 0;

////////////////////////////////////////////////////////////////////////
void Issue2902Test::Callback(const ConstPerformanceMetricsPtr &/*_msg*/)
{
  g_messageCount++;
}

/////////////////////////////////////////////////
TEST_F(Issue2902Test, PR2)
{
  PR2Test();
}

////////////////////////////////////////////////////////////////////////
void Issue2902Test::PR2Test()
{
  this->LoadArgs(" --lockstep -u worlds/pr2.world");
  physics::WorldPtr world = physics::get_world();
  ASSERT_TRUE(world != NULL);

  // Check that transport is running and there are advertised topics
  EXPECT_FALSE(transport::is_stopped());
  EXPECT_TRUE(transport::ConnectionManager::Instance()->IsRunning());
  EXPECT_FALSE(transport::getAdvertisedTopics().empty());

  // Check that image, laser scan and contacts topics are advertised
  auto topics = transport::getAdvertisedTopics("gazebo.msgs.Contacts");
  EXPECT_EQ(13u, topics.size());

  topics = transport::getAdvertisedTopics("gazebo.msgs.ImageStamped");
  EXPECT_EQ(10u, topics.size());

  topics = transport::getAdvertisedTopics("gazebo.msgs.LaserScanStamped");
  EXPECT_EQ(2u, topics.size());

  gzdbg << "Step simulation before subscribing" << std::endl;
  world->Step(1000);
  gzdbg << " -- Completed simulation steps" << std::endl;

  // Initialize transport node
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  ASSERT_TRUE(node != NULL);

  // Subscribe to performance metrics
  std::string topicName = "/gazebo/performance_metrics";
  auto subscriber = node->Subscribe(topicName, &Issue2902Test::Callback, this);

  gzdbg << "Step simulation after subscribing (#2902 deadlock here)"
        << std::endl;
  world->Step(1000);
  gzdbg << " -- Completed simulation steps" << std::endl;
}

/////////////////////////////////////////////////
// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
