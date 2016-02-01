/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include <unistd.h>
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;

class Issue1782Test : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(Issue1782Test, Unadvertise)
{
  this->Load("worlds/empty.world");

  // Check that transport is running and there are advertised topics
  EXPECT_FALSE(transport::is_stopped());
  EXPECT_TRUE(transport::ConnectionManager::Instance()->IsRunning());
  EXPECT_FALSE(transport::getAdvertisedTopics().empty());

  // Check that our topic is not advertised yet
  auto topics = transport::getAdvertisedTopics("gazebo.msgs.Vector3d");
  EXPECT_TRUE(topics.empty());

  // Initialize transport node
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  ASSERT_TRUE(node != NULL);

  std::string fullTopic = "/gazebo/" +  node->GetTopicNamespace() +
      "/test_topic";

  // Advertise two publishers to the same topic
  auto pubA = node->Advertise<msgs::Vector3d>(fullTopic);
  ASSERT_TRUE(pubA != NULL);

  auto pubB = node->Advertise<msgs::Vector3d>(fullTopic);
  ASSERT_TRUE(pubA != NULL);

  // Check that topic has been advertised
  topics = transport::getAdvertisedTopics("gazebo.msgs.Vector3d");
  int sleep = 0;
  int maxSleep = 10;
  while (topics.empty() && sleep < maxSleep)
  {
    topics = transport::getAdvertisedTopics("gazebo.msgs.Vector3d");
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_FALSE(topics.empty());
  EXPECT_TRUE(std::find(topics.begin(), topics.end(),
      fullTopic) != topics.end());

  // Finish pubB
  pubB->Fini();
  pubB.reset();

  // Check that topic is still advertised
  topics = transport::getAdvertisedTopics("gazebo.msgs.Vector3d");
  sleep = 0;
  while (!topics.empty() && sleep < maxSleep)
  {
    topics = transport::getAdvertisedTopics("gazebo.msgs.Vector3d");
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_FALSE(topics.empty());
  EXPECT_TRUE(std::find(topics.begin(), topics.end(),
      fullTopic) != topics.end());

  // Finish pubA
  pubA->Fini();
  pubA.reset();

  // Check that topic was unadvertised now that there are no publishers left
  topics = transport::getAdvertisedTopics("gazebo.msgs.Vector3d");
  sleep = 0;
  while (!topics.empty() && sleep < maxSleep)
  {
    topics = transport::getAdvertisedTopics("gazebo.msgs.Vector3d");
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_TRUE(topics.empty());
}

/////////////////////////////////////////////////
// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
