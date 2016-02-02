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

#include <boost/thread.hpp>
#include "gazebo/test/ServerFixture.hh"
#include "RAMLibrary.hh"

using namespace gazebo;

class TransportStressTest : public ServerFixture
{
};

boost::mutex g_mutex;

unsigned int g_localPublishMessageCount = 0;
unsigned int g_localPublishCount = 0;
unsigned int g_totalExpectedMsgCount = 0;
common::Time g_localPublishEndTime;

void LocalPublishCB(ConstImagePtr & /*_msg*/)
{
  boost::mutex::scoped_lock lock(g_mutex);

  if (g_localPublishCount+1 >= g_totalExpectedMsgCount)
    g_localPublishEndTime = common::Time::GetWallTime();
  g_localPublishCount++;
}

/////////////////////////////////////////////////
// Test for local publication. This test will create a large image and
// publish it to a local subscriber. Serialization should be bypassed,
// resulting fast publication.
TEST_F(TransportStressTest, LocalPublish)
{
  Load("worlds/empty.world");

  // Number of messages to publish
  g_localPublishMessageCount = 1000000;

  /// Expected number of messages to receive
  g_totalExpectedMsgCount = g_localPublishMessageCount;

  // Reset the received message counter
  g_localPublishCount = 0;

  transport::NodePtr testNode = transport::NodePtr(new transport::Node());
  testNode->Init("default");

  transport::PublisherPtr pub = testNode->Advertise<msgs::Image>(
      "~/test/local_publish__", g_localPublishMessageCount);

  transport::SubscriberPtr sub = testNode->Subscribe("~/test/local_publish__",
      &LocalPublishCB);

  unsigned int width = 2048;
  unsigned int height = 2048;
  unsigned char *fakeData = new unsigned char[width * height];

  // Create a large image message with fake data
  msgs::Image fakeMsg;
  fakeMsg.set_width(width);
  fakeMsg.set_height(height);
  fakeMsg.set_pixel_format(0);
  fakeMsg.set_step(1);
  fakeMsg.set_data(fakeData, width*height);

  // Get the start time
  common::Time startTime = common::Time::GetWallTime();

  // Publish the messages many times
  for (unsigned int i = 0; i < g_localPublishMessageCount; ++i)
  {
    pub->Publish(fakeMsg);
  }
  gzmsg << "Publish Complete" << std::endl;

  // Wait for all the messages
  int waitCount = 0;
  while (g_localPublishCount < g_totalExpectedMsgCount && waitCount < 50)
  {
    common::Time::MSleep(1000);
    waitCount++;
  }

  // Time it took to publish the messages.
  common::Time diff = g_localPublishEndTime - startTime;

  EXPECT_LT(waitCount, 50);

  // Make sure we received all the messages.
  EXPECT_EQ(g_totalExpectedMsgCount, g_localPublishCount);

  // The total duration should always be very short.
  EXPECT_LT(diff.sec, g_localPublishMessageCount * 0.0008);

  // Out time time for human testing purposes
  gzmsg << "Time to publish " << g_localPublishCount  << " messages = "
    << diff << "\n";

  delete [] fakeData;
}

/////////////////////////////////////////////////
// Create a lot of nodes, each with a publisher and subscriber. Then send
// out a few large messages.
TEST_F(TransportStressTest, ManyNodes)
{
  Load("worlds/empty.world");

  // Storage for all the nodes, subscribers, and publishers
  std::list<transport::NodePtr> nodes;
  std::list<transport::SubscriberPtr> subs;
  std::list<transport::PublisherPtr> pubs;

  // The number of nodes to create.
  #ifdef USE_LOW_MEMORY_TESTS
    // 1k nodes publish 10 times needs about 400Mb of RAM
    unsigned int nodeCount = 500;
    unsigned int requiredMB = 400;
  #else
    // 2k nodes publish 10 times needs about 1.7Gb of RAM
    unsigned int nodeCount = 1000;
    unsigned int requiredMB = 1700;
  #endif

  // Check if there is enough memory available
  if (!gazebo::test::memory::IsMemoryAvailable(requiredMB))
  {
    gzdbg << "Skipped test since " << requiredMB <<
              "Mb of RAM were not available \n";
    SUCCEED();
    return;
  }

  // The number of messages to send
  g_localPublishMessageCount = 10;

  // The expected number of messages to receive
  g_totalExpectedMsgCount = nodeCount * nodeCount * g_localPublishMessageCount;

  // Reset the received message counter
  g_localPublishCount = 0;

  // Create all the nodes.
  for (unsigned int i = 0; i < nodeCount; ++i)
  {
    nodes.push_back(transport::NodePtr(new transport::Node()));
    nodes.back()->Init();

    pubs.push_back(nodes.back()->Advertise<msgs::Image>(
          "~/test/local_publish2__", nodeCount * g_localPublishMessageCount));

    subs.push_back(nodes.back()->Subscribe("~/test/local_publish2__",
          &LocalPublishCB));
  }

  // Use a 2048x2048 image as the message
  unsigned int width = 2048;
  unsigned int height = 2048;
  unsigned char *fakeData = new unsigned char[width * height];

  // Create a large image message with fake data
  msgs::Image fakeMsg;
  fakeMsg.set_width(width);
  fakeMsg.set_height(height);
  fakeMsg.set_pixel_format(0);
  fakeMsg.set_step(1);
  fakeMsg.set_data(fakeData, width*height);

  // Get the start time
  common::Time startTime = common::Time::GetWallTime();

  // Publish the messages many times
  for (unsigned int i = 0; i < g_localPublishMessageCount; ++i)
  {
    for (std::list<transport::PublisherPtr>::iterator iter = pubs.begin();
        iter != pubs.end(); ++iter)
    {
      (*iter)->Publish(fakeMsg);
    }
  }
  common::Time publishTime = common::Time::GetWallTime();
  gzmsg << "Publish complete" << std::endl;

  // Wait for all the messages
  int waitCount = 0;
  while (g_localPublishCount < g_totalExpectedMsgCount && waitCount < 100)
  {
    common::Time::MSleep(1000);
    waitCount++;
  }

  // Time it took to publish the messages.
  common::Time pubDiff = publishTime - startTime;

  // Time it took to received the messages.
  common::Time receiveDiff = g_localPublishEndTime - startTime;

  EXPECT_LT(waitCount, 100);

  // Make sure we received all the messages.
  EXPECT_EQ(g_totalExpectedMsgCount, g_localPublishCount);

  // The total publish duration should always be very short.
  // The calculation here is the number of messages published multiplied by
  // the expected time to publish a single image message.
  // We used to check against 1 sec. However, the time can occasionally jump
  // depending on the system and its usage.
  EXPECT_LE(pubDiff.sec, 2);

  // The total receive duration will be longer.
  EXPECT_LT(receiveDiff.sec, g_localPublishCount * 1e-5);

  // Out time time for human testing purposes
  gzmsg << "Time to publish " << g_localPublishMessageCount * nodes.size()
    << " = " << pubDiff << std::endl;

  gzmsg << "Time to receive " << g_localPublishCount << " = "
    << receiveDiff << std::endl;

  delete [] fakeData;
}

/////////////////////////////////////////////////
// Main function
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
