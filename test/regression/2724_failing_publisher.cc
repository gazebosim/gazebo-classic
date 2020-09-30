/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef _WIN32
#include <unistd.h>
#endif
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
using namespace gazebo::transport;

class FailingPublisherTest : public ServerFixture
{
};

class TestTransport : public CallbackHelper
{
  /////////////////////////////////////////////////
  public: virtual bool HandleData(const std::string &/*_newdata*/,
                                  boost::function<void(uint32_t)> _cb,
                                  uint32_t _id) override
  {
    _cb(_id);
    return true;
  }

  /////////////////////////////////////////////////
  public: virtual bool HandleMessage(MessagePtr /*_newMsg*/) override
  {
    return true;
  }

  /////////////////////////////////////////////////
  public: virtual bool IsLocal() const override
  {
    return false;
  }
};

int g_receivedMsgs = 0;

void RequestMsgCb(const ConstRequestPtr &/*_msg*/)
{
  ++g_receivedMsgs;
}

/////////////////////////////////////////////////
TEST_F(FailingPublisherTest, DirectPublish)
{
  Load("worlds/empty.world");

  // Initialize node
  NodePtr node(new Node());
  node->Init();

  // Create publisher and subscriber
  SubscriberPtr sub = node->Subscribe<msgs::Request>("~/request", &RequestMsgCb);
  PublisherPtr pub = node->Advertise<msgs::Request>("~/request");

  // Add a subscription in the topic publication
  auto publication = TopicManager::Instance()->FindPublication(pub->GetTopic());
  ASSERT_FALSE(publication == nullptr);
  boost::shared_ptr<TestTransport> subscription(new TestTransport());
  publication->AddSubscription(subscription);

  // Publish messages
  msgs::Request *msg = msgs::CreateRequest("entity_delete", "test_model");
  pub->Publish(*msg, true);
  pub->Publish(*msg, true);
  delete msg;

  // Wait for messages
  bool success = false;
  for (int i = 0; i < 500; ++i)
  {
    if (g_receivedMsgs == 2)
    {
      success = true;
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(success) << "Number of received messages is " << g_receivedMsgs;
}

/////////////////////////////////////////////////
// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
