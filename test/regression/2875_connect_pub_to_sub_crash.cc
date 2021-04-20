/*
 * Copyright (C) 2021 Amazon.com Inc or its affiliates. All rights reserved.
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

#include "gazebo/transport/Connection.hh"
#include "gazebo/transport/SubscriptionTransport.hh"
#include "gazebo/transport/TopicManager.hh"

using namespace gazebo;

class Issue2875Test : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(Issue2875Test, ConnectPubToSubDoesNotCrashForMissingPublication)
{
  this->Load("worlds/empty.world");
  const std::string topic = "/chatter";
  // Add a topic to the advertised topics list so that the search in FindPublication isn't
  // against an empty container.
  auto Publisher = transport::TopicManager::Instance()->Advertise("/other", "some_type", 1, 1);

  transport::ConnectionPtr conn(new transport::Connection());
  transport::SubscriptionTransportPtr subLink(new transport::SubscriptionTransport());
  subLink->Init(conn, false);
  // Expect that this doesn't die
  transport::TopicManager::Instance()->ConnectPubToSub(topic, subLink);
  // The topic does not exist - but the above call should not have crashed anyways.
  ASSERT_EQ(transport::TopicManager::Instance()->FindPublication(topic), nullptr);
}

/////////////////////////////////////////////////
// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
