/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

using namespace gazebo;

class TransportTest : public ServerFixture
{
};

TEST_F(TransportTest, Load)
{
  for (unsigned int i = 0; i < 2; i++)
  {
    Load("worlds/empty.world");
    Unload();
  }
}

void ReceiveSceneMsg(ConstScenePtr &/*_msg*/)
{
}

TEST_F(TransportTest, PubSub)
{
  Load("worlds/empty.world");

  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  transport::PublisherPtr scenePub = node->Advertise<msgs::Scene>("~/scene");
  transport::SubscriberPtr sceneSub = node->Subscribe("~/scene",
      &ReceiveSceneMsg);

  msgs::Scene msg;
  msgs::Init(msg, "test");
  msg.set_name("default");

  scenePub->Publish(msg);

  std::vector<transport::PublisherPtr> pubs;
  std::vector<transport::SubscriberPtr> subs;

  for (unsigned int i = 0; i < 10; i++)
  {
    pubs.push_back(node->Advertise<msgs::Scene>("~/scene"));
    subs.push_back(node->Subscribe("~/scene", &ReceiveSceneMsg));
    scenePub->Publish(msg);
  }

  pubs.clear();
  subs.clear();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

