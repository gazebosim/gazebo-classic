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

#include <unistd.h>

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

bool g_worldStatsMsg2 = false;
bool g_sceneMsg = false;
bool g_worldStatsMsg = false;
bool g_worldStatsDebugMsg = false;

void ReceiveSceneMsg(ConstScenePtr &/*_msg*/)
{
  g_sceneMsg = true;
}

void ReceiveWorldStatsMsg(ConstWorldStatisticsPtr &/*_msg*/)
{
  g_worldStatsMsg = true;
}

void ReceiveWorldStatsMsg2(ConstWorldStatisticsPtr &/*_msg*/)
{
  printf("Receive world stats msgs 2\n");
  g_worldStatsMsg2 = true;
}

void ReceiveWorldStatsDebugMsg(ConstStringPtr &/*_data*/)
{
  g_worldStatsDebugMsg = true;
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

TEST_F(TransportTest, Processes)
{
  pid_t pid = fork();
  if (pid == 0)
  {
    usleep(1000000);
    printf("HERE\n");
    transport::init();
    transport::run();

    transport::NodePtr node(new transport::Node());
    node->Init();

    transport::SubscriberPtr sub =
      node->Subscribe("~/world_stats", &ReceiveWorldStatsMsg2);

    std::cout << "SubTopic[" << sub->GetTopic() << "]\n";

    int i = 0;
    while (!g_worldStatsMsg2 && i < 20)
    {
      std::cout << "Waiting for message\n";
      usleep(1000000);
      ++i;
    }
    EXPECT_LT(i, 20);

    transport::fini();
  }
  else if (pid < 0)
    printf("Fork failed\n");
  else
  {
    Load("worlds/empty.world");
    for (unsigned int i = 0; i < 20; ++i)
    {
      usleep(1000000);
    }
    printf("Now complete\n");
  }
}

TEST_F(TransportTest, Errors)
{
  Load("worlds/empty.world");
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init("default");
  transport::PublisherPtr scenePub;

  transport::SubscriberPtr statsSub =
    node->Subscribe("~/world_stats", &ReceiveWorldStatsMsg);
  EXPECT_STREQ("/gazebo/default/world_stats", statsSub->GetTopic().c_str());

  transport::SubscriberPtr statsSubDebug =
    node->Subscribe("~/world_stats/__dbg", &ReceiveWorldStatsDebugMsg);

  EXPECT_THROW(node->Advertise<math::Vector3>("~/scene"), common::Exception);

  scenePub = node->Advertise<msgs::Scene>("~/scene");
  EXPECT_THROW(node->Advertise<msgs::Factory>("~/scene"), common::Exception);
  EXPECT_TRUE(scenePub->GetPrevMsg().empty());

  transport::PublisherPtr factoryPub =
    node->Advertise<msgs::Factory>("~/factory");
  factoryPub->WaitForConnection();
  EXPECT_EQ(0, factoryPub->GetOutgoingCount());
  EXPECT_STREQ("/gazebo/default/factory", factoryPub->GetTopic().c_str());
  EXPECT_STREQ("gazebo.msgs.Factory", factoryPub->GetMsgType().c_str());

  EXPECT_STREQ("default", node->GetTopicNamespace().c_str());
  EXPECT_STREQ("/gazebo/default/factory",
               node->DecodeTopicName("~/factory").c_str());
  EXPECT_STREQ("~/factory",
               node->EncodeTopicName("/gazebo/default/factory").c_str());

  msgs::Scene sceneMsg;
  EXPECT_THROW(scenePub->Publish(sceneMsg), common::Exception);

  msgs::Factory factoryMsg;
  EXPECT_THROW(scenePub->Publish(factoryMsg), common::Exception);

  // Get the original URI
  char *uri = getenv("GAZEBO_MASTER_URI");
  std::string origURI = "GAZEBO_MASTER_URI=";
  if (uri)
    origURI += uri;

  int i = 0;
  while (!g_worldStatsMsg && !g_sceneMsg && !g_worldStatsDebugMsg && i < 20)
  {
    usleep(10000);
    ++i;
  }
  EXPECT_LT(i, 20);



  putenv(const_cast<char*>("GAZEBO_MASTER_URI="));
  std::string masterHost;
  unsigned int masterPort;
  EXPECT_FALSE(transport::get_master_uri(masterHost, masterPort));
  EXPECT_STREQ("localhost", masterHost.c_str());
  EXPECT_EQ(11345, masterPort);

  // restore original URI
  putenv(const_cast<char*>(origURI.c_str()));

  transport::clear_buffers();
  transport::pause_incoming(true);
  transport::TopicManager::Instance()->ProcessNodes();
  transport::pause_incoming(false);

  transport::stop();
  EXPECT_TRUE(transport::init(masterHost, masterPort));

  scenePub.reset();
  statsSub.reset();
  node.reset();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
