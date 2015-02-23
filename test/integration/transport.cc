/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/transport/transport.hh"
#include "ServerFixture.hh"

using namespace gazebo;

class TransportTest : public ServerFixture
{
};

bool g_worldStatsMsg2 = false;
bool g_sceneMsg = false;
bool g_worldStatsMsg = false;
bool g_worldStatsDebugMsg = false;
bool g_stringMsg = false;
bool g_stringMsg2 = false;
bool g_stringMsg3 = false;
bool g_stringMsg4 = false;

void ReceiveStringMsg(ConstGzStringPtr &/*_msg*/)
{
  g_stringMsg = true;
}

void ReceiveStringMsg2(ConstGzStringPtr &/*_msg*/)
{
  g_stringMsg2 = true;
}

void ReceiveStringMsg3(ConstGzStringPtr &/*_msg*/)
{
  g_stringMsg3 = true;
}

void ReceiveStringMsg4(ConstGzStringPtr &/*_msg*/)
{
  g_stringMsg4 = true;
}

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
  g_worldStatsMsg2 = true;
}

void ReceiveWorldStatsDebugMsg(ConstGzStringPtr &/*_data*/)
{
  g_worldStatsDebugMsg = true;
}

/////////////////////////////////////////////////
TEST_F(TransportTest, Load)
{
  for (unsigned int i = 0; i < 2; ++i)
  {
    Load("worlds/empty.world");
    Unload();
  }
}

/////////////////////////////////////////////////
// Standard pub/sub
TEST_F(TransportTest, PubSub)
{
  Load("worlds/empty.world");

  transport::NodePtr localNode = transport::NodePtr(new transport::Node());
  localNode->Init();
  transport::PublisherPtr scenePub =
    localNode->Advertise<msgs::Scene>("~/scene");
  transport::SubscriberPtr sceneSub = localNode->Subscribe("~/scene",
      &ReceiveSceneMsg);

  msgs::Scene msg;
  msgs::Init(msg, "test");
  msg.set_name("default");

  scenePub->Publish(msg);

  std::vector<transport::PublisherPtr> pubs;
  std::vector<transport::SubscriberPtr> subs;

  for (unsigned int i = 0; i < 10; ++i)
  {
    pubs.push_back(localNode->Advertise<msgs::Scene>("~/scene"));
    subs.push_back(localNode->Subscribe("~/scene", &ReceiveSceneMsg));
    scenePub->Publish(msg);
  }

  pubs.clear();
  subs.clear();
}

/////////////////////////////////////////////////
TEST_F(TransportTest, DirectPublish)
{
  Load("worlds/empty.world");

  g_sceneMsg = false;

  msgs::Scene msg;
  msgs::Init(msg, "test");
  msg.set_name("default");

  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();

  transport::SubscriberPtr sceneSub = node->Subscribe("~/scene",
      &ReceiveSceneMsg);
  transport::publish<msgs::Scene>("~/scene", msg);

  // Not nice to time check here but 10 seconds should be 'safe' to check
  // against
  int timeout = 1000;
  while (not g_sceneMsg)
  {
    common::Time::MSleep(10);

    timeout--;
    if (timeout == 0)
      break;
  }

  ASSERT_GT(timeout, 0) << "Not received a message in 10 seconds";
}

/////////////////////////////////////////////////
void SinglePub()
{
  transport::NodePtr node(new transport::Node());
  node->Init();

  transport::PublisherPtr pub = node->Advertise<msgs::GzString>("~/test");

  msgs::GzString msg;
  msg.set_data("Child process sending message.");
  pub->Publish(msg);
}


/////////////////////////////////////////////////
// This test creates a child process to test interprocess communication
TEST_F(TransportTest, ThreadedSinglePubSub)
{
  g_stringMsg = false;

  Load("worlds/empty.world");

  transport::NodePtr node(new transport::Node());
  node->Init();

  transport::SubscriberPtr sub =
    node->Subscribe("~/test", &ReceiveStringMsg, true);

  EXPECT_STREQ("gazebo.msgs.GzString",
      node->GetMsgType("/gazebo/default/test").c_str());

  boost::thread *thread = new boost::thread(boost::bind(&SinglePub));

  for (int i = 0; i < 10 && !g_stringMsg; ++i)
    common::Time::MSleep(100);

  EXPECT_TRUE(g_stringMsg);

  thread->join();
}

/////////////////////////////////////////////////
TEST_F(TransportTest, ThreadedMultiSubSinglePub)
{
  g_stringMsg = false;
  g_stringMsg2 = false;

  Load("worlds/empty.world");

  transport::NodePtr node(new transport::Node());
  node->Init();

  transport::SubscriberPtr sub =
    node->Subscribe("~/test", &ReceiveStringMsg, true);

  transport::SubscriberPtr sub2 =
    node->Subscribe("~/test", &ReceiveStringMsg2, true);

  EXPECT_STREQ("gazebo.msgs.GzString",
      node->GetMsgType("/gazebo/default/test").c_str());

  boost::thread *thread = new boost::thread(boost::bind(&SinglePub));

  for (int i = 0; i < 10 && !g_stringMsg && !g_stringMsg2; ++i)
    common::Time::MSleep(100);

  EXPECT_TRUE(g_stringMsg);

  thread->join();
}

/////////////////////////////////////////////////
void MultiPub()
{
  transport::NodePtr node(new transport::Node());
  node->Init();

  transport::PublisherPtr pub = node->Advertise<msgs::GzString>("~/test");
  transport::PublisherPtr pub2 = node->Advertise<msgs::GzString>("~/test");

  msgs::GzString msg;
  msg.set_data("Child process sending message.");
  pub->Publish(msg);
  pub2->Publish(msg);
}

/////////////////////////////////////////////////
TEST_F(TransportTest, ThreadedMultiPubSub)
{
  g_stringMsg = false;
  g_stringMsg2 = false;

  Load("worlds/empty.world");

  transport::NodePtr node(new transport::Node());
  node->Init();

  transport::SubscriberPtr sub =
    node->Subscribe("~/test", &ReceiveStringMsg, true);

  transport::SubscriberPtr sub2 =
    node->Subscribe("~/test", &ReceiveStringMsg2, true);

  EXPECT_STREQ("gazebo.msgs.GzString",
      node->GetMsgType("/gazebo/default/test").c_str());

  boost::thread *thread = new boost::thread(boost::bind(&MultiPub));

  for (int i = 0; i < 10 && !g_stringMsg && !g_stringMsg2; ++i)
    common::Time::MSleep(100);

  EXPECT_TRUE(g_stringMsg);

  thread->join();
}

/////////////////////////////////////////////////
void MultiPubSub()
{
  transport::NodePtr node(new transport::Node());
  node->Init("default");

  transport::PublisherPtr pub = node->Advertise<msgs::GzString>("~/test");
  transport::PublisherPtr pub2 = node->Advertise<msgs::GzString>("~/test");

  transport::SubscriberPtr sub =
    node->Subscribe("~/testO", &ReceiveStringMsg3, true);

  transport::SubscriberPtr sub2 =
    node->Subscribe("~/testO", &ReceiveStringMsg4, true);

  EXPECT_STREQ("gazebo.msgs.GzString",
      node->GetMsgType("/gazebo/default/testO").c_str());

  msgs::GzString msg;
  msg.set_data("Child process sending message.");
  pub->Publish(msg);
  pub2->Publish(msg);

  int i = 0;
  for (; i < 10 && (!g_stringMsg3 && !g_stringMsg4); ++i)
    common::Time::MSleep(100);

  EXPECT_TRUE(g_stringMsg3);
  EXPECT_TRUE(g_stringMsg4);
}

/////////////////////////////////////////////////
TEST_F(TransportTest, ThreadedMultiPubSubBidirectional)
{
  Load("worlds/empty.world");

  g_stringMsg = false;
  g_stringMsg2 = false;
  g_stringMsg3 = false;
  g_stringMsg4 = false;

  transport::NodePtr node(new transport::Node());
  node->Init("default");

  transport::SubscriberPtr sub =
    node->Subscribe("~/test", &ReceiveStringMsg, true);
  transport::SubscriberPtr sub2 =
    node->Subscribe("~/test", &ReceiveStringMsg2, true);

  transport::PublisherPtr pub = node->Advertise<msgs::GzString>("~/testO");
  transport::PublisherPtr pub2 = node->Advertise<msgs::GzString>("~/testO");

  EXPECT_STREQ("gazebo.msgs.GzString",
      node->GetMsgType("/gazebo/default/test").c_str());

  msgs::GzString msg;
  msg.set_data("Parent send message");
  pub->Publish(msg);
  pub2->Publish(msg);

  boost::thread *thread = new boost::thread(boost::bind(&MultiPubSub));

  for (int i = 0; i < 10 && !g_stringMsg && !g_stringMsg2; ++i)
    common::Time::MSleep(100);

  EXPECT_TRUE(g_stringMsg);
  EXPECT_TRUE(g_stringMsg2);

  thread->join();
}

/////////////////////////////////////////////////
TEST_F(TransportTest, PublicationTransportNoConnection)
{
  Load("worlds/empty.world");
  transport::PublicationTransport pubTransport("~/no_topic", "msg::Scene");
  ASSERT_EQ("~/no_topic", pubTransport.GetTopic());
  ASSERT_EQ("msg::Scene", pubTransport.GetMsgType());

  ASSERT_NO_THROW(pubTransport.Fini());
}

/////////////////////////////////////////////////
TEST_F(TransportTest, PublicationTransportFiniConnection)
{
  Load("worlds/empty.world");
  transport::PublicationTransport pubTransport("~/no_topic", "msg::Scene");
  ASSERT_EQ("~/no_topic", pubTransport.GetTopic());
  ASSERT_EQ("msg::Scene", pubTransport.GetMsgType());

  transport::ConnectionPtr conn(new transport::Connection);
  ASSERT_NO_THROW(pubTransport.Init(conn, false));

  ASSERT_NO_THROW(pubTransport.Fini());
}

/////////////////////////////////////////////////
TEST_F(TransportTest, IfaceGetMsgTyp)
{
  Load("worlds/empty.world");
  std::string type;

  type = transport::getTopicMsgType("/gazebo/default/world_stats");
  EXPECT_EQ(type, "gazebo.msgs.WorldStatistics");

  type = transport::getTopicMsgType("garbage");
  EXPECT_TRUE(type.empty());
}

/////////////////////////////////////////////////
TEST_F(TransportTest, IfaceGetTopicNameSpaces)
{
  Load("worlds/empty.world");
  std::list<std::string> ns;

  transport::get_topic_namespaces(ns);
  EXPECT_FALSE(ns.empty());
}

/////////////////////////////////////////////////
TEST_F(TransportTest, IfaceGetAdvertisedTopics)
{
  Load("worlds/empty.world");
  std::list<std::string> topics;

  topics = transport::getAdvertisedTopics("");
  EXPECT_FALSE(topics.empty());

  topics = transport::getAdvertisedTopics("no_msgs_of_this_type");
  EXPECT_TRUE(topics.empty());

  topics = transport::getAdvertisedTopics("gazebo.msgs.WorldStatistics");
  EXPECT_FALSE(topics.empty());
  EXPECT_EQ(topics.size(), 1u);

  topics = transport::getAdvertisedTopics("gazebo.msgs.PosesStamped");
  EXPECT_FALSE(topics.empty());
  EXPECT_EQ(topics.size(), 2u);

  std::map<std::string, std::list<std::string> > topicMap =
    transport::getAdvertisedTopics();

  EXPECT_FALSE(topics.empty());
  EXPECT_TRUE(topicMap.find("gazebo.msgs.PosesStamped") != topicMap.end());
}

/////////////////////////////////////////////////
// Test error cases
TEST_F(TransportTest, Errors)
{
  Load("worlds/empty.world");
  transport::NodePtr testNode = transport::NodePtr(new transport::Node());
  testNode->Init("default");
  transport::PublisherPtr scenePub;

  transport::SubscriberPtr localStatsSub =
    testNode->Subscribe("~/world_stats", &ReceiveWorldStatsMsg);
  EXPECT_STREQ("/gazebo/default/world_stats",
               localStatsSub->GetTopic().c_str());

  // This generates a warning message
  // EXPECT_THROW(testNode->Advertise<math::Vector3>("~/scene"),
  //             common::Exception);

  scenePub = testNode->Advertise<msgs::Scene>("~/scene");
  EXPECT_THROW(testNode->Advertise<msgs::Factory>("~/scene"),
               common::Exception);

  transport::PublisherPtr localFactoryPub =
    testNode->Advertise<msgs::Factory>("~/factory");
  localFactoryPub->WaitForConnection();
  EXPECT_EQ(static_cast<unsigned int>(0), localFactoryPub->GetOutgoingCount());
  EXPECT_STREQ("/gazebo/default/factory", localFactoryPub->GetTopic().c_str());
  EXPECT_STREQ("gazebo.msgs.Factory", localFactoryPub->GetMsgType().c_str());

  EXPECT_STREQ("default", testNode->GetTopicNamespace().c_str());
  EXPECT_STREQ("/gazebo/default/factory",
               testNode->DecodeTopicName("~/factory").c_str());
  EXPECT_STREQ("~/factory",
               testNode->EncodeTopicName("/gazebo/default/factory").c_str());

  // Get the original URI
  char *uri = getenv("GAZEBO_MASTER_URI");
  std::string origURI = "GAZEBO_MASTER_URI=";
  if (uri)
    origURI += uri;

  int i = 0;
  while (!g_worldStatsMsg && !g_sceneMsg && !g_worldStatsDebugMsg && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }
  EXPECT_LT(i, 20);

  putenv(const_cast<char*>("GAZEBO_MASTER_URI="));
  std::string masterHost;
  unsigned int masterPort;
  EXPECT_FALSE(transport::get_master_uri(masterHost, masterPort));
  EXPECT_STREQ("localhost", masterHost.c_str());
  EXPECT_EQ(static_cast<unsigned int>(11345), masterPort);

  // restore original URI
  putenv(const_cast<char*>(origURI.c_str()));

  transport::clear_buffers();
  transport::pause_incoming(true);
  transport::TopicManager::Instance()->ProcessNodes();
  transport::pause_incoming(false);

  transport::stop();
  EXPECT_TRUE(transport::init(masterHost, masterPort));

  scenePub.reset();
  localStatsSub.reset();
  testNode.reset();
}

/////////////////////////////////////////////////
// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
