/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
int g_createdBeforePub = 0;
int g_noLatchCreatedAfterPub = 0;
int g_latchCreatedAfterPub = 0;
int g_latchCreatedAfterPub2 = 0;
int g_subBeforeClear = 0;
int g_subAfterClear = 0;

void ReceiveBeforeClear(ConstVector3dPtr &/*_msg*/)
{
  g_subBeforeClear++;
}

void ReceiveAfterClear(ConstVector3dPtr &/*_msg*/)
{
  g_subAfterClear++;
}

void ReceiveNoLatchCreatedAfterPub(ConstVector3dPtr &/*_msg*/)
{
  g_noLatchCreatedAfterPub++;
}

void ReceiveLatchCreatedAfterPub2(ConstVector3dPtr &/*_msg*/)
{
  g_latchCreatedAfterPub2++;
}

void ReceiveLatchCreatedAfterPub(ConstVector3dPtr &/*_msg*/)
{
  g_latchCreatedAfterPub++;
}

void ReceiveCreatedBeforePub(ConstVector3dPtr &/*_msg*/)
{
  g_createdBeforePub++;
}

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

  for (unsigned int i = 0; i < 10; ++i)
  {
    pubs.push_back(node->Advertise<msgs::Scene>("~/scene"));
    subs.push_back(node->Subscribe("~/scene", &ReceiveSceneMsg));
    scenePub->Publish(msg);
  }

  pubs.clear();
  subs.clear();
}

// Standard pub/sub using lambdas
TEST_F(TransportTest, PubSubNoncapturingLambda)
{
  Load("worlds/empty.world");

  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  transport::PublisherPtr scenePub = node->Advertise<msgs::Scene>("~/scene");
  transport::SubscriberPtr sceneSub = node->Subscribe<msgs::Scene>("~/scene",
      +[](ConstScenePtr & _msg) -> void {
      	g_sceneMsg=true;
	  }
  );
}

TEST_F(TransportTest, PubSubCapturingLambda)
{
  Load("worlds/empty.world");

  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  transport::PublisherPtr scenePub = node->Advertise<msgs::Scene>("~/scene");
  transport::SubscriberPtr sceneSub = node->Subscribe<msgs::Scene>("~/scene",
      [this](ConstScenePtr & _msg) -> void {
      	g_sceneMsg=true;
	  }
  );
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
  while (!g_sceneMsg)
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
  transport::PublicationTransportPtr pubTransport(
      new transport::PublicationTransport("~/no_topic", "msg::Scene"));
  ASSERT_EQ("~/no_topic", pubTransport->GetTopic());
  ASSERT_EQ("msg::Scene", pubTransport->GetMsgType());

  ASSERT_NO_THROW(pubTransport->Fini());
}

/////////////////////////////////////////////////
TEST_F(TransportTest, PublicationTransportFiniConnection)
{
  Load("worlds/empty.world");
  transport::PublicationTransportPtr pubTransport(
      new transport::PublicationTransport("~/no_topic", "msg::Scene"));
  ASSERT_EQ("~/no_topic", pubTransport->GetTopic());
  ASSERT_EQ("msg::Scene", pubTransport->GetMsgType());

  transport::ConnectionPtr conn(new transport::Connection);
  ASSERT_NO_THROW(pubTransport->Init(conn, false));

  ASSERT_NO_THROW(pubTransport->Fini());
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
// Test clearing buffers
TEST_F(TransportTest, ClearBuffers)
{
  this->Load("worlds/empty.world");

  // Check that transport is running and there are advertised topics
  EXPECT_FALSE(transport::is_stopped());
  EXPECT_TRUE(transport::ConnectionManager::Instance()->IsRunning());
  EXPECT_FALSE(transport::getAdvertisedTopics().empty());

  std::string fullTopic = "/gazebo/" +  node->GetTopicNamespace() +
      "/test_topic";

  // Initialize transport node
  auto node = transport::NodePtr(new transport::Node());
  node->Init();
  ASSERT_TRUE(node != NULL);

  // Advertise publisher
  auto pub = node->Advertise<msgs::Vector3d>(fullTopic);
  ASSERT_TRUE(pub != NULL);

  // Publish a message
  msgs::Vector3d msg;
  msg.set_x(1);
  msg.set_y(2);
  msg.set_z(3);
  pub->Publish(msg);

  // FIXME: Need to sleep a bit before creating the latched subscriber,
  // otherwise it might receive the message twice.
  common::Time::MSleep(100);

  // Check that no messages have been received in callback yet
  EXPECT_EQ(g_subBeforeClear, 0);

  // Create a latched subscriber after publishing
  auto subBeforeClear = node->Subscribe(fullTopic,
      &ReceiveBeforeClear, true);
  ASSERT_TRUE(subBeforeClear != NULL);

  // Check that message is in buffer, since it's received by latched subscriber
  int sleep = 0;
  int maxSleep = 30;
  while (g_subBeforeClear != 1 && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_EQ(g_subBeforeClear, 1);

  // Clear buffers
  transport::clear_buffers();

  // Check that transport is still running
  EXPECT_FALSE(transport::is_stopped());
  EXPECT_TRUE(transport::ConnectionManager::Instance()->IsRunning());
  EXPECT_FALSE(transport::getAdvertisedTopics().empty());

  // Create another latched subscriber after publishing
  auto subAfterClear = node->Subscribe(fullTopic, &ReceiveAfterClear, true);
  ASSERT_TRUE(subAfterClear != NULL);

  // Check that previous message is not received by new latched subscriber
  sleep = 0;
  while (g_subAfterClear != 1 && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_EQ(g_subBeforeClear, 1);
  EXPECT_EQ(g_subAfterClear, 0);

  pub.reset();
  subBeforeClear.reset();
  subAfterClear.reset();
  node.reset();
}

/////////////////////////////////////////////////
// Test latching
TEST_F(TransportTest, Latching)
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
  transport::NodePtr node1 = transport::NodePtr(new transport::Node());
  node1->Init();
  ASSERT_TRUE(node1 != NULL);

  std::string fullTopic = "/gazebo/" +  node1->GetTopicNamespace() +
      "/test_topic";

  // Create a non latched subscriber before publishing
  auto subCreatedBeforePub = node1->Subscribe(fullTopic,
      &ReceiveCreatedBeforePub);
  ASSERT_TRUE(subCreatedBeforePub != NULL);

  // Advertise publisher
  auto pub1 = node1->Advertise<msgs::Vector3d>(fullTopic);
  ASSERT_TRUE(pub1 != NULL);

  // Check that topic has been advertised
  topics = transport::getAdvertisedTopics("gazebo.msgs.Vector3d");
  int sleep = 0;
  int maxSleep = 30;
  while (topics.empty() && sleep < maxSleep)
  {
    topics = transport::getAdvertisedTopics("gazebo.msgs.Vector3d");
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_FALSE(topics.empty());
  EXPECT_TRUE(std::find(topics.begin(), topics.end(),
      fullTopic) != topics.end());

  // Check no message has been received by subCreatedBeforePub
  EXPECT_EQ(g_createdBeforePub, 0);

  // Publish a message
  msgs::Vector3d msg;
  msg.set_x(1);
  msg.set_y(2);
  msg.set_z(3);
  pub1->Publish(msg);

  // Wait for message to be received
  sleep = 0;
  while (g_createdBeforePub != 1 && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_EQ(g_createdBeforePub, 1);

  // Create a non latched subscriber after publishing
  auto subNoLatchCreatedAfterPub = node1->Subscribe(fullTopic,
      &ReceiveNoLatchCreatedAfterPub, false);
  ASSERT_TRUE(subNoLatchCreatedAfterPub != NULL);

  // Create a latched subscriber after publishing
  auto subLatchCreatedAfterPub = node1->Subscribe(fullTopic,
      &ReceiveLatchCreatedAfterPub, true);
  ASSERT_TRUE(subLatchCreatedAfterPub != NULL);

  // Wait for message to be received by latched subscriber
  sleep = 0;
  while (g_latchCreatedAfterPub != 1 && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_EQ(g_createdBeforePub, 1);
  EXPECT_EQ(g_latchCreatedAfterPub, 1);
  EXPECT_EQ(g_noLatchCreatedAfterPub, 0);

  // Create another latched subscriber after publishing
  auto subLatchCreatedAfterPub2 = node1->Subscribe(fullTopic,
      &ReceiveLatchCreatedAfterPub2, true);
  ASSERT_TRUE(subLatchCreatedAfterPub2 != NULL);

  // Wait for message to be received only by new latched subscriber
  sleep = 0;
  while (g_latchCreatedAfterPub2 != 1 && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_EQ(g_createdBeforePub, 1);
  EXPECT_EQ(g_latchCreatedAfterPub, 1);
  EXPECT_EQ(g_noLatchCreatedAfterPub, 0);
  EXPECT_EQ(g_latchCreatedAfterPub2, 1);

  // Publish another message
  msgs::Vector3d msg2;
  msg2.set_x(4);
  msg2.set_y(5);
  msg2.set_z(6);
  pub1->Publish(msg2);

  // Wait for message to be received by all subscribers
  sleep = 0;
  while (g_latchCreatedAfterPub2 != 2 && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_EQ(g_createdBeforePub, 2);
  EXPECT_EQ(g_latchCreatedAfterPub, 2);
  EXPECT_EQ(g_noLatchCreatedAfterPub, 1);
  EXPECT_EQ(g_latchCreatedAfterPub2, 2);

  // Initialize another transport node
  transport::NodePtr node2 = transport::NodePtr(new transport::Node());
  node2->Init();
  ASSERT_TRUE(node2 != NULL);

  // Advertise another publisher
  auto pub2 = node2->Advertise<msgs::Vector3d>(fullTopic);
  ASSERT_TRUE(pub2 != NULL);

  // Check that the subscribers didn't get latched messages again
  sleep = 0;
  while (sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_EQ(g_createdBeforePub, 2);
  EXPECT_EQ(g_latchCreatedAfterPub, 2);
  EXPECT_EQ(g_noLatchCreatedAfterPub, 1);
  EXPECT_EQ(g_latchCreatedAfterPub2, 2);

  // Publish with new publisher
  msgs::Vector3d msg3;
  msg3.set_x(4);
  msg3.set_y(5);
  msg3.set_z(6);
  pub2->Publish(msg3);

  // Wait for message to be received
  sleep = 0;
  while (g_createdBeforePub != 3 && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_EQ(g_createdBeforePub, 3);
  EXPECT_EQ(g_latchCreatedAfterPub, 3);
  EXPECT_EQ(g_noLatchCreatedAfterPub, 2);
  EXPECT_EQ(g_latchCreatedAfterPub2, 3);
}

/////////////////////////////////////////////////
// Test error cases
// This test must be run after all the others, because it messes up
// GAZEO_MASTER_URI
TEST_F(TransportTest, Errors)
{
  Load("worlds/empty.world");
  transport::NodePtr testNode = transport::NodePtr(new transport::Node());
  testNode->Init("default");
  transport::PublisherPtr scenePub;

  transport::SubscriberPtr statsSub =
    testNode->Subscribe("~/world_stats", &ReceiveWorldStatsMsg);
  EXPECT_STREQ("/gazebo/default/world_stats", statsSub->GetTopic().c_str());

  // This generates a warning message
  // EXPECT_THROW(testNode->Advertise<ignition::math::Vector3d>("~/scene"),
  //             common::Exception);

  scenePub = testNode->Advertise<msgs::Scene>("~/scene");
  EXPECT_THROW(testNode->Advertise<msgs::Factory>("~/scene"),
               common::Exception);

  transport::PublisherPtr factoryPub =
    testNode->Advertise<msgs::Factory>("~/factory");
  factoryPub->WaitForConnection();
  EXPECT_EQ(static_cast<unsigned int>(0), factoryPub->GetOutgoingCount());
  EXPECT_STREQ("/gazebo/default/factory", factoryPub->GetTopic().c_str());
  EXPECT_STREQ("gazebo.msgs.Factory", factoryPub->GetMsgType().c_str());

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
  statsSub.reset();
  testNode.reset();
}

/////////////////////////////////////////////////
TEST_F(TransportTest, TryInit)
{
  // If the ConnectionManager has not been initialized, then TryInit() is
  // certain to fail.
  transport::NodePtr node = transport::NodePtr(new transport::Node);
  EXPECT_FALSE(node->IsInitialized());
  EXPECT_FALSE(node->TryInit(common::Time(0.01)));
  EXPECT_FALSE(node->IsInitialized());

  // Loading the server will initialize the ConnectionManager
  this->Load("worlds/empty.world");

  // The server will initialize some Nodes, so a namespace will be available now
  EXPECT_FALSE(node->IsInitialized());
  EXPECT_TRUE(node->TryInit(common::Time(0.01)));
  EXPECT_TRUE(node->IsInitialized());

  // The namespace of the Node should match the name of the world that we loaded
  EXPECT_EQ(physics::get_world()->Name(), node->GetTopicNamespace());
}

/////////////////////////////////////////////////
// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
