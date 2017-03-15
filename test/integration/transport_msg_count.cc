/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gtest/gtest.h>
#include <unistd.h>
#include <iostream>

using namespace gazebo;

class TransportMsgCountTest: public ::testing::Test
{
protected:

  TransportMsgCountTest()
    :fakeProgramName("TransportMsgCountTest")
  {}
  virtual ~TransportMsgCountTest() {}

  virtual void SetUp()
  {
    // irrelevant to pass fake argv, so make an exception
    // and pass away constness, so that fakeProgramName can be
    // initialized easily in constructor.
    gazebo::setupServer(1, (char**)&fakeProgramName);
  }

  virtual void TearDown()
  {
    gazebo::shutdown();
  }

private:
  const char * fakeProgramName;
};


// number of pose messages received
int g_receivedPosesStamped = 0;

// callback counting pose messages received
void ReceivePosesStampedMsgCounter(ConstPosesStampedPtr &/*_msg*/)
{
  ++g_receivedPosesStamped;
}

// this test needs to be done manually at the moment because it requires
// you to start up a remote subscriber connection.
// Once a workaround has been found to simulate a remote connection, this
// can be added as automated test.
TEST_F(TransportMsgCountTest, ManualTest)
{
  // register a namespace
  transport::TopicManager::Instance()->RegisterTopicNamespace("ttest");

  msgs::PosesStamped msg;
  msgs::Init(msg, "test");

  transport::NodePtr node(new transport::Node());
  node->Init();

  // create the publisher
  int bufferSize = 10000;
  transport::PublisherPtr pub =
    node->Advertise<msgs::PosesStamped>("~/test", bufferSize);

  // create a subscriber

  // IMPORTANT: The test case error (error with the old code) will only be
  // triggered with remote connections (see code in Publisher.cc
  // and Publishing.cc)

  // workaround: start a new remote connection. At some point simulate
  // a remote connection here.
  std::cout<<"Run the following command in a new terminal:"<<std::endl;
  std::cout<<"gz topic -e /gazebo/ttest/test"<<std::endl;

  // wait for the connection
  pub->WaitForConnection();

  // connection received, continue.
  std::cout<<"Received remote connection."<<std::endl;

  // now create an additional subscriber which will count the
  // number of messages arrived.
  g_receivedPosesStamped = 0;
  transport::SubscriberPtr sceneSub =
    node->Subscribe("~/test", &ReceivePosesStampedMsgCounter);

  // The dead-end of message reception happens randomly due to multi-threading.
  // Send a large numer of messages to increase the chance that this happens.
  int numMsgs = 10000;
  // FIXME we still have a problem when the number of messages sent quickly
  // is greater than the buffer size
  numMsgs = std::max(numMsgs, bufferSize);
  for (int i = 0; i < numMsgs; ++i)
  {
    // std::cout<<"SENDING MESSAGE! "<<i<<"..."<<std::endl;
    msgs::Set(msg.mutable_time(), common::Time(i));
    // do a direct publishing in which the message should
    // be written out right away: set "blocking" parameter to true.
    pub->Publish(msg, true);
  }

  std::cout<<"Sent out all messages."<<std::endl;

  // Now we actually need to call SendMessage() at least once more
  // if not all outgoing messages were already sent.
  // This can happen because the actual sending of messages happens
  // asynchronously. The existing message buffer in Publisher has to be
  // regularly emptied with SendMessage() calls.
  // Trying more times than the actualy amount of left-over messages
  // would be an error though.
  int tries = 20; // pub->GetOutgoingCount();  20 is plenty for testing
  while ((pub->GetOutgoingCount() > 0) && (tries > 0))
  {
    std::cout << "Sending out " << pub->GetOutgoingCount()
              << " left-over messages. " << std::endl;
    pub->SendMessage();
    // NOTE: Tried to use ConnectionManager::Instance()->TriggerUpdate()
    // instead but that does not do the job!
    // transport::ConnectionManager::Instance()->TriggerUpdate();
    common::Time::MSleep(200);
    --tries;
  }

  // it is not very nice to sleep for even longer, but to be sure all messages
  // have arrived and the callback has been called, sleep for a max of
  // timeoutSecs seconds.
  static const int timeoutSecs = 10;
  double sleepTime = 0;
  while ((g_receivedPosesStamped != numMsgs) && (sleepTime < timeoutSecs))
  {
    // not very nice but sleep just to make sure
    // enough time has passed for all messages to arrive
    static const double sleepSecs = 1.0;
    common::Time::Sleep(sleepSecs);
    sleepTime += sleepSecs;
  }

  // Optional: Sleep a bit more if no time was slept, just to see the messages
  // also arriving in the remote subscriber (it will be interrupted if
  // the publisher gets destroyed after the test ends)
  if (sleepTime < 3) common::Time::Sleep(3);

  ASSERT_EQ(g_receivedPosesStamped, numMsgs)
    << "Have not received all messages.";
}
