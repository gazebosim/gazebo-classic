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

#include <thread>

using namespace gazebo;

/**
 * \brief Test class for gtest which forks into parent and child process
 */
class TransportMsgCountTest: public ::testing::Test
{
 protected:

  TransportMsgCountTest()
    :fakeProgramName("TransportMsgCountTest"),
     pid(-1)
  {}
  virtual ~TransportMsgCountTest() {}

  virtual void SetUp()
  {
    pid = fork();
    if (pid > 0)
    {
      // parent process
      // irrelevant to pass fake argv, so make an exception
      // and pass away constness, so that fakeProgramName can be
      // initialized easily in constructor.
      gazebo::setupServer(1, (char**)&fakeProgramName);
    }
    else if (pid == 0)
    {
      // child process
      if (!gazebo::transport::init())
      {
        gzerr << "Unable to initialize transport.\n";
      }
      else
      {
        gazebo::transport::run();
      }
    }
  }

  virtual void TearDown()
  {
    KillChildProcess();
    gazebo::shutdown();
  }

 protected:

  bool IsParent()
  {
    return pid > 0;
  }

  bool IsChild()
  {
    return pid == 0;
  }

  bool ForkSuccess()
  {
    return pid >= 0;
  }

  // \brief can be used from the parent to check if the child is still running
  // \retval 1 child is still running
  // \retval 0 child is not running
  // \retval -1 this is not the parent process
  int ChildRunning()
  {
    if (!IsParent()) return -1;
    int child_status;
    // result will be 0 if child is still running
    pid_t result = waitpid(pid, &child_status, WNOHANG);
    if (result != 0)
    {
      std::cerr << "CHILD STOPPED" << std::endl;
    }
    return result == 0;
  }

 private:
  // kills the child process, if it's the parent process.
  void KillChildProcess()
  {
    if (IsParent())
    {
      kill(pid, SIGKILL);
    }
  }

  // \brief fake program name as argv for gazebo::setupServer()
  const char * fakeProgramName;

  // \brief PID for child process which will run the remote subscriber
  pid_t pid;
};


// number of pose messages received
int g_receivedPosesStamped = 0;

// callback counting pose messages received
void ReceivePosesStampedMsgCounter(ConstPosesStampedPtr &/*_msg*/)
{
  ++g_receivedPosesStamped;
}

TEST_F(TransportMsgCountTest, MsgCount)
{
  // only continue if the forking has succeeded
  ASSERT_EQ(ForkSuccess(), true);

  // The test case error (error with the old code) will only be
  // triggered with remote connections (see code in Publisher.cc
  // and Publication.cc). So create a remote subscriber with the child
  // process.
  if (IsChild())
  {
    // transport system has to have been successfully started for the
    // child process
    ASSERT_FALSE(gazebo::transport::is_stopped());

    transport::NodePtr nodeRemote(new transport::Node());
    nodeRemote->Init();
    // we can use the same callback method for the subscriber as the child
    // process global variable doesn't interfere with the parent process.
    transport::SubscriberPtr subRemote =
      nodeRemote->Subscribe("/gazebo/ttest/test",
                            &ReceivePosesStampedMsgCounter);
    // sleep forever until child process is killed
    while (true) common::Time::MSleep(500);
    return;
  }

  // at this point, it can only be the parent process
  ASSERT_TRUE(IsParent());
  ASSERT_EQ(ChildRunning(), 1);

  // size of the publisher buffer
  int bufferSize = 10000;

  // Start the publisher in the parent process
  transport::NodePtr node(new transport::Node());
  node->Init();

  // create the publisher
  transport::PublisherPtr pub =
    node->Advertise<msgs::PosesStamped>("/gazebo/ttest/test", bufferSize);

  // wait for the connection to the remote subscriber started in the
  // child process
  pub->WaitForConnection();

  // connection received, now we can continue with the test..
  std::cout << "Remote subscription count: "
            << pub->GetRemoteSubscriptionCount() << std::endl;
  ASSERT_GE(pub->GetRemoteSubscriptionCount(), 1u);

  // Create an additional subscriber which will count the
  // number of messages arrived.
  // This has to be a subscriber in the parent process itself, because the
  // remote subscriber may actually not get all the messages when we end
  // this test - this is because of the asynchronous nature of the transport
  // system itself. This test is designed to chedk that a local subscriber
  // also receives all the messages while there is a remote subscriber
  // at the same time. This test would fail before the bug fix in
  // pull request 2657 (https://bitbucket.org/osrf/gazebo/pull-requests/2657).
  transport::SubscriberPtr poseSub =
    node->Subscribe("/gazebo/ttest/test", &ReceivePosesStampedMsgCounter);

  msgs::PosesStamped msg;
  msgs::Init(msg, "test");

  // The "dead-end" of message reception happens randomly due to
  // multi-threading. So send a large numer of messages to increase the
  // chance that this happens.
  int numMsgs = 10000;
  // We still have a problem when the number of messages sent quickly
  // is greater than the buffer size, so allow number of messages
  // to be not larger than buffersize
  numMsgs = std::max(numMsgs, bufferSize);
  for (int i = 0; i < numMsgs; ++i)
  {
    msgs::Set(msg.mutable_time(), common::Time(i));
    // do a direct publishing in which the message should
    // be written out right away: set "blocking" parameter to true.
    pub->Publish(msg, true);
  }

  std::cout << "Have sent out all messages." << std::endl;

  // CAVEAT: Now we actually need to call SendMessage() at least once more
  // if not all outgoing messages were already sent.
  // This can happen because the actual sending of messages happens
  // asynchronously. The existing message buffer in Publisher has to be
  // regularly emptied with SendMessage() calls.
  // Trying more times than the actualy amount of left-over messages
  // would be an error though.
  // Could do pub->GetOutgoingCount(), but  20 is plenty for testing
  // because we also sleep in the loop
  int tries = 20;
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
