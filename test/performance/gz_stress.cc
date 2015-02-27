/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/filesystem.hpp>

#include <gazebo/common/CommonIface.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "test/util.hh"
#include "test_config.h"

class gzTest : public gazebo::testing::AutoLogFixture { };

std::string g_msgDebugOut;
boost::mutex g_mutex;
pid_t g_pid = -1;
boost::condition_variable g_msgCondition;

/////////////////////////////////////////////////
void WorldControlCB(ConstWorldControlPtr &_msg)
{
  boost::mutex::scoped_lock lock(g_mutex);
  g_msgDebugOut = _msg->DebugString();
  g_msgCondition.notify_all();
}

/////////////////////////////////////////////////
bool custom_exec(std::string _cmd)
{
  return system(_cmd.c_str()) >= 0;
}

/////////////////////////////////////////////////
std::string custom_exec_str(std::string _cmd)
{
  _cmd += " 2>&1";
  FILE *pipe = popen(_cmd.c_str(), "r");

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }

  pclose(pipe);
  return result;
}

/////////////////////////////////////////////////
void waitForMsg(const std::string &_cmd)
{
  boost::mutex::scoped_lock lock(g_mutex);
  g_msgDebugOut.clear();

  bool good = false;
  int iters = 0;
  while (!good && iters < 20)
  {
    custom_exec(_cmd);

    good = g_msgCondition.timed_wait(lock,
        boost::posix_time::milliseconds(1000));
    ++iters;
  }

  EXPECT_LT(iters, 20);
  EXPECT_TRUE(!g_msgDebugOut.empty());
}

/////////////////////////////////////////////////
void init()
{
  g_pid = fork();

  if (!g_pid)
  {
    boost::filesystem::path worldFilePath = TEST_PATH;
    worldFilePath = worldFilePath / "worlds" / "simple_arm_test.world";
    if (execlp("gzserver", worldFilePath.string().c_str(),
        "--iters", "60000", NULL) < 0)
    {
      gzerr << "Failed to start the gazebo server.\n";
    }
    return;
  }

  EXPECT_TRUE(gazebo::transport::init());
}

/////////////////////////////////////////////////
void fini()
{
  gazebo::transport::fini();
  if (kill(g_pid, SIGINT) < 0)
    gzerr << "Failed to kill the gazebo server.\n";

  int status;
  int p1 = 0;
  for (unsigned int i = 0; i < 5 && p1 != g_pid; ++i)
    p1 = waitpid(g_pid, &status, WNOHANG);
  if (p1 != g_pid)
  {
    kill(g_pid, SIGKILL);
    waitpid(g_pid, &status, 0);
  }

  g_pid = -1;
}

/////////////////////////////////////////////////
TEST_F(gzTest, Stress)
{
  init();

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::SubscriberPtr sub =
    node->Subscribe("~/world_control", &WorldControlCB, true);

  // Run the transport loop: starts a new thread
  gazebo::transport::run();

  // Test world reset time
  for (unsigned int i = 0; i < 100; ++i)
  {
    waitForMsg("gz world -w default -t");

    gazebo::msgs::WorldControl msg;
    msg.mutable_reset()->set_time_only(true);
    ASSERT_EQ(g_msgDebugOut, msg.DebugString());
  }

  fini();
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
