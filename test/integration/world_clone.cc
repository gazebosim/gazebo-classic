/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

using namespace gazebo;
class WorldClone : public ServerFixture
{
};

bool worldCloned = false;

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
void OnWorldModify(ConstWorldModifyPtr &_msg)
{
  ASSERT_TRUE(_msg->has_cloned());
  EXPECT_TRUE(_msg->cloned());
  ASSERT_TRUE(_msg->has_cloned_uri());
  EXPECT_EQ(_msg->cloned_uri(), "http://localhost:11347");

  worldCloned = _msg->has_cloned_uri() && _msg->cloned();
}

/////////////////////////////////////////////////
void OnWorldModifyNoClone(ConstWorldModifyPtr &_msg)
{
  ASSERT_TRUE(_msg->has_cloned());
  EXPECT_FALSE(_msg->cloned());

  worldCloned = _msg->has_cloned_uri() && _msg->cloned();
}

/////////////////////////////////////////////////
TEST_F(WorldClone, CloneUnknownWorld)
{
  Load("worlds/camera.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  transport::NodePtr node(new transport::Node());
  node->Init();

  transport::PublisherPtr serverControlPub =
    node->Advertise<msgs::ServerControl>("/gazebo/server/control");

  transport::SubscriberPtr worldModSub = node->Subscribe("/gazebo/world/modify",
    &OnWorldModifyNoClone);

  // Clone the server programmatically.
  msgs::ServerControl msg;
  msg.set_save_world_name("UnknownWorld");
  msg.set_clone(true);
  msg.set_new_port(11346);
  serverControlPub->Publish(msg);

  // Wait until the response from our cloning request is ready.
  worldCloned = false;
  int retries = 0;
  while (!worldCloned && retries++ < 100)
    common::Time::MSleep(20);

  ASSERT_FALSE(worldCloned);

  // Save the value of GAZEBO_MASTER_URI.
  char* master = getenv("GAZEBO_MASTER_URI");

  // Change GAZEBO_MASTER_URI to be able to see the topics of the new server.
  setenv("GAZEBO_MASTER_URI", "http://localhost:11346", 1);

  // Check that the world was not cloned by looking for some topics.
  std::string output = custom_exec_str("gz topic -l");
  EXPECT_EQ(output.find("/gazebo/default/"), std::string::npos);

  // Restore GAZEBO_MASTER_URI
  if (master)
    setenv("GAZEBO_MASTER_URI", master, 1);
  // Use default if not available
  else
  {
    std::string port = "http://localhost:" +
        std::to_string(GAZEBO_DEFAULT_MASTER_PORT);
    setenv("GAZEBO_MASTER_URI", port.c_str(), 1);
  }
}

/////////////////////////////////////////////////
TEST_F(WorldClone, CloneEmptyPort)
{
  Load("worlds/camera.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  transport::NodePtr node(new transport::Node());
  node->Init();

  transport::PublisherPtr serverControlPub =
    node->Advertise<msgs::ServerControl>("/gazebo/server/control");

  transport::SubscriberPtr worldModSub = node->Subscribe("/gazebo/world/modify",
    &OnWorldModifyNoClone);

  // Clone the server programmatically.
  msgs::ServerControl msg;
  msg.set_save_world_name("");
  msg.set_clone(true);
  serverControlPub->Publish(msg);

  // Wait until the response from our cloning request is ready.
  worldCloned = false;
  int retries = 0;
  while (!worldCloned && retries++ < 100)
    common::Time::MSleep(20);

  ASSERT_FALSE(worldCloned);
}

/////////////////////////////////////////////////
TEST_F(WorldClone, Clone)
{
  Load("worlds/camera.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  transport::NodePtr node(new transport::Node());
  node->Init();

  transport::PublisherPtr serverControlPub =
    node->Advertise<msgs::ServerControl>("/gazebo/server/control");

  transport::SubscriberPtr worldModSub = node->Subscribe("/gazebo/world/modify",
    &OnWorldModify);

  // Clone the server programmatically.
  msgs::ServerControl msg;
  msg.set_save_world_name("");
  msg.set_clone(true);
  msg.set_new_port(11347);
  serverControlPub->Publish(msg);

  // Wait until the response from our cloning request is ready.
  worldCloned = false;
  int retries = 0;
  while (!worldCloned && retries++ < 100)
    common::Time::MSleep(20);

  ASSERT_TRUE(worldCloned);

  // Remove all the models from the original world.
  world->Clear();

  // Wait until the world is really cleared.
  retries = 0;
  while (world->GetModelCount() != 0u && retries++ < 100)
    common::Time::MSleep(20);

  ASSERT_EQ(world->GetModelCount(), 0u);
  common::Time::MSleep(500);

  // Check that the original world does not contain the camera topics.
  std::string output = custom_exec_str("gz topic -l");
  EXPECT_EQ(output.find("/gazebo/default/camera/"), std::string::npos);

  // Change GAZEBO_MASTER_URI to be able to see the topics of the new server.
  setenv("GAZEBO_MASTER_URI", "http://localhost:11347", 1);

  // Give cloned world enough time to initialize transport and then
  // check that it contains the camera topics.
  retries = 0;
  output = custom_exec_str("gz topic -l");
  while (output.find("/gazebo/default/camera/") == std::string::npos
      && retries++ < 100)
  {
    common::Time::MSleep(20);
    output = custom_exec_str("gz topic -l");
  }
  EXPECT_NE(output.find("/gazebo/default/camera/"), std::string::npos);

  // Kill the cloned server. In the case of no presence of gzserver ps will
  // return the own ps process so it probably will do nothing. No effect.
  // This should work on Linux and Mac
  std::string get_pid_cmd = "ps -A | grep -m1 gzserver | awk '{print $1}'";

  std::string pid = custom_exec_str(get_pid_cmd);
  if (pid == "ERROR")
    FAIL() << "Fail to execute " + get_pid_cmd;

  std::string result = custom_exec_str("kill -15 " + pid);
  if (result == "ERROR")
    FAIL() << "Fail to run kill -15 command";
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
