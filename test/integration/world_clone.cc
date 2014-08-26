/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

class WorldClone : public ServerFixture
{
};

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
  EXPECT_EQ(_msg->cloned_uri(), "http://localhost:11346");
}

/////////////////////////////////////////////////
TEST_F(WorldClone, Clone)
{
  Load("worlds/camera.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world);

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
  msg.set_new_port(11346);
  serverControlPub->Publish(msg);

  // Wait some bit of time since the clone is not immediate.
  common::Time::MSleep(500);

  // Remove all the models from the original world.
  world->Clear();
  common::Time::MSleep(500);
  EXPECT_EQ(world->GetModelCount(), 0u);

  // Check that the original world does not contain the camera topics.
  std::string output = custom_exec_str("gz topic -l");
  EXPECT_EQ(output.find("/gazebo/default/camera/"), std::string::npos);

  // Change GAZEBO_MASTER_URI to be able to see the topics of the new server.
  setenv("GAZEBO_MASTER_URI", "http://localhost:11346", 1);

  // Check that the cloned world contains the camera topics.
  output = custom_exec_str("gz topic -l");
  EXPECT_NE(output.find("/gazebo/default/camera/"), std::string::npos);

  // Kill the cloned server.
  custom_exec_str("killall gzserver");
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
