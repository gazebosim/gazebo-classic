/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "test/data/pr2_state_log_expected.h"
#include "test/data/sdf_descriptions_expected.h"
#include "test_config.h"

std::string g_msgDebugOut;
boost::mutex g_mutex;
pid_t g_pid = -1;
boost::condition_variable g_msgCondition;

/////////////////////////////////////////////////
bool custom_exec(std::string _cmd)
{
  return system(_cmd.c_str()) >= 0;
}

/////////////////////////////////////////////////
std::string custom_exec_str(std::string _cmd)
{
  // _cmd += " 2>/dev/null";
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
  while (!good and iters < 10)
  {
    custom_exec(_cmd);

    good = g_msgCondition.timed_wait(lock,
        boost::posix_time::milliseconds(1000));
    ++iters;
  }

  EXPECT_LT(iters, 10);
  EXPECT_TRUE(!g_msgDebugOut.empty());
}

/////////////////////////////////////////////////
void init()
{
  g_pid = fork();

  if (!g_pid)
  {
    execlp("gzserver", "-q", "worlds/simple_arm.world", NULL);
    return;
  }

  EXPECT_TRUE(gazebo::transport::init());
}

/////////////////////////////////////////////////
void fini()
{
  gazebo::transport::fini();
  kill(g_pid, SIGINT);

  int status;
  int p1 = 0;
  for (unsigned int i = 0; i < 5 && p1 != g_pid; ++i)
    p1 = waitpid(g_pid, &status, WNOHANG);
  if (p1 != g_pid)
  {
    kill(g_pid, SIGKILL);
    p1 = waitpid(g_pid, &status, 0);
  }

  g_pid = -1;
}
/*
/////////////////////////////////////////////////
void JointCmdCB(ConstJointCmdPtr &_msg)
{
  boost::mutex::scoped_lock lock(g_mutex);
  g_msgDebugOut = _msg->DebugString();
  g_msgCondition.notify_all();
}

/////////////////////////////////////////////////
void ModelModifyCB(ConstModelPtr &_msg)
{
  boost::mutex::scoped_lock lock(g_mutex);
  g_msgDebugOut = _msg->DebugString();
  g_msgCondition.notify_all();
}

/////////////////////////////////////////////////
void RequestCB(ConstRequestPtr &_msg)
{
  boost::mutex::scoped_lock lock(g_mutex);
  g_msgDebugOut = _msg->DebugString();
  g_msgCondition.notify_all();
}

/////////////////////////////////////////////////
void FactoryCB(ConstFactoryPtr &_msg)
{
  boost::mutex::scoped_lock lock(g_mutex);
  g_msgDebugOut = _msg->DebugString();
  g_msgCondition.notify_all();
}

/////////////////////////////////////////////////
void WorldControlCB(ConstWorldControlPtr &_msg)
{
  boost::mutex::scoped_lock lock(g_mutex);
  g_msgDebugOut = _msg->DebugString();
  g_msgCondition.notify_all();
}

/////////////////////////////////////////////////
void PhysicsCB(ConstPhysicsPtr &_msg)
{
  boost::mutex::scoped_lock lock(g_mutex);
  g_msgDebugOut = _msg->DebugString();
  g_msgCondition.notify_all();
}

/////////////////////////////////////////////////
void CameraCB(ConstCameraCmdPtr &_msg)
{
  boost::mutex::scoped_lock lock(g_mutex);
  g_msgDebugOut = _msg->DebugString();
  g_msgCondition.notify_all();
}

/////////////////////////////////////////////////
/// Check to make sure that 'gz' exists
TEST(gz, Alive)
{
  std::string rawOutput = custom_exec_str("gz");
  std::string helpOutput = custom_exec_str("gz help");

  EXPECT_FALSE(rawOutput.empty());
  EXPECT_FALSE(helpOutput.empty());
  EXPECT_EQ(rawOutput, helpOutput);
}

/////////////////////////////////////////////////
TEST(gz, Joint)
{
  init();

  std::string expectedStr;

  std::string helpOutput = custom_exec_str("gz help joint");
  EXPECT_NE(helpOutput.find("gz joint"), std::string::npos);

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::SubscriberPtr sub =
    node->Subscribe("~/simple_arm/joint_cmd", &JointCmdCB);

  // Run the transport loop: starts a new thread
  gazebo::transport::run();

  // Test joint force
  {
    waitForMsg("gz joint -w default -m simple_arm "
        "-j arm_shoulder_pan_joint -f 10");

    gazebo::msgs::JointCmd msg;
    msg.set_name("simple_arm::arm_shoulder_pan_joint");
    msg.set_force(10);

    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  // Test joint position PID
  {
    waitForMsg("gz joint -w default -m simple_arm "
        "-j arm_shoulder_pan_joint --pos-t 1.5707 --pos-p 1.2 "
        "--pos-i 0.01 --pos-d 0.2");

    gazebo::msgs::JointCmd msg;
    msg.set_name("simple_arm::arm_shoulder_pan_joint");
    msg.mutable_position()->set_target(1.5707);
    msg.mutable_position()->set_p_gain(1.2);
    msg.mutable_position()->set_i_gain(0.01);
    msg.mutable_position()->set_d_gain(0.2);

    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  // Test joint velocity PID
  {
    waitForMsg("gz joint -w default -m simple_arm "
        "-j arm_shoulder_pan_joint --vel-t 1.5707 --vel-p 1.2 "
        "--vel-i 0.01 --vel-d 0.2");

    gazebo::msgs::JointCmd msg;
    msg.set_name("simple_arm::arm_shoulder_pan_joint");
    msg.mutable_velocity()->set_target(1.5707);
    msg.mutable_velocity()->set_p_gain(1.2);
    msg.mutable_velocity()->set_i_gain(0.01);
    msg.mutable_velocity()->set_d_gain(0.2);

    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  fini();
}

/////////////////////////////////////////////////
TEST(gz, Model)
{
  init();

  std::string helpOutput = custom_exec_str("gz help model");
  EXPECT_NE(helpOutput.find("gz model"), std::string::npos);

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::SubscriberPtr subModify =
    node->Subscribe("~/model/modify", &ModelModifyCB);

  gazebo::transport::SubscriberPtr subRequest =
    node->Subscribe("~/request", &RequestCB);

  gazebo::transport::SubscriberPtr subFactory =
    node->Subscribe("~/factory", &FactoryCB);

  // Run the transport loop: starts a new thread
  gazebo::transport::run();

  // Test model move
  {
    waitForMsg("gz model -w default -m simple_arm "
        "-x 1.1 -y 2.3 -z 4.5 -R 0.1 -P 1.2 -Y 3.4");

    gazebo::msgs::Model msg;
    msg.set_name("simple_arm");
    gazebo::msgs::Set(msg.mutable_pose(),
        gazebo::math::Pose(1.1, 2.3, 4.5, 0.1, 1.2, 3.4));
    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  // Test model spawn
  {
    std::string filename = std::string(TEST_PATH) + "/models/box.sdf";

    waitForMsg("gz model -w default -m my_box -f " + filename);

    std::ifstream ifs(filename.c_str());
    EXPECT_TRUE(ifs);

    boost::shared_ptr<sdf::SDF> sdf(new sdf::SDF());
    EXPECT_TRUE(sdf::init(sdf));

    EXPECT_TRUE(sdf::readFile(filename, sdf));
    sdf::ElementPtr modelElem = sdf->root->GetElement("model");
    modelElem->GetAttribute("name")->SetFromString("my_box");

    gazebo::msgs::Factory msg;
    msg.set_sdf(sdf->ToString());
    gazebo::msgs::Set(msg.mutable_pose(), gazebo::math::Pose(0, 0, 0, 0, 0, 0));

    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  // Test model delete
  {
    waitForMsg("gz model -w default -m simple_arm -d");

    EXPECT_NE(g_msgDebugOut.find("entity_delete"), std::string::npos);
    EXPECT_NE(g_msgDebugOut.find("simple_arm"), std::string::npos);
  }

  fini();
}

/////////////////////////////////////////////////
TEST(gz, World)
{
  init();

  std::string helpOutput = custom_exec_str("gz help world");
  EXPECT_NE(helpOutput.find("gz world"), std::string::npos);

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::SubscriberPtr sub =
    node->Subscribe("~/world_control", &WorldControlCB);

  // Run the transport loop: starts a new thread
  gazebo::transport::run();

  // Test world pause
  {
    waitForMsg("gz world -w default -p 1");

    gazebo::msgs::WorldControl msg;
    msg.set_pause(true);
    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  // Test world step
  {
    waitForMsg("gz world -w default -s");

    gazebo::msgs::WorldControl msg;
    msg.set_step(true);
    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  // Test world multi-step
  {
    waitForMsg("gz world -w default -m 10");

    gazebo::msgs::WorldControl msg;
    msg.set_multi_step(10);
    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  // Test world reset all
  {
    waitForMsg("gz world -w default -r");

    gazebo::msgs::WorldControl msg;
    msg.mutable_reset()->set_all(true);
    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  // Test world reset time
  {
    waitForMsg("gz world -w default -t");

    gazebo::msgs::WorldControl msg;
    msg.mutable_reset()->set_time_only(true);
    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  // Test world reset models
  {
    waitForMsg("gz world -w default -o");

    gazebo::msgs::WorldControl msg;
    msg.mutable_reset()->set_model_only(true);
    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  fini();
}

/////////////////////////////////////////////////
TEST(gz, Physics)
{
  init();

  std::string helpOutput = custom_exec_str("gz help physics");
  EXPECT_NE(helpOutput.find("gz physics"), std::string::npos);

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::SubscriberPtr sub =
    node->Subscribe("~/physics", &PhysicsCB);

  // Run the transport loop: starts a new thread
  gazebo::transport::run();

  // Test gravity
  {
    waitForMsg("gz physics -w default -g 1,2,3 ");

    gazebo::msgs::Physics msg;
    msg.mutable_gravity()->set_x(1);
    msg.mutable_gravity()->set_y(2);
    msg.mutable_gravity()->set_z(3);

    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  // Test step size
  {
    waitForMsg("gz physics -w default -s 0.0123 ");

    gazebo::msgs::Physics msg;
    msg.set_max_step_size(0.0123);

    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  // Test iters
  {
    waitForMsg("gz physics -w default -i 561 ");

    gazebo::msgs::Physics msg;
    msg.set_iters(561);

    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  // Test update-rate
  {
    waitForMsg("gz physics -w default -u 1234 ");

    gazebo::msgs::Physics msg;
    msg.set_real_time_update_rate(1234);

    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  fini();
}

/////////////////////////////////////////////////
TEST(gz, Camera)
{
  init();

  std::string helpOutput = custom_exec_str("gz help camera");
  EXPECT_NE(helpOutput.find("gz camera"), std::string::npos);

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::SubscriberPtr sub =
    node->Subscribe("~/user/cmd", &CameraCB);

  // Run the transport loop: starts a new thread
  gazebo::transport::run();

  // Test follow
  {
    waitForMsg("gz camera -w default -c user -f box");

    gazebo::msgs::CameraCmd msg;
    msg.set_name("user");
    msg.set_follow_model("box");

    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  fini();
}

/////////////////////////////////////////////////
TEST(gz, Stats)
{
  init();

  std::string helpOutput = custom_exec_str("gz help stats");
  EXPECT_NE(helpOutput.find("gz stats"), std::string::npos);

  // Basic output
  std::string output = custom_exec_str("gz stats -d 1");
  EXPECT_NE(output.find("Factor["), std::string::npos);

  // Plot option
  output = custom_exec_str("gz stats -d 1 -p");
  EXPECT_NE(output.find("# real-time factor (percent),"), std::string::npos);

  fini();
}

/////////////////////////////////////////////////
TEST(gz, Topic)
{
  init();

  std::string helpOutput = custom_exec_str("gz help topic");
  EXPECT_NE(helpOutput.find("gz topic"), std::string::npos);

  // List 
  std::string output = custom_exec_str("gz topic -l");
  EXPECT_NE(output.find("/gazebo/default/world_stats"), std::string::npos);

  // Info
  output = custom_exec_str("gz topic -i /gazebo/default/world_stats");
  EXPECT_NE(output.find("gazebo.msgs.WorldStatistics"), std::string::npos);

  // Echo 
  output = custom_exec_str("gz topic -e /gazebo/default/world_stats -d 1");
  EXPECT_NE(output.find("real_time {"), std::string::npos);

  // Hz 
  output = custom_exec_str("gz topic -z /gazebo/default/world_stats -d 1");
  EXPECT_NE(output.find("Hz:"), std::string::npos);

  // Bw 
  output = custom_exec_str("gz topic -b /gazebo/default/world_stats -d 10");
  EXPECT_NE(output.find("Total["), std::string::npos);

 fini();
}

/////////////////////////////////////////////////
TEST(gz, Stress)
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
*/

/////////////////////////////////////////////////
TEST(gz, SDF)
{
  std::string output;
  boost::filesystem::path path;

  init();
  std::string helpOutput = custom_exec_str("gz help sdf");
  EXPECT_NE(helpOutput.find("gz sdf"), std::string::npos);

  // 1.0 description
  output = custom_exec_str("gz sdf -d -v 1.0");
  EXPECT_EQ(output, sdf_description_1_0);

  // 1.2 description
  output = custom_exec_str("gz sdf -d -v 1.2");
  EXPECT_EQ(output, sdf_description_1_2);

  // 1.3 description
  output = custom_exec_str("gz sdf -d -v 1.3");
  EXPECT_EQ(output, sdf_description_1_3);

  // 1.0 doc
  output = custom_exec_str("gz sdf -o -v 1.0");
  EXPECT_EQ(output, sdf_doc_1_0);

  // 1.2 doc
  output = custom_exec_str("gz sdf -o -v 1.2");
  EXPECT_EQ(output, sdf_doc_1_2);

  // 1.3 doc
  output = custom_exec_str("gz sdf -o -v 1.3");
  EXPECT_EQ(output, sdf_doc_1_3);

  path = TEST_PATH;
  path /= "worlds/empty_different_name.world";

  // Check empty.world
  output = custom_exec_str(std::string("gz sdf -k ") + path.string());
  EXPECT_EQ(output, "Check complete\n");

  // Print empty.world
  output = custom_exec_str(std::string("gz sdf -p ") + path.string());
  EXPECT_EQ(output, sdf_print_empty_world_different_name);

  path = PROJECT_BINARY_PATH;
  path = path / "test" / "sdf_convert_test.world";
  std::ofstream file(path.string().c_str(), std::ios::out);
  file << "<?xml version='1.0' ?>"
    "<sdf version='1.3'>"
    "<world name='default'>"
    "<include><uri>model://camera</uri></include>"
    "</world>"
    "</sdf>";
  file.close();

  // Convert 1.3 SDF
  output = custom_exec_str(std::string("gz sdf -c ") + path.string());
  EXPECT_EQ(output, "Success\n");

  fini();
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
