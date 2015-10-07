/*
 * Copyright (C) 2013-2015 Open Source Robotics Foundation
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
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/filesystem.hpp>

#include <gazebo/common/CommonIface.hh>
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
  while (!good and iters < 20)
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
TEST_F(gzTest, Alive)
{
  std::string rawOutput = custom_exec_str("gz");
  std::string helpOutput = custom_exec_str("gz help");

  EXPECT_FALSE(rawOutput.empty());
  EXPECT_FALSE(helpOutput.empty());
  EXPECT_EQ(rawOutput, helpOutput);
}

/////////////////////////////////////////////////
TEST_F(gzTest, Joint)
{
  init();

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
TEST_F(gzTest, Model)
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

  // Test model spawn from file
  {
    std::string filename = std::string(TEST_PATH) + "/models/box.sdf";

    waitForMsg("gz model -w default -m my_box -f " + filename);

    sdf::SDFPtr sdf(new sdf::SDF());
    EXPECT_TRUE(sdf::init(sdf));

    EXPECT_TRUE(sdf::readFile(filename, sdf));
    sdf::ElementPtr modelElem = sdf->root->GetElement("model");
    modelElem->GetAttribute("name")->SetFromString("my_box");

    gazebo::msgs::Factory msg;
    msg.set_sdf(sdf->ToString());
    gazebo::msgs::Set(msg.mutable_pose(), gazebo::math::Pose(0, 0, 0, 0, 0, 0));

    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  // Test model spawn from string
  {
    std::string filename = std::string(TEST_PATH) + "/models/box.sdf";

    std::string cmd = "cat ";
    cmd += filename + " | gz model -w default -m my_box -s";
    waitForMsg(cmd);

    sdf::SDFPtr sdf(new sdf::SDF());
    EXPECT_TRUE(sdf::init(sdf));

    EXPECT_TRUE(sdf::readFile(filename, sdf));
    sdf::ElementPtr modelElem = sdf->root->GetElement("model");
    modelElem->GetAttribute("name")->SetFromString("my_box");

    gazebo::msgs::Factory msg;
    msg.set_sdf(sdf->ToString());
    gazebo::msgs::Set(msg.mutable_pose(), gazebo::math::Pose(0, 0, 0, 0, 0, 0));

    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  // Test model info and pose
  {
    // Make sure the error message is output.
    std::string modelInfo = custom_exec_str("gz model -m does_not_exist -i");
    EXPECT_EQ(gazebo::common::get_sha1<std::string>(modelInfo),
        "7b5a9ab178ce5fa6ae74c80a33a99b84183ae600");

    // Get info for a model that exists.
    modelInfo = custom_exec_str("gz model -m my_box -i");

    // Check that a few values exist. We don't check the sha1 value
    // because a few values, such as pose, are dynamic.
    EXPECT_TRUE(modelInfo.find("name: \"my_box\"") != std::string::npos);
    EXPECT_TRUE(modelInfo.find("id: 9") != std::string::npos);
    EXPECT_TRUE(modelInfo.find("name: \"my_box::link::collision\"")
        != std::string::npos);

    // Get the pose of the model.
    modelInfo = custom_exec_str("gz model -m my_box -p");
    boost::algorithm::trim(modelInfo);

    // Split the string into parts p.
    std::vector<std::string> p;
    boost::split(p, modelInfo, boost::is_any_of(" "));

    // Make sure we have the right number of parts.
    // Don't ASSERT_EQ, because we need to run fini at end of test
    EXPECT_EQ(p.size(), 6u);

    // Make sure the pose is correct.
    if (p.size() == 6u)
    {
      EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(boost::lexical_cast<double>(p[0]), 0.0));
      EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(boost::lexical_cast<double>(p[1]), 0.0));
      EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(boost::lexical_cast<double>(p[2]), 0.5));
      EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(boost::lexical_cast<double>(p[3]), 0.0));
      EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(boost::lexical_cast<double>(p[4]), 0.0));
      EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(boost::lexical_cast<double>(p[5]), 0.0));
    }
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
TEST_F(gzTest, World)
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
TEST_F(gzTest, Physics)
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
TEST_F(gzTest, Camera)
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
    msg.set_follow_model("box");

    EXPECT_EQ(g_msgDebugOut, msg.DebugString());
  }

  fini();
}

/////////////////////////////////////////////////
TEST_F(gzTest, Stats)
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
TEST_F(gzTest, Topic)
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

  // Echo unformatted
  output = custom_exec_str("gz topic -e /gazebo/default/world_stats -u -d 1");
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
TEST_F(gzTest, SDF)
{
  boost::filesystem::path path;

  init();
  std::string helpOutput = custom_exec_str("gz help sdf");
  EXPECT_NE(helpOutput.find("gz sdf"), std::string::npos);

  // Regenerate each sum using:
  // gz sdf -d -v <major.minor> | sha1sum'
  std::map<std::string, std::string> descSums;
  descSums["1.0"] = "5235eb8464a96505c2a31fe96327d704e45c9cc4";
  descSums["1.2"] = "27973b2542d7a0f7582a615b245d81797718c89a";
  descSums["1.3"] = "30ffce1c662c17185d23f30ef3af5c110d367e10";
  descSums["1.4"] = "eb1798699f1926e6e75083970528c598bfa6d7f7";
  // descSums["1.5"] = "0c056ee3d5fdfb87f4d109d5161cac69a0387839";

  // Test each descSum
  for (std::map<std::string, std::string>::iterator iter = descSums.begin();
       iter != descSums.end(); ++iter)
  {
    std::string cmd = std::string("gz sdf -d -v ") + iter->first;
    std::string output = custom_exec_str(cmd);
    std::string shasum = gazebo::common::get_sha1<std::string>(output);
    EXPECT_EQ(shasum, iter->second);
  }

  // Regenerate each sum using:
  // gz sdf -o -v <major.minor> | sha1sum'
  std::map<std::string, std::string> docSums;
  docSums["1.0"] = "4cf955ada785adf72503744604ffadcdf13ec0d2";
  docSums["1.2"] = "f84c1cf1b1ba04ab4859e96f6aea881134fb5a9b";
  docSums["1.3"] = "f3dd699687c8922710e4492aadedd1c038d678c1";
  docSums["1.4"] = "31082d3b9fda88b1ac25588323e31c305937a548";
  // docSums["1.5"] = "1345996b27eb06f8d18cc4584b37f8ca4a6e8e69";

  // Test each docSum
  for (std::map<std::string, std::string>::iterator iter = docSums.begin();
       iter != docSums.end(); ++iter)
  {
    std::string cmd = std::string("gz sdf -o -v ") + iter->first;
    std::string output = custom_exec_str(cmd);
    std::string shasum = gazebo::common::get_sha1<std::string>(output);
    EXPECT_EQ(shasum, iter->second);
  }


  path = TEST_PATH;
  path /= "worlds/empty_different_name.world";

  {
    // Check empty.world
    std::string output =
      custom_exec_str(std::string("gz sdf -k ") + path.string());
    EXPECT_EQ(output, "Check complete\n");
  }

  {
    // Print empty.world
    // Regenerate using:
    // gz sdf -p test/worlds/empty_different_name.world
    // | sed ':a;N;$!ba;s/\n/\\n/g' | sed 's/"/\\"/g'
    std::string output =
      custom_exec_str(std::string("gz sdf -p ") + path.string());
    std::string shasum = gazebo::common::get_sha1<std::string>(output);
    EXPECT_EQ(shasum, "64dcab58596d2d2dbc596a37ee198bda58cc2b8a");
  }

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

  {
    // Convert 1.3 SDF
    std::string output =
      custom_exec_str(std::string("gz sdf -c ") + path.string());
    EXPECT_EQ(output, "Success\n");
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
