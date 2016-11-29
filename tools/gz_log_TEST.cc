/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include <thread>
#include <gtest/gtest.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <gazebo/common/CommonIface.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/util/LogPlay.hh>
#include <sdf/sdf_config.h>

#include <stdio.h>
#include <string>

// This header file isn't needed if shasums are used
// #include "test/data/pr2_state_log_expected.h"
#include "test_config.h"
#include "gazebo/gazebo_config.h"

const std::string GZ_LOG_PATH(
    std::string(PROJECT_BINARY_PATH) + "/tools/gz log ");

std::string custom_exec(std::string _cmd)
{
  _cmd += " 2>/dev/null";
  FILE* pipe = popen(_cmd.c_str(), "r");

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
/// Check to make sure that 'gz log -i' returns correct information
TEST(gz_log, Info)
{
  std::string info = custom_exec(GZ_LOG_PATH + "-i -f" +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  boost::trim_right(info);

  std::string validInfo =
    "Log Version:    1.0\n"
    "Gazebo Version: 1.4.6\n"
    "Random Seed:    32606\n"
    // "Start:          Feb 08 13 05:35:55.667456998\n"
    // "End:            Feb 08 13 05:35:58.947304437\n"
    // "Duration:       00:00:03.279847439\n"
    // "Steps:          3\n"
    "Size:           12.377 KB\n"
    "Encoding:       bz2";
    // "Model Count:    2";

  EXPECT_EQ(validInfo, info);
}

/////////////////////////////////////////////////
/// Check to make sure that 'gz log -e' returns correct information
TEST(gz_log, Echo)
{
  std::string echo = custom_exec(std::string(GZ_LOG_PATH + "-e -f ") +
      PROJECT_SOURCE_PATH + "/test/data/empty_state.log");
  boost::trim_right(echo);

  std::string validEcho =
    "<?xml version='1.0'?>\n<gazebo_log>\n<header>\n<log_version>1.0"
    "</log_version>\n<gazebo_version>1.4.6</gazebo_version>\n"
    "<rand_seed>24794</rand_seed>\n<log_start>0 0</log_start>\n"
    "<log_end>0 0</log_end>\n</header>\n\n<chunk encoding='txt'>"
    "<![CDATA[\n<sdf version ='1.3'>\n<world name='default'>\n  "
    "<light name='sun' type='directional'>\n    <cast_shadows>1"
    "</cast_shadows>\n    <pose>0.000000 0.000000 10.000000 0.000000 0.000000 "
    "0.000000</pose>\n    <diffuse>0.800000 0.800000 0.800000 1.000000"
    "</diffuse>\n    <specular>0.100000 0.100000 0.100000 1.000000"
    "</specular>\n    <attenuation>\n      <range>1000.000000</range>\n      "
    "<constant>0.900000</constant>\n      <linear>0.010000</linear>\n      "
    "<quadratic>0.001000</quadratic>\n    </attenuation>\n    "
    "<direction>-0.500000 0.500000 -1.000000</direction>\n  </light>\n  "
    "<model name='ground_plane'>\n    <static>1</static>\n    "
    "<link name='link'>\n      <collision name='collision'>\n        "
    "<geometry>\n          <plane>\n            <normal>0.000000 0.000000 "
    "1.000000</normal>\n            <size>100.000000 100.000000"
    "</size>\n          </plane>\n        </geometry>\n        "
    "<surface>\n          <friction>\n            <ode>\n              "
    "<mu>100.000000</mu>\n              <mu2>50.000000</mu2>\n            "
    "</ode>\n          </friction>\n          <bounce/>\n          "
    "<contact>\n            <ode/>\n          </contact>\n        "
    "</surface>\n      </collision>\n      <visual name='visual'>\n        "
    "<cast_shadows>0</cast_shadows>\n        <geometry>\n          "
    "<plane>\n            <normal>0.000000 0.000000 1.000000"
    "</normal>\n            <size>100.000000 100.000000</size>\n          "
    "</plane>\n        </geometry>\n        <material>\n          "
    "<script>\n            <uri>file://media/materials/scripts/gazebo.material"
    "</uri>\n            <name>Gazebo/Grey</name>\n          "
    "</script>\n        </material>\n      </visual>\n      "
    "<velocity_decay>\n        <linear>0.000000</linear>\n        "
    "<angular>0.000000</angular>\n      </velocity_decay>\n      "
    "<self_collide>0</self_collide>\n      <kinematic>0"
    "</kinematic>\n      <gravity>1</gravity>\n    </link>\n  "
    "</model>\n  <physics type='ode'>\n    <update_rate>1000.000000"
    "</update_rate>\n    <gravity>0.000000 0.000000 -9.800000"
    "</gravity>\n  </physics>\n  <scene>\n    <ambient>0.200000 0.200000 "
    "0.200000 1.000000</ambient>\n    <background>0.700000 0.700000 "
    "0.700000 1.000000</background>\n    <shadows>1</shadows>\n  "
    "</scene>\n  <state world_name='default'>\n    <sim_time>0 0"
    "</sim_time>\n    <real_time>0 0</real_time>\n    <wall_time>"
    "1360300141 918692496</wall_time>\n  </state>\n</world>\n</sdf>]]>"
    "</chunk>\n</gazebo_log>";

  EXPECT_EQ(validEcho, echo);
}

/////////////////////////////////////////////////
/// Check to make sure that 'gz log -e --filter' returns correct information
TEST(gz_log, EchoFilter)
{
  std::string echo, shasum;

  // Test model filter
  echo = custom_exec(
      std::string(GZ_LOG_PATH + "-e --filter pr2 -f ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  shasum = gazebo::common::get_sha1<std::string>(echo);
  // EXPECT_EQ(pr2StateLog, echo);
  if (std::string("1.4").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "0bf1f293b164bbe820267f970c4b419acdca4b01");
  else if (std::string("1.5").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "6ab471e6c410ebe90a939383bd168660ba9019e8");
  else if (std::string("1.6").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "fa985e8b71e5bef52b1c8c824d4536bb5ad8b3b4");
  else
    FAIL() << "Please add support for sdf version: " << SDF_VERSION;

  echo = custom_exec(
      std::string(GZ_LOG_PATH + "-e --filter pr2.pose -f ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  shasum = gazebo::common::get_sha1<std::string>(echo);
  // EXPECT_EQ(pr2PoseStateLog, echo);
  if (std::string("1.4").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "33db2cbd0841466a67abd7d2bbc69cf2cfae19b6");
  else if (std::string("1.5").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "739e4454112440e2c1782fff51a1382edf6cfa57");
  else if (std::string("1.6").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "37fb8793d2b9358a38f095718817a0de5df0f2d7");
  else
    FAIL() << "Please add support for sdf version: " << SDF_VERSION;

  echo = custom_exec(
      std::string(GZ_LOG_PATH + "-e --filter pr2.pose.x -f ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  shasum = gazebo::common::get_sha1<std::string>(echo);
  // EXPECT_EQ(pr2PoseXStateLog, echo);
  if (std::string("1.4").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "07113f16d44e2484f769fd1947ff5dca93f55cf4");
  else if (std::string("1.5").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "592bbdf6511ea2e76b5225a1d5c13e8a1884c429");
  else if (std::string("1.6").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "b112fffb06668852ad11149639e105bfca8d686b");
  else
    FAIL() << "Please add support for sdf version: " << SDF_VERSION;

  echo = custom_exec(
      std::string(GZ_LOG_PATH + "-e --filter pr2.pose.[x,y] -f ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  shasum = gazebo::common::get_sha1<std::string>(echo);
  // EXPECT_EQ(pr2PoseXYStateLog, echo);
  if (std::string("1.4").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "7f34f3fac505707727a74ac8659bb8736932ab07");
  else if (std::string("1.5").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "594de8f16f10ce881fcb5a17ff98ed9847891221");
  else if (std::string("1.6").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "2066e0d45ef941cce339e3219b19fe9c4a1b07c9");
  else
    FAIL() << "Please add support for sdf version: " << SDF_VERSION;

  // Test link filter
  echo = custom_exec(
      std::string(GZ_LOG_PATH + " -e --filter pr2/r_upper*.pose -f ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  shasum = gazebo::common::get_sha1<std::string>(echo);
  // EXPECT_EQ(pr2LinkStateLog, echo);
  if (std::string("1.4").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "d52ba4333511b7e4339db3eb71814c73473fba36");
  else if (std::string("1.5").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "10beb40e2870ac4cf92e67bd95c19432177a7bce");
  else if (std::string("1.6").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "396361aba2155ebd311899050211fd913b3bccce");
  else
    FAIL() << "Please add support for sdf version: " << SDF_VERSION;

  // Test joint filter
  echo = custom_exec(std::string(GZ_LOG_PATH +
        " -e --filter pr2//r_upper_arm_roll_joint -f ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  shasum = gazebo::common::get_sha1<std::string>(echo);
  // EXPECT_EQ(pr2JointStateLog, echo);
  if (std::string("1.4").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "2f689dadc66171a76f7f3400bc218485a923c324");
  else if (std::string("1.5").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "25ec9631bd32a9ac5292823e553081375a255399");
  else if (std::string("1.6").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "29b572a28f6b83e0c24e8be028c660a58626515c");
  else
    FAIL() << "Please add support for sdf version: " << SDF_VERSION;
}

/////////////////////////////////////////////////
/// Check to Hz filtering
TEST(gz_log, HzFilter)
{
  std::string echo, validEcho;

  // Test Hz filter
  echo = custom_exec(
      std::string(GZ_LOG_PATH + " -e -r -z 1.0 --filter pr2.pose.z -f ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  boost::trim_right(echo);
  validEcho = "-0.000008";
  EXPECT_EQ(validEcho, echo);

  // Test zero Hz filter
  echo = custom_exec(
      std::string(GZ_LOG_PATH + " -e -r -z 0 --filter pr2.pose.z -f ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  boost::trim_right(echo);
  validEcho = "-0.000008 \n-0.000015";
  EXPECT_EQ(validEcho, echo);

  // Test negative Hz filter
  echo = custom_exec(
      std::string(GZ_LOG_PATH + " -e -r -z -1.0 --filter pr2.pose.z -f ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  boost::trim_right(echo);
  validEcho = "-0.000008 \n-0.000015";
  EXPECT_EQ(validEcho, echo);
}

/////////////////////////////////////////////////
/// Check to raw filtering with time stamps
TEST(gz_log, RawFilterStamp)
{
  std::string echo, validEcho;

  // Sim time
  echo = custom_exec(std::string(GZ_LOG_PATH +
        " --echo -r --stamp sim --filter pr2.pose.x -f ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  boost::trim_right(echo);
  validEcho = "0.021344 0.000000 \n0.028958 0.000000";
  EXPECT_EQ(validEcho, echo);

  // Real time
  echo = custom_exec(std::string(GZ_LOG_PATH +
        " --echo -r --stamp real --filter pr2.pose.x -f ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  boost::trim_right(echo);
  validEcho = "0.001000 0.000000 \n0.002000 0.000000";
  EXPECT_EQ(validEcho, echo);

  // Wall time
  echo = custom_exec(std::string(GZ_LOG_PATH +
        " --echo -r --stamp wall --filter pr2.pose.x -f ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  boost::trim_right(echo);
  validEcho = std::string("1360301758.939690 0.000000 \n")
            + std::string("1360301758.947304 0.000000");
  EXPECT_EQ(validEcho, echo);
}

/////////////////////////////////////////////////
/// Check to make sure that 'gz log -s' returns correct information
TEST(gz_log, Step)
{
  std::string stepCmd, shasum;
  stepCmd = std::string(GZ_LOG_PATH + " -s -f ") + PROJECT_SOURCE_PATH +
    std::string("/test/data/pr2_state.log");

  // Call gz log step and press q immediately
  std::string stepq0 = custom_exec(std::string("echo 'q' | ") + stepCmd);
  shasum = gazebo::common::get_sha1<std::string>(stepq0);
  EXPECT_EQ(shasum, "81ceefcf3f81169cc9baf98f463469db242d9534");

  // Call gz log step and press space once, then q
  std::string stepq1 = custom_exec(std::string("echo ' q' | ") + stepCmd);
  shasum = gazebo::common::get_sha1<std::string>(stepq1);
  if (std::string("1.4").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "43eacb140e00ef0525d54667bc558d63dac3d21f");
  else if (std::string("1.5").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "cc55bc387eec68fc2b2aef70bfb7372a2706a6dc");
  else if (std::string("1.6").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "eefcf9d19040d39c9dbb49d3ee771fee820032d3");
  else
    FAIL() << "Please add support for sdf version: " << SDF_VERSION;

  // Call gz log step and press space twice, then q
  std::string stepq2 = custom_exec(std::string("echo '  q' | ") + stepCmd);
  shasum = gazebo::common::get_sha1<std::string>(stepq2);
  if (std::string("1.4").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "37e133d15d3f74cbc686bfceb26b8db46e2f6bf5");
  else if (std::string("1.5").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "37934b37eb4e2e7d88ed764c296232124c8a38ab");
  else if (std::string("1.6").compare(SDF_VERSION) == 0)
    EXPECT_EQ(shasum, "e4a90a629b65ff0eede877f75bcf203cfa2f7891");
  else
    FAIL() << "Please add support for sdf version: " << SDF_VERSION;
}

/////////////////////////////////////////////////
TEST(gz_log, HangCheck)
{
  gazebo::common::Time start = gazebo::common::Time::GetWallTime();
  custom_exec(GZ_LOG_PATH + " -r 0");
  gazebo::common::Time end = gazebo::common::Time::GetWallTime();

  EXPECT_LT(end - start, gazebo::common::Time(60, 0));
}

/////////////////////////////////////////////////
TEST(gz_log, Output)
{
#ifndef _MSCV
  std::string origEcho = custom_exec(std::string(GZ_LOG_PATH + " -e -f ") +
      PROJECT_SOURCE_PATH + "/test/data/empty_state.log");
  boost::trim_right(origEcho);

  std::string validEcho =
    "<?xml version='1.0'?>\n<gazebo_log>\n<header>\n<log_version>1.0"
    "</log_version>\n<gazebo_version>1.4.6</gazebo_version>\n"
    "<rand_seed>24794</rand_seed>\n<log_start>0 0</log_start>\n"
    "<log_end>0 0</log_end>\n</header>\n\n<chunk encoding='txt'>"
    "<![CDATA[\n<sdf version ='1.3'>\n<world name='default'>\n  "
    "<light name='sun' type='directional'>\n    <cast_shadows>1"
    "</cast_shadows>\n    <pose>0.000000 0.000000 10.000000 0.000000 0.000000 "
    "0.000000</pose>\n    <diffuse>0.800000 0.800000 0.800000 1.000000"
    "</diffuse>\n    <specular>0.100000 0.100000 0.100000 1.000000"
    "</specular>\n    <attenuation>\n      <range>1000.000000</range>\n      "
    "<constant>0.900000</constant>\n      <linear>0.010000</linear>\n      "
    "<quadratic>0.001000</quadratic>\n    </attenuation>\n    "
    "<direction>-0.500000 0.500000 -1.000000</direction>\n  </light>\n  "
    "<model name='ground_plane'>\n    <static>1</static>\n    "
    "<link name='link'>\n      <collision name='collision'>\n        "
    "<geometry>\n          <plane>\n            <normal>0.000000 0.000000 "
    "1.000000</normal>\n            <size>100.000000 100.000000"
    "</size>\n          </plane>\n        </geometry>\n        "
    "<surface>\n          <friction>\n            <ode>\n              "
    "<mu>100.000000</mu>\n              <mu2>50.000000</mu2>\n            "
    "</ode>\n          </friction>\n          <bounce/>\n          "
    "<contact>\n            <ode/>\n          </contact>\n        "
    "</surface>\n      </collision>\n      <visual name='visual'>\n        "
    "<cast_shadows>0</cast_shadows>\n        <geometry>\n          "
    "<plane>\n            <normal>0.000000 0.000000 1.000000"
    "</normal>\n            <size>100.000000 100.000000</size>\n          "
    "</plane>\n        </geometry>\n        <material>\n          "
    "<script>\n            <uri>file://media/materials/scripts/gazebo.material"
    "</uri>\n            <name>Gazebo/Grey</name>\n          "
    "</script>\n        </material>\n      </visual>\n      "
    "<velocity_decay>\n        <linear>0.000000</linear>\n        "
    "<angular>0.000000</angular>\n      </velocity_decay>\n      "
    "<self_collide>0</self_collide>\n      <kinematic>0"
    "</kinematic>\n      <gravity>1</gravity>\n    </link>\n  "
    "</model>\n  <physics type='ode'>\n    <update_rate>1000.000000"
    "</update_rate>\n    <gravity>0.000000 0.000000 -9.800000"
    "</gravity>\n  </physics>\n  <scene>\n    <ambient>0.200000 0.200000 "
    "0.200000 1.000000</ambient>\n    <background>0.700000 0.700000 "
    "0.700000 1.000000</background>\n    <shadows>1</shadows>\n  "
    "</scene>\n  <state world_name='default'>\n    <sim_time>0 0"
    "</sim_time>\n    <real_time>0 0</real_time>\n    <wall_time>"
    "1360300141 918692496</wall_time>\n  </state>\n</world>\n</sdf>]]>"
    "</chunk>\n</gazebo_log>";

  EXPECT_EQ(validEcho, origEcho);

  std::ostringstream newFileStream, stream;
  newFileStream << "/tmp/__gz_log_test" << std::this_thread::get_id()
    << ".log";

  stream << GZ_LOG_PATH + " -f " << PROJECT_SOURCE_PATH
    << "/test/data/empty_state.log"
    << " -o " << newFileStream.str();

  // Generate new log file using the original encoding
  custom_exec(stream.str());

  EXPECT_NO_THROW(gazebo::util::LogPlay::Instance()->Open(newFileStream.str()));
  EXPECT_EQ(gazebo::util::LogPlay::Instance()->Encoding(), "bz2");

  std::string newEcho = custom_exec(std::string(GZ_LOG_PATH + " -e -f ") +
      newFileStream.str());
  boost::trim_right(newEcho);
  EXPECT_EQ(validEcho, newEcho);

  // Output as txt
  std::ostringstream stream2;
  stream2 << GZ_LOG_PATH + " -f " << PROJECT_SOURCE_PATH
    << "/test/data/empty_state.log"
    << " -o " << newFileStream.str() << " -n txt";
  custom_exec(stream2.str());

  EXPECT_NO_THROW(gazebo::util::LogPlay::Instance()->Open(newFileStream.str()));
  EXPECT_EQ(gazebo::util::LogPlay::Instance()->Encoding(), "txt");

  newEcho = custom_exec(std::string(GZ_LOG_PATH + " -e -f ") +
      newFileStream.str());
  boost::trim_right(newEcho);
  EXPECT_EQ(validEcho, newEcho);
#endif
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
