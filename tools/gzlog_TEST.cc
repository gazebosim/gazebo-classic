/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include <gazebo/common/Time.hh>

#include <stdio.h>
#include <string>

#include "test/data/pr2_state_log_expected.h"
#include "test_config.h"
#include "gazebo/gazebo_config.h"

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
/// Check to make sure that 'gzlog info' returns correct information
TEST(gz_log, Info)
{
  std::string info = custom_exec(std::string("gzlog info ") +
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
/// Check to make sure that 'gzlog echo' returns correct information
TEST(gz_log, Echo)
{
  std::string echo = custom_exec(std::string("gzlog echo ") +
      PROJECT_SOURCE_PATH + "/test/data/empty_state.log");
  boost::trim_right(echo);

  std::string validEcho =
    "<?xml version='1.0'?>\n<gazebo_log>\n<header>\n<log_version>1.0"
    "</log_version>\n<gazebo_version>1.4.6</gazebo_version>\n"
    "<rand_seed>24794</rand_seed>\n</header>\n\n<chunk encoding='txt'>"
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
/// Check to make sure that 'gzlog echo --filter' returns correct information
TEST(gz_log, EchoFilter)
{
  std::string echo;

  // Test model filter
  echo = custom_exec(
      std::string("gzlog echo --filter pr2 ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  boost::trim_right(echo);
  EXPECT_EQ(pr2StateLog, echo);

  echo = custom_exec(
      std::string("gzlog echo --filter pr2.pose ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  boost::trim_right(echo);
  EXPECT_EQ(pr2PoseStateLog, echo);

  echo = custom_exec(
      std::string("gzlog echo --filter pr2.pose.x ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log 2>/dev/null");
  boost::trim_right(echo);
  EXPECT_EQ(pr2PoseXStateLog, echo);

  echo = custom_exec(
      std::string("gzlog echo --filter pr2.pose.[x,y] ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  boost::trim_right(echo);
  EXPECT_EQ(pr2PoseXYStateLog, echo);

  // Test link filter
  echo = custom_exec(
      std::string("gzlog echo --filter pr2/r_upper*.pose ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  boost::trim_right(echo);
  EXPECT_EQ(pr2LinkStateLog, echo);

  // Test joint filter
  echo = custom_exec(
      std::string("gzlog echo --filter pr2//r_upper_arm_roll_joint ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  boost::trim_right(echo);
  EXPECT_EQ(pr2JointStateLog, echo);
}

/////////////////////////////////////////////////
/// Check to Hz filtering
TEST(gz_log, HzFilter)
{
  std::string echo, validEcho;

  // Test Hz filter
  echo = custom_exec(
      std::string("gzlog echo -r -z 1.0 --filter pr2.pose.z ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  boost::trim_right(echo);
  validEcho = "-0.000008";
  EXPECT_EQ(validEcho, echo);

  // Test zero Hz filter
  echo = custom_exec(
      std::string("gzlog echo -r -z 0 --filter pr2.pose.z ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  boost::trim_right(echo);
  validEcho = "-0.000008 \n-0.000015";
  EXPECT_EQ(validEcho, echo);

  // Test negative Hz filter
  echo = custom_exec(
      std::string("gzlog echo -r -z -1.0 --filter pr2.pose.z ") +
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
  boost::trim_right(echo);
  validEcho = "-0.000008 \n-0.000015";
  EXPECT_EQ(validEcho, echo);
}

/////////////////////////////////////////////////
/// Check to make sure that 'gzlog step' returns correct information
/// Just check number of characters returned for now
TEST(gz_log, Step)
{
  std::string stepCmd;
  stepCmd = std::string("gzlog step ") + PROJECT_SOURCE_PATH +
    std::string("/test/data/pr2_state.log");

  // Call gzlog step and press q immediately
  std::string stepq0 = custom_exec(std::string("echo 'q' | ") + stepCmd);
  EXPECT_EQ(stepq0.length(), 115569u);

  // Call gzlog step and press space once, then q
  std::string stepq1 = custom_exec(std::string("echo ' q' | ") + stepCmd);
#ifdef HAVE_SDF
  EXPECT_EQ(stepq1.length(), 124131u);
#else
  EXPECT_EQ(stepq1.length(), 124082u);
#endif

  // Call gzlog step and press space twice, then q
  std::string stepq2 = custom_exec(std::string("echo '  q' | ") + stepCmd);
#ifdef HAVE_SDF
  EXPECT_EQ(stepq2.length(), 132516u);
#else
  EXPECT_EQ(stepq2.length(), 132427u);
#endif
}

/////////////////////////////////////////////////
TEST(gz_log, HangCheck)
{
  gazebo::common::Time start = gazebo::common::Time::GetWallTime();
  custom_exec("gzlog stop");
  gazebo::common::Time end = gazebo::common::Time::GetWallTime();

  EXPECT_LT(end - start, gazebo::common::Time(60, 0));
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
