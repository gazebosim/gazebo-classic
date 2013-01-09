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
#include <stdio.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>

#include "test_config.h"

std::string custom_exec(const std::string &_cmd)
{
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
      PROJECT_SOURCE_PATH + "/tools/test/data/pr2_state.log");
  boost::trim_right(info);

  std::string validInfo =
    "Log Version:    1.0\n"
    "Gazebo Version: 1.4.0\n"
    "Random Seed:    22123\n"
    "Start:          Jan 09 13 16:17:47.384234901\n"
    "End:            Jan 09 13 16:17:51.403684732\n"
    "Duration:       00:00:04.19449831\n"
    "Steps:          1090\n"
    "Size:           1.04359 MB\n"
    "Encoding:       bz2\n"
    "Model Count:    2";

  EXPECT_EQ(validInfo, info);
}

/////////////////////////////////////////////////
/// Check to make sure that 'gzlog echo' returns correct information
TEST(gz_log, Echo)
{
  std::string echo = custom_exec(std::string("gzlog echo ") +
      PROJECT_SOURCE_PATH + "/tools/test/data/empty_state.log");
  boost::trim_right(echo);

  std::string validEcho =
"<sdf version ='1.3'>\n"
"<world name='default'>\n"
"  <light name='sun' type='directional'>\n"
"    <cast_shadows>1</cast_shadows>\n"
"    <pose>0.000000 0.000000 10.000000 0.000000 0.000000 0.000000</pose>\n"
"    <diffuse>0.800000 0.800000 0.800000 1.000000</diffuse>\n"
"    <specular>0.100000 0.100000 0.100000 1.000000</specular>\n"
"    <attenuation>\n"
"      <range>1000.000000</range>\n"
"      <constant>0.900000</constant>\n"
"      <linear>0.010000</linear>\n"
"      <quadratic>0.001000</quadratic>\n"
"    </attenuation>\n"
"    <direction>-0.500000 0.500000 -1.000000</direction>\n"
"  </light>\n"
"  <model name='ground_plane'>\n"
"    <static>1</static>\n"
"    <link name='link'>\n"
"      <collision name='collision'>\n"
"        <geometry>\n"
"          <plane>\n"
"            <normal>0.000000 0.000000 1.000000</normal>\n"
"            <size>100.000000 100.000000</size>\n"
"          </plane>\n"
"        </geometry>\n"
"        <surface>\n"
"          <friction>\n"
"            <ode>\n"
"              <mu>100.000000</mu>\n"
"              <mu2>50.000000</mu2>\n"
"            </ode>\n"
"          </friction>\n"
"          <bounce/>\n"
"          <contact>\n"
"            <ode/>\n"
"          </contact>\n"
"        </surface>\n"
"      </collision>\n"
"      <visual name='visual'>\n"
"        <cast_shadows>0</cast_shadows>\n"
"        <geometry>\n"
"          <plane>\n"
"            <normal>0.000000 0.000000 1.000000</normal>\n"
"            <size>100.000000 100.000000</size>\n"
"          </plane>\n"
"        </geometry>\n"
"        <material>\n"
"          <script>\n"
"            <uri>file://media/materials/scripts/gazebo.material</uri>\n"
"            <name>Gazebo/Grey</name>\n"
"          </script>\n"
"        </material>\n"
"      </visual>\n"
"      <velocity_decay>\n"
"        <linear>0.000000</linear>\n"
"        <angular>0.000000</angular>\n"
"      </velocity_decay>\n"
"      <self_collide>0</self_collide>\n"
"      <kinematic>0</kinematic>\n"
"      <gravity>1</gravity>\n"
"    </link>\n"
"  </model>\n"
"  <physics type='ode'>\n"
"    <update_rate>1000.000000</update_rate>\n"
"    <gravity>0.000000 0.000000 -9.800000</gravity>\n"
"  </physics>\n"
"  <scene>\n"
"    <ambient>0.000000 0.000000 0.000000 1.000000</ambient>\n"
"    <background>0.700000 0.700000 0.700000 1.000000</background>\n"
"    <shadows>1</shadows>\n"
"  </scene>\n"
"  <state world_name='__default__'>\n"
"    <sim_time>0 0</sim_time>\n"
"    <real_time>0 0</real_time>\n"
"    <wall_time>1357757564 822623113</wall_time>\n"
"  </state>\n"
"</world>\n"
"</sdf>";

  EXPECT_EQ(validEcho, echo);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
