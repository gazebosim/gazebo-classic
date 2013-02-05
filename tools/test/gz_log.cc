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
    "End:            Jan 09 13 16:17:47.411576612\n"
    "Duration:       00:00:00.27341711\n"
    "Steps:          3\n"
    "Size:           9.161 KB\n"
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

  // Test model filter
  echo = custom_exec(
      std::string("gzlog echo --filter pr2 ") +
      PROJECT_SOURCE_PATH + "/tools/test/data/pr2_state.log");
  boost::trim_right(echo);


  validEcho =
    "  <model name='pr2'>\n"
    "    <pose>0 0 -8e-06 0 0 0</pose>\n"
    "    <joint name='torso_lift_joint'>\n"
    "      <angle axis='0'>1.41007e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='l_shoulder_lift_joint'>\n"
    "      <angle axis='0'>-1.24472e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='l_upper_arm_roll_joint'>\n"
    "      <angle axis='0'>1.04205e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='l_elbow_flex_joint'>\n"
    "      <angle axis='0'>2.63091e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='l_forearm_roll_joint'>\n"
    "      <angle axis='0'>-1.03937e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='l_wrist_flex_joint'>\n"
    "      <angle axis='0'>-5.93599e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='l_wrist_roll_joint'>\n"
    "      <angle axis='0'>-2.00905e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='l_gripper_motor_screw_joint'>\n"
    "      <angle axis='0'>-2.80641e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='l_gripper_r_finger_tip_joint'>\n"
    "      <angle axis='0'>1.53825e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='laser_tilt_mount_joint'>\n"
    "      <angle axis='0'>-2.47273e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='r_shoulder_lift_joint'>\n"
    "      <angle axis='0'>-1.28046e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='r_upper_arm_roll_joint'>\n"
    "      <angle axis='0'>-1.0234e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='r_elbow_flex_joint'>\n"
    "      <angle axis='0'>3.52763e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='r_forearm_roll_joint'>\n"
    "      <angle axis='0'>1.0253e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='r_wrist_flex_joint'>\n"
    "      <angle axis='0'>2.10212e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='r_wrist_roll_joint'>\n"
    "      <angle axis='0'>-6.13793e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='r_gripper_motor_screw_joint'>\n"
    "      <angle axis='0'>-3.52607e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='r_gripper_r_finger_joint'>\n"
    "      <angle axis='0'>-2.04123e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='r_gripper_r_finger_tip_joint'>\n"
    "      <angle axis='0'>-1.00512e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='torso_lift_motor_screw_joint'>\n"
    "      <angle axis='0'>7.9351e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='torso_lift_screw_torso_lift_joint'>\n"
    "      <angle axis='0'>-1.98549e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='l_gripper_r_parallel_root_joint'>\n"
    "      <angle axis='0'>-1.12785e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='l_gripper_r_parallel_tip_joint'>\n"
    "      <angle axis='0'>1.85524e-06</angle>\n"
    "    </joint>\n"
    "  </model>\n"
    "\n"
    "  <model name='pr2'>\n"
    "    <pose>0 0 -1.5e-05 0 -1e-06 0</pose>\n"
    "    <joint name='bl_caster_l_wheel_joint'>\n"
    "      <angle axis='0'>1.01079e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='br_caster_l_wheel_joint'>\n"
    "      <angle axis='0'>1.00809e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='br_caster_r_wheel_joint'>\n"
    "      <angle axis='0'>1.00718e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='fr_caster_l_wheel_joint'>\n"
    "      <angle axis='0'>1.01174e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='fr_caster_r_wheel_joint'>\n"
    "      <angle axis='0'>1.0102e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='torso_lift_joint'>\n"
    "      <angle axis='0'>1.61697e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='head_tilt_joint'>\n"
    "      <angle axis='0'>1.59176e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='l_shoulder_lift_joint'>\n"
    "      <angle axis='0'>-2.5384e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='l_upper_arm_roll_joint'>\n"
    "      <angle axis='0'>2.25789e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='l_elbow_flex_joint'>\n"
    "      <angle axis='0'>2.4368e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='l_forearm_roll_joint'>\n"
    "      <angle axis='0'>-2.26431e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='l_wrist_flex_joint'>\n"
    "      <angle axis='0'>4.61642e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='l_wrist_roll_joint'>\n"
    "      <angle axis='0'>4.04443e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='l_gripper_motor_screw_joint'>\n"
    "      <angle axis='0'>-1.66172e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='l_gripper_r_finger_joint'>\n"
    "      <angle axis='0'>1.55625e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='laser_tilt_mount_joint'>\n"
    "      <angle axis='0'>-4.67204e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='r_shoulder_lift_joint'>\n"
    "      <angle axis='0'>-2.57981e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='r_upper_arm_roll_joint'>\n"
    "      <angle axis='0'>-2.22655e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='r_elbow_flex_joint'>\n"
    "      <angle axis='0'>4.39678e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='r_forearm_roll_joint'>\n"
    "      <angle axis='0'>2.22922e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='r_wrist_flex_joint'>\n"
    "      <angle axis='0'>-1.43521e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='r_wrist_roll_joint'>\n"
    "      <angle axis='0'>-2.56647e-05</angle>\n"
    "    </joint>\n"
    "    <joint name='r_gripper_l_finger_joint'>\n"
    "      <angle axis='0'>2.47396e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='r_gripper_l_finger_tip_joint'>\n"
    "      <angle axis='0'>1.96603e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='r_gripper_r_finger_joint'>\n"
    "      <angle axis='0'>5.37142e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='r_gripper_r_finger_tip_joint'>\n"
    "      <angle axis='0'>3.84165e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='torso_lift_motor_screw_joint'>\n"
    "      <angle axis='0'>0.000183324</angle>\n"
    "    </joint>\n"
    "    <joint name='torso_lift_screw_torso_lift_joint'>\n"
    "      <angle axis='0'>-2.25856e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='r_gripper_r_parallel_tip_joint'>\n"
    "      <angle axis='0'>1.13052e-06</angle>\n"
    "    </joint>\n"
    "    <joint name='r_gripper_l_parallel_tip_joint'>\n"
    "      <angle axis='0'>1.09768e-06</angle>\n"
    "    </joint>\n"
    "  </model>";

  EXPECT_EQ(validEcho, echo);

  // Test joint filter
  echo = custom_exec(
      std::string("gzlog echo --filter pr2::r_upper_arm_roll_joint ") +
      PROJECT_SOURCE_PATH + "/tools/test/data/pr2_state.log");
  boost::trim_right(echo);

  validEcho =
    "    <joint name='r_upper_arm_roll_joint'>\n"
    "      <angle axis='0'>-1.0234e-05</angle>\n"
    "    </joint>\n\n"
    "    <joint name='r_upper_arm_roll_joint'>\n"
    "      <angle axis='0'>-2.22655e-05</angle>\n"
    "    </joint>";

  EXPECT_EQ(validEcho, echo);

  // Test joint filter with angle
  echo = custom_exec(
      std::string("gzlog echo --filter pr2::r_upper_arm_roll_joint::0 ") +
      PROJECT_SOURCE_PATH + "/tools/test/data/pr2_state.log");
  boost::trim_right(echo);

  validEcho =
    "-1.0234e-05\n"
    "-2.22655e-05";

  EXPECT_EQ(validEcho, echo);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
