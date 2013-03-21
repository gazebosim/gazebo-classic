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

static std::string pr2LinkStateLog =
"<state name='default'>\n"
"<sim_time>0 21343973</sim_time>\n"
"<real_time>0 1000000</real_time>\n"
"<wall_time>1360301758 939690376</wall_time>\n"
"<model name='pr2'>\n"
"<link name='r_upper_arm_roll_link'>\n"
"<pose>0.000000 0.000000 0.000001 -0.000010 -0.000013 0.000000</pose>\n"
"</link>\n"
"</model>\n"
"</state>\n"
"<state name='default'>\n"
"<sim_time>0 28958235</sim_time>\n"
"<real_time>0 2000000</real_time>\n"
"<wall_time>1360301758 947304437</wall_time>\n"
"<model name='pr2'>\n"
"<link name='r_upper_arm_roll_link'>\n"
"<pose>0.000000 0.000000 0.000002 -0.000022 -0.000026 0.000000</pose>\n"
"</link>\n"
"</model>\n"
"</state>";

static std::string pr2PoseStateLog =
"<state name='default'>\n"
"<sim_time>0 21343973</sim_time>\n"
"<real_time>0 1000000</real_time>\n"
"<wall_time>1360301758 939690376</wall_time>\n"
"<model name='pr2'>\n"
"<pose>0.000000 0.000000 -0.000008 0.000000 0.000000 0.000000</pose>\n"
"</model>\n"
"</state>\n"
"<state name='default'>\n"
"<sim_time>0 28958235</sim_time>\n"
"<real_time>0 2000000</real_time>\n"
"<wall_time>1360301758 947304437</wall_time>\n"
"<model name='pr2'>\n"
"<pose>0.000000 0.000000 -0.000015 0.000000 -0.000001 0.000000</pose>\n"
"</model>\n"
"</state>";

static std::string pr2PoseXStateLog =
"<state name='default'>\n"
"<sim_time>0 21343973</sim_time>\n"
"<real_time>0 1000000</real_time>\n"
"<wall_time>1360301758 939690376</wall_time>\n"
"<model name='pr2'>\n"
"0.000000 \n"
"</model>\n"
"</state>\n"
"<state name='default'>\n"
"<sim_time>0 28958235</sim_time>\n"
"<real_time>0 2000000</real_time>\n"
"<wall_time>1360301758 947304437</wall_time>\n"
"<model name='pr2'>\n"
"0.000000 \n"
"</model>\n"
"</state>";

static std::string pr2PoseXYStateLog =
"<state name='default'>\n"
"<sim_time>0 21343973</sim_time>\n"
"<real_time>0 1000000</real_time>\n"
"<wall_time>1360301758 939690376</wall_time>\n"
"<model name='pr2'>\n"
"0.000000 0.000000 \n"
"</model>\n"
"</state>\n"
"<state name='default'>\n"
"<sim_time>0 28958235</sim_time>\n"
"<real_time>0 2000000</real_time>\n"
"<wall_time>1360301758 947304437</wall_time>\n"
"<model name='pr2'>\n"
"0.000000 0.000000 \n"
"</model>\n"
"</state>";

static std::string pr2JointStateLog =
"<state name='default'>\n"
"<sim_time>0 21343973</sim_time>\n"
"<real_time>0 1000000</real_time>\n"
"<wall_time>1360301758 939690376</wall_time>\n"
"<model name='pr2'>\n"
"    <joint name='r_upper_arm_roll_joint'>\n"
"      <angle axis='0'>-0.000010</angle>\n"
"    </joint>\n"
"</model>\n"
"</state>\n"
"<state name='default'>\n"
"<sim_time>0 28958235</sim_time>\n"
"<real_time>0 2000000</real_time>\n"
"<wall_time>1360301758 947304437</wall_time>\n"
"<model name='pr2'>\n"
"    <joint name='r_upper_arm_roll_joint'>\n"
"      <angle axis='0'>-0.000022</angle>\n"
"    </joint>\n"
"</model>\n"
"</state>";


static std::string pr2StateLog =
"<state name='default'>\n"
"<sim_time>0 21343973</sim_time>\n"
"<real_time>0 1000000</real_time>\n"
"<wall_time>1360301758 939690376</wall_time>\n"
"  <model name='pr2'>\n"
"    <pose>0.000000 0.000000 -0.000008 0.000000 0.000000 0.000000</pose>\n"
"    <link name='base_footprint'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000124 0.000045 -0.007966 0.000178 -0.000477 -0.000068\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='bl_caster_rotation_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000166 0.000067 -0.008024 0.000173 -0.000487 -0.000007\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='bl_caster_l_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000199 0.000079 -0.008031 0.000174 0.000008 -0.000007\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='bl_caster_r_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000198 0.000081 -0.008046 0.000182 -0.000009 -0.000007\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='br_caster_rotation_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000135 0.000067 -0.008105 0.000160 -0.000508 -0.000007\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='br_caster_l_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000171 0.000078 -0.008099 0.000177 0.000008 -0.000004\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='br_caster_r_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000169 0.000076 -0.008117 0.000168 0.000008 -0.000007\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='fl_caster_rotation_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000164 0.000034 -0.007804 0.000177 -0.000499 -0.000007\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='fl_caster_l_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000200 0.000047 -0.007798 0.000171 0.000008 -0.000007\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='fl_caster_r_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000199 0.000047 -0.007812 0.000185 0.000009 -0.000008\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='fr_caster_rotation_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000135 0.000033 -0.007880 0.000163 -0.000504 -0.000004\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='fr_caster_l_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000172 0.000047 -0.007880 0.000147 0.000007 -0.000008\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='fr_caster_r_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000170 0.000048 -0.007893 0.000165 0.000008 -0.000006\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='torso_lift_link'>\n"
"      <pose>0.000000 0.000000 0.000001 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>-0.000125 -0.000039 -0.006598 0.000257 -0.000557 -0.000050\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='head_pan_link'>\n"
"      <pose>0.000000 0.000000 0.000001 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>-0.000408 -0.000142 -0.006521 0.000269 -0.000537 -0.000027\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='head_tilt_link'>\n"
"      <pose>0.000000 0.000000 0.000001 0.000000 0.000001 0.000000</pose>\n"
"      <velocity>-0.000355 -0.000141 -0.006499 0.000235 0.000285 -0.000006\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_shoulder_pan_link'>\n"
"      <pose>0.000000 0.000000 0.000001 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>-0.000105 -0.000043 -0.006514 0.000204 -0.000547 0.000021\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_shoulder_lift_link'>\n"
"      <pose>0.000000 0.000000 0.000001 0.000000 -0.000013 0.000000</pose>\n"
"      <velocity>0.000232 -0.000055 -0.006142 0.000164 -0.012994 -0.000011\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_upper_arm_roll_link'>\n"
"      <pose>0.000000 0.000000 0.000001 0.000010 -0.000013 0.000000</pose>\n"
"      <velocity>-0.000166 -0.000047 -0.003496 0.010584 -0.013020 0.000021\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_elbow_flex_link'>\n"
"      <pose>0.000000 0.000000 0.000006 0.000010 -0.000010 0.000000</pose>\n"
"      <velocity>-0.000041 0.000073 -0.001117 0.010487 -0.010393 0.000104\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_forearm_roll_link'>\n"
"      <pose>0.000000 0.000000 0.000006 0.000000 -0.000010 0.000000</pose>\n"
"      <velocity>-0.000056 -0.000035 0.000636 0.000103 -0.010853 0.000103\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_wrist_flex_link'>\n"
"      <pose>0.000000 0.000000 0.000010 0.000000 -0.000016 0.000000</pose>\n"
"      <velocity>-0.000188 -0.000021 0.002071 0.000102 -0.016789 0.000095\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_wrist_roll_link'>\n"
"      <pose>0.000000 0.000000 0.000010 0.000000 0.000004 0.000000</pose>\n"
"      <velocity>-0.000207 -0.000012 0.001890 -0.000006 0.003301 0.000134\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_gripper_l_finger_link'>\n"
"      <pose>0.000000 0.000000 0.000009 0.000000 0.000004 0.000000</pose>\n"
"      <velocity>-0.000211 -0.000004 0.001672 0.000112 0.003367 0.000391\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_gripper_l_finger_tip_link'>\n"
"      <pose>0.000000 0.000000 0.000009 -0.000001 0.000003 0.000001</pose>\n"
"      <velocity>-0.000205 0.000004 0.001475 -0.000525 0.002883 0.000777\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_gripper_motor_slider_link'>\n"
"      <pose>0.000000 0.000000 0.000009 0.000000 0.000004 0.000000</pose>\n"
"      <velocity>-0.000217 0.000002 0.001502 -0.000027 0.003252 0.000126\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_gripper_motor_screw_link'>\n"
"      <pose>0.000000 0.000000 0.000009 0.000000 0.000001 0.000000</pose>\n"
"      <velocity>-0.000198 0.000003 0.001530 -0.000019 0.000446 0.000102\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_gripper_r_finger_link'>\n"
"      <pose>0.000000 0.000000 0.000009 0.000000 0.000005 0.000000</pose>\n"
"      <velocity>-0.000195 -0.000005 0.001689 0.000528 0.004075 0.000231\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_gripper_r_finger_tip_link'>\n"
"      <pose>0.000000 0.000000 0.000009 -0.000000 0.000003 0.000001</pose>\n"
"      <velocity>-0.000196 -0.000007 0.001461 -0.000277 0.002914 0.000838\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='laser_tilt_mount_link'>\n"
"      <pose>0.000000 0.000000 0.000001 0.000000 -0.000003 0.000000</pose>\n"
"      <velocity>-0.000251 -0.000097 -0.006470 0.000167 -0.003028 -0.000026\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_shoulder_pan_link'>\n"
"      <pose>0.000000 0.000000 0.000001 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>-0.000131 -0.000042 -0.006546 0.000175 -0.000491 0.000113\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_shoulder_lift_link'>\n"
"      <pose>0.000000 0.000000 0.000001 0.000000 -0.000013 0.000000</pose>\n"
"      <velocity>0.000247 -0.000040 -0.006179 0.000157 -0.013296 0.000132\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_upper_arm_roll_link'>\n"
"      <pose>0.000000 0.000000 0.000001 -0.000010 -0.000013 0.000000</pose>\n"
"      <velocity>-0.000172 -0.000028 -0.003538 -0.010076 -0.013338 0.000097\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_elbow_flex_link'>\n"
"      <pose>0.000000 0.000000 0.000007 -0.000010 -0.000009 -0.000000</pose>\n"
"      <velocity>-0.000010 -0.000128 -0.001074 -0.010150 -0.009811 0.000068\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_forearm_roll_link'>\n"
"      <pose>0.000000 0.000000 0.000006 0.000000 -0.000010 0.000000</pose>\n"
"      <velocity>-0.000077 0.000010 0.000654 0.000037 -0.010973 0.000103\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_wrist_flex_link'>\n"
"      <pose>0.000000 0.000000 0.000010 0.000000 0.000011 0.000000</pose>\n"
"      <velocity>-0.000161 0.000021 0.002104 0.000036 0.010048 -0.000014\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_wrist_roll_link'>\n"
"      <pose>0.000000 0.000000 0.000010 0.000000 0.000004 0.000000</pose>\n"
"      <velocity>-0.000155 0.000024 0.001899 -0.000007 0.003910 0.000019\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_l_finger_link'>\n"
"      <pose>0.000000 0.000000 0.000009 -0.000001 0.000005 0.000000</pose>\n"
"      <velocity>-0.000157 0.000023 0.001601 -0.000388 0.004576 -0.000012\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_l_finger_tip_link'>\n"
"      <pose>0.000000 0.000000 0.000009 0.000000 0.000005 0.000000</pose>\n"
"      <velocity>-0.000144 0.000032 0.001325 0.000280 0.004739 0.000044\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_motor_slider_link'>\n"
"      <pose>0.000000 0.000000 0.000009 0.000000 0.000005 0.000000</pose>\n"
"      <velocity>-0.000146 0.000021 0.001405 -0.000027 0.004035 0.000020\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_motor_screw_link'>\n"
"      <pose>0.000000 0.000000 0.000009 0.000000 0.000001 0.000000</pose>\n"
"      <velocity>-0.000141 0.000022 0.001340 -0.000003 0.000509 0.000055\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_r_finger_link'>\n"
"      <pose>0.000000 0.000000 0.000009 0.000001 0.000006 0.000000</pose>\n"
"      <velocity>-0.000144 0.000024 0.001637 0.001538 0.005243 0.000076\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_r_finger_tip_link'>\n"
"      <pose>0.000000 0.000000 0.000009 0.000000 0.000006 0.000000</pose>\n"
"      <velocity>-0.000142 0.000025 0.001314 0.000541 0.005139 -0.000003\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='torso_lift_motor_screw_link'>\n"
"      <pose>0.000000 0.000000 -0.000001 -0.000001 0.000000 0.000079</pose>\n"
"      <velocity>-0.000153 -0.000064 -0.008583 -0.000379 -0.000480 0.079281\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_l_parallel_link'>\n"
"      <pose>0.000000 0.000000 0.000009 -0.000001 0.000005 0.000000</pose>\n"
"      <velocity>-0.000164 0.000028 0.001671 -0.000599 0.004494 -0.000162\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_r_parallel_link'>\n"
"      <pose>0.000000 0.000000 0.000009 0.000000 0.000005 0.000000</pose>\n"
"      <velocity>-0.000143 0.000020 0.001670 0.000480 0.004431 -0.000033\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_gripper_l_parallel_link'>\n"
"      <pose>0.000000 0.000000 0.000009 0.000000 0.000004 0.000001</pose>\n"
"      <velocity>-0.000211 -0.000007 0.001734 -0.000121 0.003723 0.000500\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_gripper_r_parallel_link'>\n"
"      <pose>0.000000 0.000000 0.000009 0.000001 0.000005 0.000000</pose>\n"
"      <velocity>-0.000186 -0.000003 0.001741 0.000700 0.004127 -0.000169\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <joint name='torso_lift_joint'>\n"
"      <angle axis='0'>0.000001</angle>\n"
"    </joint>\n"
"    <joint name='l_shoulder_lift_joint'>\n"
"      <angle axis='0'>-0.000012</angle>\n"
"    </joint>\n"
"    <joint name='l_upper_arm_roll_joint'>\n"
"      <angle axis='0'>0.000010</angle>\n"
"    </joint>\n"
"    <joint name='l_elbow_flex_joint'>\n"
"      <angle axis='0'>0.000003</angle>\n"
"    </joint>\n"
"    <joint name='l_forearm_roll_joint'>\n"
"      <angle axis='0'>-0.000010</angle>\n"
"    </joint>\n"
"    <joint name='l_wrist_flex_joint'>\n"
"      <angle axis='0'>-0.000006</angle>\n"
"    </joint>\n"
"    <joint name='l_wrist_roll_joint'>\n"
"      <angle axis='0'>-0.000020</angle>\n"
"    </joint>\n"
"    <joint name='l_gripper_motor_screw_joint'>\n"
"      <angle axis='0'>-0.000003</angle>\n"
"    </joint>\n"
"    <joint name='l_gripper_r_finger_tip_joint'>\n"
"      <angle axis='0'>0.000002</angle>\n"
"    </joint>\n"
"    <joint name='laser_tilt_mount_joint'>\n"
"      <angle axis='0'>-0.000002</angle>\n"
"    </joint>\n"
"    <joint name='r_shoulder_lift_joint'>\n"
"      <angle axis='0'>-0.000013</angle>\n"
"    </joint>\n"
"    <joint name='r_upper_arm_roll_joint'>\n"
"      <angle axis='0'>-0.000010</angle>\n"
"    </joint>\n"
"    <joint name='r_elbow_flex_joint'>\n"
"      <angle axis='0'>0.000004</angle>\n"
"    </joint>\n"
"    <joint name='r_forearm_roll_joint'>\n"
"      <angle axis='0'>0.000010</angle>\n"
"    </joint>\n"
"    <joint name='r_wrist_flex_joint'>\n"
"      <angle axis='0'>0.000021</angle>\n"
"    </joint>\n"
"    <joint name='r_wrist_roll_joint'>\n"
"      <angle axis='0'>-0.000006</angle>\n"
"    </joint>\n"
"    <joint name='r_gripper_motor_screw_joint'>\n"
"      <angle axis='0'>-0.000004</angle>\n"
"    </joint>\n"
"    <joint name='r_gripper_r_finger_joint'>\n"
"      <angle axis='0'>-0.000002</angle>\n"
"    </joint>\n"
"    <joint name='r_gripper_r_finger_tip_joint'>\n"
"      <angle axis='0'>-0.000001</angle>\n"
"    </joint>\n"
"    <joint name='torso_lift_motor_screw_joint'>\n"
"      <angle axis='0'>0.000079</angle>\n"
"    </joint>\n"
"    <joint name='torso_lift_screw_torso_lift_joint'>\n"
"      <angle axis='0'>-0.000002</angle>\n"
"    </joint>\n"
"    <joint name='l_gripper_r_parallel_root_joint'>\n"
"      <angle axis='0'>-0.000001</angle>\n"
"    </joint>\n"
"    <joint name='l_gripper_r_parallel_tip_joint'>\n"
"      <angle axis='0'>0.000002</angle>\n"
"    </joint>\n"
"  </model>\n"
"</state>\n"
"<state name='default'>\n"
"<sim_time>0 28958235</sim_time>\n"
"<real_time>0 2000000</real_time>\n"
"<wall_time>1360301758 947304437</wall_time>\n"
"  <model name='pr2'>\n"
"    <pose>0.000000 0.000000 -0.000015 0.000000 -0.000001 0.000000</pose>\n"
"    <link name='base_footprint'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000116 0.000046 -0.007403 0.000204 -0.000505 -0.000055\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='bl_caster_rotation_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000153 0.000069 -0.007439 0.000138 -0.000505 -0.000006\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='bl_caster_l_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000001 0.000000</pose>\n"
"      <velocity>0.000197 0.000083 -0.007405 0.000194 0.000009 -0.000002\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='bl_caster_r_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000001 0.000000</pose>\n"
"      <velocity>0.000195 0.000081 -0.007462 0.000167 -0.000008 -0.000010\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='br_caster_rotation_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000126 0.000067 -0.007533 0.000261 -0.000482 -0.000010\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='br_caster_l_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000001 0.000000</pose>\n"
"      <velocity>0.000165 0.000088 -0.007513 0.000227 0.000011 -0.000013\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='br_caster_r_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000001 0.000000</pose>\n"
"      <velocity>0.000166 0.000086 -0.007535 0.000192 0.000009 -0.000008\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='fl_caster_rotation_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000156 0.000045 -0.007216 0.000239 -0.000482 -0.000009\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='fl_caster_l_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000001 0.000000</pose>\n"
"      <velocity>0.000196 0.000060 -0.007220 0.000163 0.000008 -0.000007\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='fl_caster_r_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000001 0.000000</pose>\n"
"      <velocity>0.000195 0.000055 -0.007239 0.000141 0.000007 -0.000006\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='fr_caster_rotation_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>0.000127 0.000047 -0.007314 0.000149 -0.000489 -0.000009\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='fr_caster_l_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000001 0.000000</pose>\n"
"      <velocity>0.000165 0.000057 -0.007290 0.000235 0.000011 -0.000006\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='fr_caster_r_wheel_link'>\n"
"      <pose>0.000000 0.000000 0.000000 0.000000 0.000001 0.000000</pose>\n"
"      <velocity>0.000165 0.000057 -0.007331 0.000186 0.000009 -0.000010\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='torso_lift_link'>\n"
"      <pose>0.000000 0.000000 0.000002 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>-0.000126 -0.000044 -0.007240 0.000118 -0.000415 -0.000075\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='head_pan_link'>\n"
"      <pose>0.000000 0.000000 0.000002 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>-0.000412 -0.000157 -0.007230 0.000037 -0.000510 0.000048\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='head_tilt_link'>\n"
"      <pose>0.000000 0.000000 0.000002 0.000000 0.000002 0.000000</pose>\n"
"      <velocity>-0.000316 -0.000148 -0.007178 0.000054 0.000261 -0.000017\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_shoulder_pan_link'>\n"
"      <pose>0.000000 0.000000 0.000002 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>-0.000112 -0.000049 -0.007149 0.000173 -0.000543 0.000015\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_shoulder_lift_link'>\n"
"      <pose>0.000000 0.000000 0.000002 0.000000 -0.000025 0.000000</pose>\n"
"      <velocity>0.000195 -0.000057 -0.006877 0.000177 -0.013480 0.000080\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_upper_arm_roll_link'>\n"
"      <pose>0.000000 0.000000 0.000002 0.000023 -0.000026 0.000000</pose>\n"
"      <velocity>-0.000182 -0.000095 -0.004053 0.012333 -0.013693 0.000180\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_elbow_flex_link'>\n"
"      <pose>0.000000 0.000000 0.000012 0.000023 -0.000023 0.000000</pose>\n"
"      <velocity>0.000015 0.000090 -0.001495 0.012455 -0.013886 0.000230\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_forearm_roll_link'>\n"
"      <pose>0.000000 0.000000 0.000012 0.000000 -0.000024 0.000000</pose>\n"
"      <velocity>-0.000041 -0.000036 0.000995 0.000195 -0.013748 0.000049\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_wrist_flex_link'>\n"
"      <pose>0.000000 0.000000 0.000020 0.000000 0.000023 0.000000</pose>\n"
"      <velocity>-0.000086 -0.000033 0.003007 0.000194 0.038352 0.000013\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_wrist_roll_link'>\n"
"      <pose>0.000000 0.000000 0.000020 0.000000 0.000018 0.000000</pose>\n"
"      <velocity>-0.000126 -0.000028 0.002169 0.000002 0.014212 0.000098\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_gripper_l_finger_link'>\n"
"      <pose>0.000000 0.000000 0.000018 -0.000001 0.000019 0.000000</pose>\n"
"      <velocity>-0.000137 -0.000022 0.001392 -0.000618 0.014293 -0.000139\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_gripper_l_finger_tip_link'>\n"
"      <pose>0.000000 0.000000 0.000016 0.000000 0.000018 0.000000</pose>\n"
"      <velocity>-0.000109 -0.000013 0.000532 0.000646 0.014604 -0.000656\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_gripper_motor_slider_link'>\n"
"      <pose>0.000000 0.000000 0.000016 0.000000 0.000018 0.000000</pose>\n"
"      <velocity>-0.000092 -0.000006 0.000625 0.000045 0.014263 0.000110\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_gripper_r_finger_link'>\n"
"      <pose>0.000000 0.000000 0.000018 -0.000001 0.000018 -0.000000</pose>\n"
"      <velocity>-0.000132 -0.000024 0.001376 -0.000792 0.013210 -0.000136\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_gripper_r_finger_tip_link'>\n"
"      <pose>0.000000 0.000000 0.000016 0.000000 0.000019 0.000000</pose>\n"
"      <velocity>-0.000111 -0.000006 0.000560 0.000694 0.015000 -0.000728\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='laser_tilt_mount_link'>\n"
"      <pose>0.000000 0.000000 0.000002 0.000000 -0.000005 0.000000</pose>\n"
"      <velocity>-0.000297 -0.000116 -0.007252 0.000219 -0.002617 -0.000079\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_shoulder_pan_link'>\n"
"      <pose>0.000000 0.000000 0.000002 0.000000 0.000000 0.000000</pose>\n"
"      <velocity>-0.000142 -0.000045 -0.007275 0.000228 -0.000590 0.000097\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_shoulder_lift_link'>\n"
"      <pose>0.000000 0.000000 0.000002 0.000000 -0.000026 0.000000</pose>\n"
"      <velocity>0.000168 -0.000055 -0.006943 0.000010 -0.013582 0.000028\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_upper_arm_roll_link'>\n"
"      <pose>0.000000 0.000000 0.000002 -0.000022 -0.000026 0.000000</pose>\n"
"      <velocity>-0.000213 0.000012 -0.004147 -0.012020 -0.013480 -0.000002\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_elbow_flex_link'>\n"
"      <pose>0.000000 0.000000 0.000012 -0.000023 -0.000021 0.000000</pose>\n"
"      <velocity>-0.000089 -0.000137 -0.001615 -0.012091 -0.012612 0.000015\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_forearm_roll_link'>\n"
"      <pose>0.000000 0.000000 0.000012 0.000000 -0.000023 0.000000</pose>\n"
"      <velocity>-0.000069 0.000015 0.000538 -0.000033 -0.012990 0.000078\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_wrist_flex_link'>\n"
"      <pose>0.000000 0.000000 0.000019 0.000000 -0.000037 0.000000</pose>\n"
"      <velocity>-0.000213 0.000029 0.002283 -0.000034 -0.048363 0.000245\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_wrist_roll_link'>\n"
"      <pose>0.000000 0.000000 0.000019 0.000000 0.000001 0.000000</pose>\n"
"      <velocity>-0.000241 0.000033 0.002408 -0.000033 -0.004285 0.000125\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_l_finger_link'>\n"
"      <pose>0.000000 0.000000 0.000019 0.000000 0.000002 0.000001</pose>\n"
"      <velocity>-0.000240 0.000042 0.002724 0.000605 -0.003930 0.000412\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_l_finger_tip_link'>\n"
"      <pose>0.000000 0.000000 0.000019 0.000000 0.000003 0.000000</pose>\n"
"      <velocity>-0.000248 0.000029 0.002980 -0.000071 -0.003186 0.000067\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_motor_slider_link'>\n"
"      <pose>0.000000 0.000000 0.000019 0.000000 0.000001 0.000000</pose>\n"
"      <velocity>-0.000244 0.000055 0.002912 -0.000005 -0.003571 0.000126\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_motor_screw_link'>\n"
"      <pose>0.000000 0.000000 0.000019 0.000000 0.000002 0.000000</pose>\n"
"      <velocity>-0.000238 0.000046 0.002944 -0.000043 0.000804 0.000132\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_r_finger_link'>\n"
"      <pose>0.000000 0.000000 0.000019 0.000000 0.000002 0.000000</pose>\n"
"      <velocity>-0.000247 0.000037 0.002672 -0.000894 -0.004483 -0.000188\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_r_finger_tip_link'>\n"
"      <pose>0.000000 0.000000 0.000019 -0.000002 0.000002 -0.000000</pose>\n"
"      <velocity>-0.000249 0.000030 0.002969 -0.001725 -0.004043 0.000137\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='torso_lift_motor_screw_link'>\n"
"      <pose>0.000000 0.000000 -0.000001 0.000000 0.000000 0.000183</pose>\n"
"      <velocity>-0.000151 -0.000013 -0.007513 0.000745 -0.000663 0.103921\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_l_parallel_link'>\n"
"      <pose>0.000000 0.000000 0.000019 0.000000 0.000001 0.000000</pose>\n"
"      <velocity>-0.000238 0.000031 0.002651 0.000555 -0.004326 0.000028\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='r_gripper_r_parallel_link'>\n"
"      <pose>0.000000 0.000000 0.000019 -0.000001 0.000001 0.000000</pose>\n"
"      <velocity>-0.000242 0.000040 0.002661 -0.001241 -0.004413 -0.000078\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_gripper_l_parallel_link'>\n"
"      <pose>0.000000 0.000000 0.000018 -0.000001 0.000018 -0.000000</pose>\n"
"      <velocity>-0.000146 -0.000021 0.001652 -0.000197 0.013567 -0.000258\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <link name='l_gripper_r_parallel_link'>\n"
"      <pose>0.000000 0.000000 0.000018 -0.000001 0.000019 0.000000</pose>\n"
"      <velocity>-0.000135 -0.000029 0.001649 -0.000872 0.013488 0.000434\
</velocity>\n"
"      <acceleration>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000\
</acceleration>\n"
"      <wrench>0.000000 0.000000 0.000000 0.000000 0.000000 0.000000</wrench>\n"
"    </link>\n"
"    <joint name='bl_caster_l_wheel_joint'>\n"
"      <angle axis='0'>0.000001</angle>\n"
"    </joint>\n"
"    <joint name='br_caster_l_wheel_joint'>\n"
"      <angle axis='0'>0.000001</angle>\n"
"    </joint>\n"
"    <joint name='br_caster_r_wheel_joint'>\n"
"      <angle axis='0'>0.000001</angle>\n"
"    </joint>\n"
"    <joint name='fr_caster_l_wheel_joint'>\n"
"      <angle axis='0'>0.000001</angle>\n"
"    </joint>\n"
"    <joint name='fr_caster_r_wheel_joint'>\n"
"      <angle axis='0'>0.000001</angle>\n"
"    </joint>\n"
"    <joint name='torso_lift_joint'>\n"
"      <angle axis='0'>0.000002</angle>\n"
"    </joint>\n"
"    <joint name='head_tilt_joint'>\n"
"      <angle axis='0'>0.000002</angle>\n"
"    </joint>\n"
"    <joint name='l_shoulder_lift_joint'>\n"
"      <angle axis='0'>-0.000025</angle>\n"
"    </joint>\n"
"    <joint name='l_upper_arm_roll_joint'>\n"
"      <angle axis='0'>0.000023</angle>\n"
"    </joint>\n"
"    <joint name='l_elbow_flex_joint'>\n"
"      <angle axis='0'>0.000002</angle>\n"
"    </joint>\n"
"    <joint name='l_forearm_roll_joint'>\n"
"      <angle axis='0'>-0.000023</angle>\n"
"    </joint>\n"
"    <joint name='l_wrist_flex_joint'>\n"
"      <angle axis='0'>0.000046</angle>\n"
"    </joint>\n"
"    <joint name='l_wrist_roll_joint'>\n"
"      <angle axis='0'>0.000004</angle>\n"
"    </joint>\n"
"    <joint name='l_gripper_motor_screw_joint'>\n"
"      <angle axis='0'>-0.000017</angle>\n"
"    </joint>\n"
"    <joint name='l_gripper_r_finger_joint'>\n"
"      <angle axis='0'>0.000002</angle>\n"
"    </joint>\n"
"    <joint name='laser_tilt_mount_joint'>\n"
"      <angle axis='0'>-0.000005</angle>\n"
"    </joint>\n"
"    <joint name='r_shoulder_lift_joint'>\n"
"      <angle axis='0'>-0.000026</angle>\n"
"    </joint>\n"
"    <joint name='r_upper_arm_roll_joint'>\n"
"      <angle axis='0'>-0.000022</angle>\n"
"    </joint>\n"
"    <joint name='r_elbow_flex_joint'>\n"
"      <angle axis='0'>0.000004</angle>\n"
"    </joint>\n"
"    <joint name='r_forearm_roll_joint'>\n"
"      <angle axis='0'>0.000022</angle>\n"
"    </joint>\n"
"    <joint name='r_wrist_flex_joint'>\n"
"      <angle axis='0'>-0.000014</angle>\n"
"    </joint>\n"
"    <joint name='r_wrist_roll_joint'>\n"
"      <angle axis='0'>-0.000026</angle>\n"
"    </joint>\n"
"    <joint name='r_gripper_l_finger_joint'>\n"
"      <angle axis='0'>0.000002</angle>\n"
"    </joint>\n"
"    <joint name='r_gripper_l_finger_tip_joint'>\n"
"      <angle axis='0'>0.000002</angle>\n"
"    </joint>\n"
"    <joint name='r_gripper_r_finger_joint'>\n"
"      <angle axis='0'>0.000005</angle>\n"
"    </joint>\n"
"    <joint name='r_gripper_r_finger_tip_joint'>\n"
"      <angle axis='0'>0.000004</angle>\n"
"    </joint>\n"
"    <joint name='torso_lift_motor_screw_joint'>\n"
"      <angle axis='0'>0.000183</angle>\n"
"    </joint>\n"
"    <joint name='torso_lift_screw_torso_lift_joint'>\n"
"      <angle axis='0'>-0.000002</angle>\n"
"    </joint>\n"
"    <joint name='r_gripper_r_parallel_tip_joint'>\n"
"      <angle axis='0'>0.000001</angle>\n"
"    </joint>\n"
"    <joint name='r_gripper_l_parallel_tip_joint'>\n"
"      <angle axis='0'>0.000001</angle>\n"
"    </joint>\n"
"  </model>\n"
"</state>";

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
    "Start:          Feb 08 13 05:35:55.667456998\n"
    "End:            Feb 08 13 05:35:58.947304437\n"
    "Duration:       00:00:03.279847439\n"
    "Steps:          3\n"
    "Size:           12.377 KB\n"
    "Encoding:       bz2\n"
    "Model Count:    2";

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
    "    <ambient>0.200000 0.200000 0.200000 1.000000</ambient>\n"
    "    <background>0.700000 0.700000 0.700000 1.000000</background>\n"
    "    <shadows>1</shadows>\n"
    "  </scene>\n"
    "  <state world_name='default'>\n"
    "    <sim_time>0 0</sim_time>\n"
    "    <real_time>0 0</real_time>\n"
    "    <wall_time>1360300141 918692496</wall_time>\n"
    "  </state>\n"
    "</world>\n"
    "</sdf>";

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
      PROJECT_SOURCE_PATH + "/test/data/pr2_state.log");
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
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
