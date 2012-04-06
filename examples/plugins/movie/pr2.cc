/* Copyright 2011 Nate Koenig & Andrew Howard
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
#include <boost/bind.hpp>

#include "common/common.h"
#include "transport/transport.h"
#include "physics/physics.h"
#include "math/Pose.hh"

#include "pr2.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MoviePR2)

MoviePR2::MoviePR2()
{
//  this->leftArmStowPos.Set(0.2, 0.4, -0.3);
//  this->rightArmStowPos.Set(0.2, -0.4, -0.3);
}

void MoviePR2::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
}

void MoviePR2::Init()
{
  this->world = physics::get_world("default");
  this->pr2 = this->world->GetModel("pr2");
if (!this->pr2)
gzerr << "Unable to find model\n";
//  this->connections.push_back( event::Events::ConnectWorldUpdateStart(
//        boost::bind(&LfDPR2::OnUpdate, this) ) );

  this->TuckArms();
}

void MoviePR2::TuckArms()
{
   std::map<std::string, double> tuckedAngles;

  //tuckedAngles["r_gripper_joint"] = -0.1;
  //tuckedAngles["r_gripper_r_parallel_root_joint"] = -0.1;
  //tuckedAngles["r_gripper_r_parallel_root_joint"] = -1.5;
  //tuckedAngles["r_gripper_l_parallel_root_joint"] = -1.5;
  //tuckedAngles["r_gripper_r_parallel_root_joint"] = -1.0;

  tuckedAngles["r_shoulder_pan_joint"] = -0.023593;
  tuckedAngles["r_shoulder_lift_joint"] = 1.1072800;
  tuckedAngles["r_upper_arm_roll_joint"] = -1.5566882;
  tuckedAngles["r_elbow_flex_joint"] = -2.124408;
  tuckedAngles["r_forearm_roll_joint"] = -1.4175;
  tuckedAngles["r_wrist_flex_joint"] = -1.8417;
  tuckedAngles["r_wrist_roll_joint"] = 0.21436;

  tuckedAngles["l_shoulder_pan_joint"] = 0.06024;
  tuckedAngles["l_shoulder_lift_joint"] = 1.248526;
  tuckedAngles["l_upper_arm_roll_joint"] = 1.789070;
  tuckedAngles["l_elbow_flex_joint"] = -1.683386;
  tuckedAngles["l_forearm_roll_joint"] = -1.7343417;
  tuckedAngles["l_wrist_flex_joint"] = -0.0962141;
  tuckedAngles["l_wrist_roll_joint"] = -0.0864407;

  tuckedAngles["torso_lift_joint"] = 0.25;

  this->pr2->SetJointPositions(tuckedAngles);
}
