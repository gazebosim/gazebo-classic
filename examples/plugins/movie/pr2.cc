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

/////////////////////////////////////////////////
MoviePR2::MoviePR2()
{
  this->initJointAngles["fl_caster_rotation_joint"] = -1.34133e-05;
  this->initJointAngles["fl_caster_l_wheel_joint"] = 0.0363128;
  this->initJointAngles["fl_caster_r_wheel_joint"] = 0.0354769;
  this->initJointAngles["fr_caster_rotation_joint"] = 2.20451e-05;
  this->initJointAngles["fr_caster_l_wheel_joint"] = 0.0387796;
  this->initJointAngles["fr_caster_r_wheel_joint"] = 0.0411882;
  this->initJointAngles["bl_caster_rotation_joint"] = 2.15834e-05;
  this->initJointAngles["bl_caster_l_wheel_joint"] = 0.0163751;
  this->initJointAngles["bl_caster_r_wheel_joint"] = 0.0189168;
  this->initJointAngles["br_caster_rotation_joint"] = -3.23288e-05;
  this->initJointAngles["br_caster_l_wheel_joint"] = 0.0198638;
  this->initJointAngles["br_caster_r_wheel_joint"] = 0.0193917;
  this->initJointAngles["torso_lift_joint"] = 0.0114989;
  this->initJointAngles["torso_lift_motor_screw_joint"] = 0;
  this->initJointAngles["head_pan_joint"] = 0.0886893;
  this->initJointAngles["head_tilt_joint"] = 0.289404;
  this->initJointAngles["laser_tilt_mount_joint"] = 0.0447445;
  this->initJointAngles["r_upper_arm_roll_joint"] = 5.9785e-05;
  this->initJointAngles["r_shoulder_pan_joint"] = -0.400182;
  this->initJointAngles["r_shoulder_lift_joint"] = 1.0029;
  this->initJointAngles["r_forearm_roll_joint"] = 1.28884e-06;
  this->initJointAngles["r_elbow_flex_joint"] = -2.05015;
  this->initJointAngles["r_wrist_flex_joint"] = -0.0996016;
  this->initJointAngles["r_wrist_roll_joint"] = 4.67052e-06;
  this->initJointAngles["r_gripper_l_finger_joint"] = 0.00326152;
  this->initJointAngles["r_gripper_r_finger_joint"] = 0.00326152;
  this->initJointAngles["r_gripper_r_finger_tip_joint"] = 0.00326152;
  this->initJointAngles["r_gripper_l_finger_tip_joint"] = 0.00326152;
  this->initJointAngles["l_upper_arm_roll_joint"] = 9.86255e-05;
  this->initJointAngles["l_shoulder_pan_joint"] = 0.400183;
  this->initJointAngles["l_shoulder_lift_joint"] = 1.00088;
  this->initJointAngles["l_forearm_roll_joint"] = 2.78374e-06;
  this->initJointAngles["l_elbow_flex_joint"] = -2.05019;
  this->initJointAngles["l_wrist_flex_joint"] = -0.0999533;
  this->initJointAngles["l_wrist_roll_joint"] = 4.54168e-06;
  this->initJointAngles["l_gripper_l_finger_joint"] = 0.0024712;
  this->initJointAngles["l_gripper_r_finger_joint"] = 0.0024712;
  this->initJointAngles["l_gripper_r_finger_tip_joint"] = 0.0024712;
  this->initJointAngles["l_gripper_l_finger_tip_joint"] = 0.0024712;

//  this->leftArmStowPos.Set(0.2, 0.4, -0.3);
//  this->rightArmStowPos.Set(0.2, -0.4, -0.3);
}

/////////////////////////////////////////////////
void MoviePR2::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->node = transport::NodePtr(new transport::Node());
}

/////////////////////////////////////////////////
void MoviePR2::Init()
{
  this->node->Init();
  this->jointAnimSub = this->node->Subscribe("~/joint_animation",
      &MoviePR2::OnJointAnimation, this);

  this->world = physics::get_world("default");
  this->pr2 = this->world->GetModel("pr2");

  if (!this->pr2)
    gzerr << "Unable to find model\n";

  this->pr2->SetJointPositions(this->initJointAngles);

  // this->connections.push_back( event::Events::ConnectWorldUpdateStart(
  //      boost::bind(&MoviePR2::OnUpdate, this) ) );

}

/////////////////////////////////////////////////
/*void MoviePR2::OnUpdate()
{
}*/

/////////////////////////////////////////////////
void MoviePR2::TuckArms()
{
  std::map<std::string, double> tuckedAngles;
  std::map<std::string, double>::iterator iter;
  std::map<std::string, common::NumericAnimationPtr> tuckArmsAnim;
  common::NumericKeyFrame *key;

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

  double duration = 1.0;
  for (iter = tuckedAngles.begin(); iter != tuckedAngles.end(); ++iter)
  {
    tuckArmsAnim[iter->first].reset(
        new gazebo::common::NumericAnimation(iter->first, duration, false));
    key = tuckArmsAnim[iter->first]->CreateKeyFrame(0);
    key->SetValue(
        this->pr2->GetJoint(iter->first)->GetAngle(0).GetAsRadian());
    key = tuckArmsAnim[iter->first]->CreateKeyFrame(duration);
    key->SetValue(iter->second);
  }

  this->pr2->SetJointAnimation(tuckArmsAnim);
}

/////////////////////////////////////////////////
void MoviePR2::OnJointAnimation(ConstJointAnimationPtr &_anim)
{
  if (_anim->model_name() == "pr2")
  {
    std::map<std::string, common::NumericAnimationPtr> anim;
    for (unsigned int i=0; i < _anim->joint(0).name_size(); ++i)
    {
      anim[_anim->joint(0).name(i)].reset(
          new common::NumericAnimation(_anim->joint(0).name(i) + "_anim",
            20, false));
    }
      
    double stepSize = 20.0 / _anim->joint_size();
    for (unsigned int i=0; i < _anim->joint_size(); ++i)
    {
      double time = i*stepSize;
      for (unsigned int j=0; j < _anim->joint(i).name_size(); ++j)
      {
        common::NumericKeyFrame *key =
          anim[_anim->joint(i).name(j)]->CreateKeyFrame(time);
        key->SetValue(_anim->joint(i).angle(j));
      }
    }

    this->pr2->SetJointAnimation(anim);
  }
}
