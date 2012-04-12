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

  /*this->boxPoses.push_back(math::Pose(0, 0, 0,0,0,0));
  this->boxPoses.push_back(math::Pose(-5.54842e-05, 0.000409954, 0.000433487,0,0,0));
  this->boxPoses.push_back(math::Pose(-0.00398473, 0.00175377, 0.000292246,0,0,0));
  this->boxPoses.push_back(math::Pose(-0.00317138, 0.00193565, 0.000353519,0,0,0));
  this->boxPoses.push_back(math::Pose(-0.00330441, 0.00187775, 0.000329412,0,0,0));
  this->boxPoses.push_back(math::Pose(-0.00333291, 0.00189231, 0.000316815,0,0,0));
  this->boxPoses.push_back(math::Pose(0.00463545, -0.00562677, 0.0916906,0,0,0));
  this->boxPoses.push_back(math::Pose(-0.00540163, -0.0018754, 0.11129,0,0,0));
  this->boxPoses.push_back(math::Pose(-0.0049024, -0.125314, 0.123428,0,0,0));
  this->boxPoses.push_back(math::Pose(-0.00522803, -0.119145, 0.115009,0,0,0));
  this->boxPoses.push_back(math::Pose(0.00415493, -0.118132, 0.116519,0,0,0));
  this->boxPoses.push_back(math::Pose(0.0117716, -0.138053, 0.114465,0,0,0));
  this->boxPoses.push_back(math::Pose(0.00980751, -0.281352, 0.115558,0,0,0));
  this->boxPoses.push_back(math::Pose(0.0167366, -0.298412, 0.115664,0,0,0));
  this->boxPoses.push_back(math::Pose(0.0445353, -0.300066, 0.110787,0,0,0));
  this->boxPoses.push_back(math::Pose(0.0445099, -0.300383, 0.114708,0,0,0));
  this->boxPoses.push_back(math::Pose(0.0457681, -0.300534, 0.115256,0,0,0));
  this->boxPoses.push_back(math::Pose(0.0458142, -0.300444, 0.115282,0,0,0));
  this->boxPoses.push_back(math::Pose(0.0461633, -0.300372, 0.115277,0,0,0));
  this->boxPoses.push_back(math::Pose(0.0458949, -0.300276, 0.115265,0,0,0));
  this->boxPoses.push_back(math::Pose(0.0461615, -0.300416, 0.115269,0,0,0));
  this->boxPoses.push_back(math::Pose(0.0724202, -0.304378, 0.118313,0,0,0));
  this->boxPoses.push_back(math::Pose(0.218398, -0.318287, 0.114632,0,0,0));
  this->boxPoses.push_back(math::Pose(0.36726, -0.332934, 0.115137,0,0,0));
  this->boxPoses.push_back(math::Pose(0.387839, -0.335767, 0.115663,0,0,0));
  this->boxPoses.push_back(math::Pose(0.531101, -0.349494, 0.114105,0,0,0));
  this->boxPoses.push_back(math::Pose(0.680722, -0.364899, 0.115203,0,0,0));
  this->boxPoses.push_back(math::Pose(0.819197, -0.377909, 0.113961,0,0,0));
  this->boxPoses.push_back(math::Pose(0.817879, -0.378466, 0.116853,0,0,0));
  this->boxPoses.push_back(math::Pose(0.839478, -0.381989, 0.118039,0,0,0));
  this->boxPoses.push_back(math::Pose(0.993265, -0.396247, 0.113574,0,0,0));
  this->boxPoses.push_back(math::Pose(1.0239, -0.399373, 0.113063,0,0,0));
  this->boxPoses.push_back(math::Pose(1.02591, -0.399431, 0.115129,0,0,0));
  this->boxPoses.push_back(math::Pose(1.15874, -0.412394, 0.115198,0,0,0));
  this->boxPoses.push_back(math::Pose(1.29609, -0.425869, 0.113953,0,0,0));
  this->boxPoses.push_back(math::Pose(1.29512, -0.426193, 0.116498,0,0,0));
  this->boxPoses.push_back(math::Pose(1.32701, -0.42931, 0.117609,0,0,0));
  this->boxPoses.push_back(math::Pose(1.59983, -0.437687, 0.0918756,0,0,0));
  this->boxPoses.push_back(math::Pose(1.72329, -0.441926, 0.114448,0,0,0));
  this->boxPoses.push_back(math::Pose(1.7563, -0.448728, 0.105698,0,0,0));
  this->boxPoses.push_back(math::Pose(1.81109, -0.461533, 0.100498,0,0,0));
  this->boxPoses.push_back(math::Pose(1.86837, -0.544885, 0.0798548,0,0,0));
  this->boxPoses.push_back(math::Pose(1.79613, -0.570809, 0.0944766,0,0,0));
  this->boxPoses.push_back(math::Pose(1.80839, -0.562181, 0.0946563,0,0,0));
  this->boxPoses.push_back(math::Pose(1.81163, -0.562316, 0.0899764,0,0,0));
  this->boxPoses.push_back(math::Pose(1.81656, -0.540296, -0.134194,0,0,0));
  this->boxPoses.push_back(math::Pose(1.81129, -0.575702, -0.36018,0,0,0));
  this->boxPoses.push_back(math::Pose(1.81252, -0.573592, -0.366416,0,0,0));
  this->boxPoses.push_back(math::Pose(1.81169, -0.574002, -0.365978,0,0,0));
  this->boxPoses.push_back(math::Pose(1.81167, -0.573782, -0.365916,0,0,0));
  this->boxPoses.push_back(math::Pose(1.81174, -0.573758, -0.365856,0,0,0));
  this->boxPoses.push_back(math::Pose(1.83931, -0.600129, -0.366621,0,0,0));
  this->boxPoses.push_back(math::Pose(1.96149, -0.719199, -0.372747,0,0,0));
  this->boxPoses.push_back(math::Pose(2.14822, -0.913825, -0.386947,0,0,0));
  this->boxPoses.push_back(math::Pose(2.18146, -0.925336, -0.406168,0,0,0));
  this->boxPoses.push_back(math::Pose(2.18778, -0.93129, -0.507521,0,0,0));
  this->boxPoses.push_back(math::Pose(2.16072, -0.928525, -0.616825,0,0,0));
  this->boxPoses.push_back(math::Pose(2.13983, -0.948493, -0.665556,0,0,0));
  this->boxPoses.push_back(math::Pose(2.10334, -0.96154, -0.704115,0,0,0));
  this->boxPoses.push_back(math::Pose(2.06028, -0.975018, -0.74349,0,0,0));
  this->boxPoses.push_back(math::Pose(2.04467, -0.981096, -0.754095,0,0,0));
  this->boxPoses.push_back(math::Pose(2.04503, -0.982305, -0.754366,0,0,0));
  this->boxPoses.push_back(math::Pose(2.0474, -0.982613, -0.755858,0,0,0));
  this->boxPoses.push_back(math::Pose(2.0456, -0.980992, -0.756073,0,0,0));
  this->boxPoses.push_back(math::Pose(2.04394, -0.979394, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.04252, -0.97798, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.03393, -0.965661, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.01627, -0.940832, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.01675, -0.939771, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.01774, -0.939118, -0.756208,0,0,0));
  this->boxPoses.push_back(math::Pose(2.02008, -0.939832, -0.756208,0,0,0));
  this->boxPoses.push_back(math::Pose(2.02264, -0.94079, -0.756208,0,0,0));
  this->boxPoses.push_back(math::Pose(2.03023, -0.939056, -0.7562,0,0,0));
  this->boxPoses.push_back(math::Pose(2.03507, -0.947282, -0.7562,0,0,0));
  this->boxPoses.push_back(math::Pose(2.03992, -0.955508, -0.7562,0,0,0));
  this->boxPoses.push_back(math::Pose(2.04477, -0.963735, -0.7562,0,0,0));
  this->boxPoses.push_back(math::Pose(2.04961, -0.971961, -0.7562,0,0,0));
  this->boxPoses.push_back(math::Pose(2.05446, -0.980186, -0.7562,0,0,0));
  this->boxPoses.push_back(math::Pose(2.05931, -0.98833, -0.756199,0,0,0));
  this->boxPoses.push_back(math::Pose(2.06415, -0.996311, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.069, -1.00424, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.07385, -1.01216, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.0787, -1.02007, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.07834, -1.02289, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.07739, -1.02512, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.07643, -1.02735, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.0755, -1.02939, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.07466, -1.03025, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.07379, -1.03116, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.07305, -1.03192, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.07241, -1.03246, -0.756205,0,0,0));
  this->boxPoses.push_back(math::Pose(2.07181, -1.03284, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.07111, -1.03322, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.07034, -1.03363, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.06963, -1.03398, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.06891, -1.03432, -0.756199,0,0,0));
  this->boxPoses.push_back(math::Pose(2.06827, -1.03458, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.06763, -1.0348, -0.756199,0,0,0));
  this->boxPoses.push_back(math::Pose(2.06702, -1.03495, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.06647, -1.03505, -0.756198,0,0,0));
  this->boxPoses.push_back(math::Pose(2.06577, -1.03517, -0.7562,0,0,0));
  */


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

  this->poseAnimSub = this->node->Subscribe("~/pose_animation",
      &MoviePR2::OnPoseAnimation, this);

  this->world = physics::get_world("default");
  this->pr2 = this->world->GetModel("pr2");
  this->box = this->world->GetModel("box1");

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
void MoviePR2::OnPoseAnimation(ConstPoseAnimationPtr &_anim)
{
  if (_anim->model_name() == "pr2")
    this->pr2->SetJointAnimation(this->myAnim);

  /*
  if (_anim->model_name() == "box1")
  {
    math::Pose startPose = this->box->GetWorldPose();
    common::PoseAnimationPtr anim(new common::PoseAnimation("mov2", 20, false));

    double stepSize = 20.0 / this->boxPoses.size();
    for (unsigned int i=0; i < this->boxPoses.size(); i++)
    {
      double time = i*stepSize;
      common::PoseKeyFrame *key = anim->CreateKeyFrame(time);
      math::Pose p = this->boxPoses[i];

      key->SetTranslation(startPose.pos + p.pos);
      key->SetRotation(math::Quaternion(1,0,0,0));
    }

   // math::Pose pose;

    //std::cout << "OnPoseAnimation!\n";
    //common::PoseAnimationPtr anim(new common::PoseAnimation("mov2", 20, false));

    //double stepSize = 20.0 / _anim->pose_size();
    //for (unsigned int i=0; i < _anim->pose_size(); ++i)
    //{
    //  double time = i*stepSize;
    //  common::PoseKeyFrame *key = anim->CreateKeyFrame(time);
    //      //msgs::Convert(_anim->time(i)).Double());

    //  pose.Set(msgs::Convert(_anim->pose(i).position()),
    //           msgs::Convert(_anim->pose(i).orientation()));
    //  std::cout << "Pose[" << startPose.pos + pose.pos << "] Time[" << time << "]\n";
    //  // pose = startPose + pose;

    //  key->SetTranslation(startPose.pos + pose.pos);
    //  key->SetRotation(math::Quaternion(1,0,0,0));
    //}

    this->box->SetAnimation(anim);
  }
*/
}

/////////////////////////////////////////////////
void MoviePR2::OnJointAnimation(ConstJointAnimationPtr &_anim)
{
  if (_anim->model_name() == "pr2")
  {
    for (unsigned int i=0; i < _anim->joint(0).name_size(); ++i)
    {
      this->myAnim[_anim->joint(0).name(i)].reset(
          new common::NumericAnimation(_anim->joint(0).name(i) + "_anim",
            30, false));
    }
      
    double stepSize = 20.0 / _anim->joint_size();
    for (unsigned int i=0; i < _anim->joint_size(); ++i)
    {
      double time = i*stepSize;
      for (unsigned int j=0; j < _anim->joint(i).name_size(); ++j)
      {
        common::NumericKeyFrame *key =
          this->myAnim[_anim->joint(i).name(j)]->CreateKeyFrame(time);
        key->SetValue(_anim->joint(i).angle(j));
      }
    }

    int i = _anim->joint_size()-1;
    double time = 30;
      for (unsigned int j=0; j < _anim->joint(i).name_size(); ++j)
      {
        common::NumericKeyFrame *key =
          this->myAnim[_anim->joint(i).name(j)]->CreateKeyFrame(time);
        key->SetValue(_anim->joint(i).angle(j));
      }

  }
}
