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

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"

#include "gazebo/physics/rtql8/rtql8_inc.h"
#include "gazebo/physics/rtql8/RTQL8Utils.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
void RTQL8Utils::ConvPoseToMat(Eigen::Matrix4d* _mat, const math::Pose& _pose)
{
  assert(_mat);

  _mat->setIdentity();

  Eigen::Quaterniond quat(_pose.rot.w, _pose.rot.x,
                          _pose.rot.y, _pose.rot.z);
  _mat->topLeftCorner(3, 3) = rtql8::utils::rotation::quatToMatrix(quat);

  (*_mat)(0, 3) = _pose.pos.x;
  (*_mat)(1, 3) = _pose.pos.y;
  (*_mat)(2, 3) = _pose.pos.z;

  //(*_mat)(3, 3) = 1.0;
}

//////////////////////////////////////////////////
void RTQL8Utils::ConvMatToPose(math::Pose* _pose, const Eigen::Matrix4d& _mat)
{
  assert(_pose);

  // Set position
  _pose->pos.Set(_mat(0,3), _mat(1,3), _mat(2,3));

  // Set rotation
  Eigen::Matrix3d mat3x3 = _mat.topLeftCorner<3,3>();
  Eigen::Quaterniond quat = rtql8::utils::rotation::matrixToQuat(mat3x3);
  _pose->rot.Set(quat.w(), quat.x(), quat.y(), quat.z());
}

//////////////////////////////////////////////////
rtql8::kinematics::TrfmTranslate* RTQL8Utils::createTrfmTranslate(
    const math::Vector3& _vec)
{
  rtql8::kinematics::Dof* dofX = new rtql8::kinematics::Dof(_vec.x);
  rtql8::kinematics::Dof* dofY = new rtql8::kinematics::Dof(_vec.y);
  rtql8::kinematics::Dof* dofZ = new rtql8::kinematics::Dof(_vec.z);

  rtql8::kinematics::TrfmTranslate* ret
      = new rtql8::kinematics::TrfmTranslate(dofX, dofY, dofZ);

  return ret;
}

//////////////////////////////////////////////////
rtql8::kinematics::TrfmRotateQuat* RTQL8Utils::createTrfmRotateQuat(
    const math::Quaternion& _quat)
{
  rtql8::kinematics::Dof* dofW = new rtql8::kinematics::Dof(_quat.w);
  rtql8::kinematics::Dof* dofX = new rtql8::kinematics::Dof(_quat.x);
  rtql8::kinematics::Dof* dofY = new rtql8::kinematics::Dof(_quat.y);
  rtql8::kinematics::Dof* dofZ = new rtql8::kinematics::Dof(_quat.z);

  rtql8::kinematics::TrfmRotateQuat* ret
      = new rtql8::kinematics::TrfmRotateQuat(dofW, dofX, dofY, dofZ);

  return ret;
}

//////////////////////////////////////////////////
void RTQL8Utils::addTransformToRTQL8Joint(rtql8::kinematics::Joint* _rtl8Joint,
                                          const math::Pose& _pose)
{
  rtql8::kinematics::TrfmTranslate* trfmTrans
      = RTQL8Utils::createTrfmTranslate(_pose.pos);

  _rtl8Joint->addTransform(trfmTrans, false);

  rtql8::kinematics::TrfmRotateQuat* trfmRot
      = RTQL8Utils::createTrfmRotateQuat(_pose.rot);

  _rtl8Joint->addTransform(trfmRot, false);


}

//////////////////////////////////////////////////
void RTQL8Utils::add6DOFToRTQL8Joint(rtql8::kinematics::Joint* /*_rtql8Joint*/,
                                     const math::Pose& /*_initialPose*/)
{
}











