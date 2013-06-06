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

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTUtils.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
void DARTUtils::ConvPoseToMat(Eigen::Matrix4d* _mat, const math::Pose& _pose)
{
  assert(_mat);

  _mat->setIdentity();

  Eigen::Quaterniond quat(_pose.rot.w, _pose.rot.x,
                          _pose.rot.y, _pose.rot.z);
  _mat->topLeftCorner<3,3>() = dart::math::quatToMatrix(quat);

  (*_mat)(0, 3) = _pose.pos.x;
  (*_mat)(1, 3) = _pose.pos.y;
  (*_mat)(2, 3) = _pose.pos.z;

  //(*_mat)(3, 3) = 1.0;
}

//////////////////////////////////////////////////
void DARTUtils::ConvMatToPose(math::Pose* _pose, const Eigen::Matrix4d& _mat)
{
  assert(_pose);

  // Set position
  _pose->pos.Set(_mat(0,3), _mat(1,3), _mat(2,3));

  // Set rotation
  Eigen::Matrix3d mat3x3 = _mat.topLeftCorner<3,3>();
  Eigen::Quaterniond quat = dart::math::matrixToQuat(mat3x3);
  _pose->rot.Set(quat.w(), quat.x(), quat.y(), quat.z());
}

//////////////////////////////////////////////////
//void DARTUtils::Add6DOFToDARTJoint(kinematics::Joint* /*_dartJoint*/,
//                                     const math::Pose& /*_initialPose*/)
//{
//}











