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
Eigen::Matrix4d RTQL8Utils::ConvPose(const math::Pose& _pose)
{
  Eigen::Matrix4d ret;
  Eigen::Quaterniond quat(_pose.rot.w, _pose.rot.x,
                          _pose.rot.y, _pose.rot.z);
  ret.topLeftCorner(3, 3) = rtql8::utils::rotation::quatToMatrix(quat);
  ret(0, 3) = _pose.pos.x;
  ret(1, 3) = _pose.pos.y;
  ret(2, 3) = _pose.pos.z;
  ret(3, 3) = 1.0;

  return ret;
}

//////////////////////////////////////////////////
bool RTQL8Utils::ConvPose(Eigen::Matrix4d* _mat, const math::Pose& _pose)
{
  Eigen::Quaterniond quat(_pose.rot.w, _pose.rot.x,
                          _pose.rot.y, _pose.rot.z);
  _mat->topLeftCorner(3, 3) = rtql8::utils::rotation::quatToMatrix(quat);
  (*_mat)(0, 3) = _pose.pos.x;
  (*_mat)(1, 3) = _pose.pos.y;
  (*_mat)(2, 3) = _pose.pos.z;
  (*_mat)(3, 3) = 1.0;

  return true;
}
