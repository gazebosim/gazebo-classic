/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "KeyFrame.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
KeyFrame::KeyFrame(double _time)
: time(_time)
{
}

/////////////////////////////////////////////////
KeyFrame::~KeyFrame()
{
}

/////////////////////////////////////////////////
double KeyFrame::GetTime() const
{
  return this->time;
}


/////////////////////////////////////////////////
PoseKeyFrame::PoseKeyFrame(double _time)
: KeyFrame(_time)
{
}

/////////////////////////////////////////////////
PoseKeyFrame::~PoseKeyFrame()
{
}

/////////////////////////////////////////////////
void PoseKeyFrame::SetTranslation(const math::Vector3 &_trans)
{
  this->translate.Set(_trans.x, _trans.y, _trans.z);
}

/////////////////////////////////////////////////
void PoseKeyFrame::Translation(const ignition::math::Vector3d &_trans)
{
  this->translate = _trans;
}

/////////////////////////////////////////////////
math::Vector3 PoseKeyFrame::GetTranslation() const
{
  return gazebo::math::Vector3(this->translate.X(), this->translate.Y(),
                               this->translate.Z());
}

/////////////////////////////////////////////////
ignition::math::Vector3d PoseKeyFrame::Translation() const
{
  return this->translate;
}

/////////////////////////////////////////////////
void PoseKeyFrame::SetRotation(const math::Quaternion &_rot)
{
  this->rotate.Set(_rot.w, _rot.x, _rot.y, _rot.z);
}

/////////////////////////////////////////////////
void PoseKeyFrame::Rotation(const ignition::math::Quaterniond &_rot)
{
  this->rotate = _rot;
}

/////////////////////////////////////////////////
math::Quaternion PoseKeyFrame::GetRotation() const
{
  return gazebo::math::Quaternion(this->rotate.W(),
      this->rotate.X(), this->rotate.Y(), this->rotate.Z());
}

/////////////////////////////////////////////////
ignition::math::Quaterniond PoseKeyFrame::Rotation() const
{
  return this->rotate;
}

/////////////////////////////////////////////////
NumericKeyFrame::NumericKeyFrame(double _time)
: KeyFrame(_time)
{
}

/////////////////////////////////////////////////
NumericKeyFrame::~NumericKeyFrame()
{
}

/////////////////////////////////////////////////
void NumericKeyFrame::SetValue(const double &_value)
{
  this->value = _value;
}

/////////////////////////////////////////////////
const double &NumericKeyFrame::GetValue() const
{
  return this->value;
}
