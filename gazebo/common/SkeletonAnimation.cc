/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/common/SkeletonAnimation.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Assert.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
NodeAnimation::NodeAnimation(const std::string& _name)
{
  this->name = _name;
  this->length = 0.0;
}

//////////////////////////////////////////////////
NodeAnimation::~NodeAnimation()
{
  this->keyFrames.clear();
}

//////////////////////////////////////////////////
void NodeAnimation::SetName(const std::string& _name)
{
  this->name = _name;
}

//////////////////////////////////////////////////
std::string NodeAnimation::GetName() const
{
  return this->name;
}

//////////////////////////////////////////////////
void NodeAnimation::AddKeyFrame(const double _time,
                                const ignition::math::Matrix4d &_trans)
{
  if (_time > this->length)
    this->length = _time;

  this->keyFrames[_time] = _trans;
}

//////////////////////////////////////////////////
void NodeAnimation::AddKeyFrame(const double _time,
    const ignition::math::Pose3d &_pose)
{
  ignition::math::Matrix4d mat(_pose.Rot());
  mat.Translate(_pose.Pos());

  this->AddKeyFrame(_time, mat);
}

//////////////////////////////////////////////////
unsigned int NodeAnimation::GetFrameCount() const
{
  return this->keyFrames.size();
}

//////////////////////////////////////////////////
void NodeAnimation::GetKeyFrame(const unsigned int _i, double &_time,
        ignition::math::Matrix4d &_trans) const
{
  if (_i >= this->keyFrames.size())
  {
    gzerr << "Invalid key frame index " << _i << "\n";
    _time = -1.0;
  }
  else
  {
    std::map<double, ignition::math::Matrix4d>::const_iterator iter =
                                                      this->keyFrames.begin();

    std::advance(iter, _i);

    _time = iter->first;
    _trans = iter->second;
  }
}

//////////////////////////////////////////////////
std::pair<double, ignition::math::Matrix4d> NodeAnimation::GetKeyFrame(
  const unsigned int _i) const
{
  double t;
  ignition::math::Matrix4d mat(ignition::math::Matrix4d::Identity);
  this->GetKeyFrame(_i, t, mat);

  return std::make_pair(t, mat);
}

//////////////////////////////////////////////////
double NodeAnimation::Length() const
{
  return this->length;
}

//////////////////////////////////////////////////
ignition::math::Matrix4d NodeAnimation::GetFrameAt(double _time, bool _loop) const
{
  double time = _time;
  if (time > this->length)
  {
    if (_loop)
      while (time > this->length)
        time = time - this->length;
    else
      time = this->length;
  }

  if (ignition::math::equal(time, this->length))
    return this->keyFrames.rbegin()->second;

  std::map<double, ignition::math::Matrix4d>::const_iterator it1 =
                                 this->keyFrames.upper_bound(time);

  if (it1 == this->keyFrames.begin() || ignition::math::equal(it1->first, time))
    return it1->second;

  std::map<double, ignition::math::Matrix4d>::const_iterator it2 = it1--;

  if (it1 == this->keyFrames.begin() || ignition::math::equal(it1->first, time))
    return it1->second;

  double nextKey = it2->first;
  ignition::math::Matrix4d nextTrans = it2->second;
  double prevKey = it1->first;
  ignition::math::Matrix4d prevTrans = it2->second;

  double t = (time - prevKey) / (nextKey - prevKey);
  GZ_ASSERT(t >= 0.0 && t <= 1.0, "t is not in the range 0.0..1.0");

  ignition::math::Vector3d nextPos = nextTrans.Translation();
  ignition::math::Vector3d prevPos = prevTrans.Translation();
  ignition::math::Vector3d pos = ignition::math::Vector3d(prevPos.x() +
      ((nextPos.x() - prevPos.x()) * t),
      prevPos.y() + ((nextPos.y() - prevPos.y()) * t),
      prevPos.z() + ((nextPos.z() - prevPos.z()) * t));

  ignition::math::Quaterniond nextRot = nextTrans.Rotation();
  ignition::math::Quaterniond prevRot = prevTrans.Rotation();
  ignition::math::Quaterniond rot = ignition::math::Quaterniond::Slerp(t,
      prevRot, nextRot, true);

  ignition::math::Matrix4d trans(rot);
  trans.Translate(pos);

  return trans;
}

//////////////////////////////////////////////////
void NodeAnimation::Scale(const double _scale)
{
  for (std::map<double, ignition::math::Matrix4d>::iterator iter = this->keyFrames.begin();
        iter != this->keyFrames.end(); ++iter)
  {
    ignition::math::Matrix4d *mat = &iter->second;
    ignition::math::Vector3d pos = mat->Translation();
    mat->Translate(pos * _scale);
  }
}

//////////////////////////////////////////////////
double NodeAnimation::GetTimeAtX(const double _x) const
{
  std::map<double, ignition::math::Matrix4d>::const_iterator it1 = this->keyFrames.begin();
  while (it1->second.Translation().x() < _x)
    ++it1;

  if (it1 == this->keyFrames.begin() ||
        ignition::math::equal(it1->second.Translation().x(), _x))
    return it1->first;

  std::map<double, ignition::math::Matrix4d>::const_iterator it2 = it1--;
  double x1 = it1->second.Translation().x();
  double x2 = it2->second.Translation().x();
  double t1 = it1->first;
  double t2 = it2->first;

  return t1 + ((t2 - t1) * (_x - x1) / (x2 - x1));
}

//////////////////////////////////////////////////
SkeletonAnimation::SkeletonAnimation(const std::string& _name)
{
  this->name = _name;
}

//////////////////////////////////////////////////
SkeletonAnimation::~SkeletonAnimation()
{
  this->animations.clear();
}

//////////////////////////////////////////////////
void SkeletonAnimation::SetName(const std::string& _name)
{
  this->name = _name;
}

//////////////////////////////////////////////////
std::string SkeletonAnimation::GetName() const
{
  return this->name;
}

//////////////////////////////////////////////////
unsigned int SkeletonAnimation::GetNodeCount() const
{
  return this->animations.size();
}

//////////////////////////////////////////////////
bool SkeletonAnimation::HasNode(const std::string& _node) const
{
  return (this->animations.find(_node) != this->animations.end());
}

//////////////////////////////////////////////////
void SkeletonAnimation::AddKeyFrame(const std::string& _node,
    const double _time, const ignition::math::Matrix4d &_mat)
{
  if (this->animations.find(_node) == this->animations.end())
    this->animations[_node] = new NodeAnimation(_node);

  if (_time > this->length)
    this->length = _time;

  this->animations[_node]->AddKeyFrame(_time, _mat);
}

//////////////////////////////////////////////////
void SkeletonAnimation::AddKeyFrame(const std::string& _node,
      const double _time, const ignition::math::Pose3d &_pose)
{
  if (this->animations.find(_node) == this->animations.end())
    this->animations[_node] = new NodeAnimation(_node);

  if (_time > this->length)
    this->length = _time;

  this->animations[_node]->AddKeyFrame(_time, _pose);
}

//////////////////////////////////////////////////
ignition::math::Matrix4d SkeletonAnimation::GetNodePoseAt(const std::string& _node,
                      const double _time, const bool _loop)
{
  ignition::math::Matrix4d mat;

  if (this->animations[_node])
    mat = this->animations[_node]->GetFrameAt(_time, _loop);

  return mat;
}

//////////////////////////////////////////////////
std::map<std::string, ignition::math::Matrix4d> SkeletonAnimation::GetPoseAt(
                      const double _time, const bool _loop) const
{
  ///  TODO need to make sure that all nodes have keyframes at the same
  ///  points in time and create the missing keyframes. if the animation
  ///  comes from bvh this is guaranteed, but if it's comming from collada
  ///  it's not guaranteed. fixing this will help not having to find the
  ///  prev and next keyframe for each node at each time step, but rather
  ///  doing it only once per time step.
  std::map<std::string, ignition::math::Matrix4d> pose;
  for (std::map<std::string, NodeAnimation*>::const_iterator iter =
          this->animations.begin(); iter != this->animations.end(); ++iter)
  {
    pose[iter->first] = iter->second->GetFrameAt(_time, _loop);
  }

  return pose;
}

//////////////////////////////////////////////////
std::map<std::string, ignition::math::Matrix4d> SkeletonAnimation::GetPoseAtX(
             const double _x, const std::string& _node, const bool _loop) const
{
  std::map<std::string, NodeAnimation*>::const_iterator nodeAnim =
      this->animations.find(_node);
  ignition::math::Matrix4d lastPos = nodeAnim->second->GetKeyFrame(
      nodeAnim->second->GetFrameCount() - 1).second;

  ignition::math::Matrix4d firstPos = nodeAnim->second->GetKeyFrame(0).second;

  double x = _x;
  if (x < firstPos.Translation().x())
    x = firstPos.Translation().x();

  double lastX = lastPos.Translation().x();
  if (x > lastX && !_loop)
    x = lastX;
  while (x > lastX)
    x -= lastX;

  double time = nodeAnim->second->GetTimeAtX(x);

  return this->GetPoseAt(time, _loop);
}

//////////////////////////////////////////////////
void SkeletonAnimation::Scale(const double _scale)
{
  for (std::map<std::string, NodeAnimation*>::iterator iter =
        this->animations.begin(); iter != this->animations.end(); ++iter)
    iter->second->Scale(_scale);
}

//////////////////////////////////////////////////
double SkeletonAnimation::Length() const
{
  return this->length;
}
