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
void NodeAnimation::AddKeyFrame(const double _time, const math::Matrix4 &_trans)
{
  if (_time > this->length)
    this->length = _time;

  this->keyFrames[_time] = _trans;
}

//////////////////////////////////////////////////
void NodeAnimation::AddKeyFrame(const double _time, const math::Pose &_pose)
{
  math::Matrix4 mat(_pose.rot.GetAsMatrix4());
  mat.SetTranslate(_pose.pos);

  this->AddKeyFrame(_time, mat);
}

//////////////////////////////////////////////////
unsigned int NodeAnimation::GetFrameCount() const
{
  return this->keyFrames.size();
}

//////////////////////////////////////////////////
void NodeAnimation::GetKeyFrame(const unsigned int _i, double& _time,
        math::Matrix4& _trans) const
{
  if (_i >= this->keyFrames.size())
  {
    gzerr << "Invalid key frame index " << _i << "\n";
    _time = -1.0;
  }
  else
  {
    std::map<double, math::Matrix4>::const_iterator iter =
                                                      this->keyFrames.begin();

    std::advance(iter, _i);

    _time = iter->first;
    _trans = iter->second;
  }
}

//////////////////////////////////////////////////
std::pair<double, math::Matrix4> NodeAnimation::GetKeyFrame(
  const unsigned int _i) const
{
  double t;
  math::Matrix4 mat(math::Matrix4::IDENTITY);
  this->GetKeyFrame(_i, t, mat);

  return std::make_pair(t, mat);
}

//////////////////////////////////////////////////
double NodeAnimation::GetLength() const
{
  return this->length;
}

//////////////////////////////////////////////////
math::Matrix4 NodeAnimation::GetFrameAt(double _time, bool _loop) const
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

  if (math::equal(time, this->length))
    return this->keyFrames.rbegin()->second;

  std::map<double, math::Matrix4>::const_iterator it1 =
                                 this->keyFrames.upper_bound(time);

  if (it1 == this->keyFrames.begin() || math::equal(it1->first, time))
    return it1->second;

  std::map<double, math::Matrix4>::const_iterator it2 = it1--;

  if (it1 == this->keyFrames.begin() || math::equal(it1->first, time))
    return it1->second;

  double nextKey = it2->first;
  math::Matrix4 nextTrans = it2->second;
  double prevKey = it1->first;
  math::Matrix4 prevTrans = it2->second;

  double t = (time - prevKey) / (nextKey - prevKey);
  GZ_ASSERT(t >= 0.0 && t <= 1.0, "t is not in the range 0.0..1.0");

  math::Vector3 nextPos = nextTrans.GetTranslation();
  math::Vector3 prevPos = prevTrans.GetTranslation();
  math::Vector3 pos = math::Vector3(prevPos.x + ((nextPos.x - prevPos.x) * t),
                                    prevPos.y + ((nextPos.y - prevPos.y) * t),
                                    prevPos.z + ((nextPos.z - prevPos.z) * t));

  math::Quaternion nextRot = nextTrans.GetRotation();
  math::Quaternion prevRot = prevTrans.GetRotation();
  math::Quaternion rot = math::Quaternion::Slerp(t, prevRot, nextRot, true);

  math::Matrix4 trans(rot.GetAsMatrix4());
  trans.SetTranslate(pos);

  return trans;
}

//////////////////////////////////////////////////
void NodeAnimation::Scale(const double _scale)
{
  for (std::map<double, math::Matrix4>::iterator iter = this->keyFrames.begin();
        iter != this->keyFrames.end(); ++iter)
  {
    math::Matrix4 *mat = &iter->second;
    math::Vector3 pos = mat->GetTranslation();
    mat->SetTranslate(pos * _scale);
  }
}

//////////////////////////////////////////////////
double NodeAnimation::GetTimeAtX(const double _x) const
{
  std::map<double, math::Matrix4>::const_iterator it1 = this->keyFrames.begin();
  while (it1->second.GetTranslation().x < _x)
    ++it1;

  if (it1 == this->keyFrames.begin() ||
        math::equal(it1->second.GetTranslation().x, _x))
    return it1->first;

  std::map<double, math::Matrix4>::const_iterator it2 = it1--;
  double x1 = it1->second.GetTranslation().x;
  double x2 = it2->second.GetTranslation().x;
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
    const double _time, const math::Matrix4 &_mat)
{
  if (this->animations.find(_node) == this->animations.end())
    this->animations[_node] = new NodeAnimation(_node);

  if (_time > this->length)
    this->length = _time;

  this->animations[_node]->AddKeyFrame(_time, _mat);
}

//////////////////////////////////////////////////
void SkeletonAnimation::AddKeyFrame(const std::string& _node,
      const double _time, const math::Pose &_pose)
{
  if (this->animations.find(_node) == this->animations.end())
    this->animations[_node] = new NodeAnimation(_node);

  if (_time > this->length)
    this->length = _time;

  this->animations[_node]->AddKeyFrame(_time, _pose);
}

//////////////////////////////////////////////////
math::Matrix4 SkeletonAnimation::GetNodePoseAt(const std::string& _node,
                      const double _time, const bool _loop)
{
  math::Matrix4 mat;

  if (this->animations[_node])
    mat = this->animations[_node]->GetFrameAt(_time, _loop);

  return mat;
}

//////////////////////////////////////////////////
std::map<std::string, math::Matrix4> SkeletonAnimation::GetPoseAt(
                      const double _time, const bool _loop) const
{
  ///  TODO need to make sure that all nodes have keyframes at the same
  ///  points in time and create the missing keyframes. if the animation
  ///  comes from bvh this is guaranteed, but if it's comming from collada
  ///  it's not guaranteed. fixing this will help not having to find the
  ///  prev and next keyframe for each node at each time step, but rather
  ///  doing it only once per time step.
  std::map<std::string, math::Matrix4> pose;
  for (std::map<std::string, NodeAnimation*>::const_iterator iter =
          this->animations.begin(); iter != this->animations.end(); ++iter)
  {
    pose[iter->first] = iter->second->GetFrameAt(_time, _loop);
  }

  return pose;
}

//////////////////////////////////////////////////
std::map<std::string, math::Matrix4> SkeletonAnimation::GetPoseAtX(
             const double _x, const std::string& _node, const bool _loop) const
{
  std::map<std::string, NodeAnimation*>::const_iterator nodeAnim =
      this->animations.find(_node);
  math::Matrix4 lastPos = nodeAnim->second->GetKeyFrame(
      nodeAnim->second->GetFrameCount() - 1).second;

  math::Matrix4 firstPos = nodeAnim->second->GetKeyFrame(0).second;

  double x = _x;
  if (x < firstPos.GetTranslation().x)
    x = firstPos.GetTranslation().x;

  double lastX = lastPos.GetTranslation().x;
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
double SkeletonAnimation::GetLength() const
{
  return this->length;
}
