/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include <common/SkeletonAnimation.hh>
#include <common/Console.hh>

using namespace gazebo;
using namespace common;


//////////////////////////////////////////////////
NodeAnimation::NodeAnimation(const std::string _name)
{
  this->name = _name;
}

//////////////////////////////////////////////////
NodeAnimation::~NodeAnimation()
{
  this->keyFrames.clear();
}

//////////////////////////////////////////////////
void NodeAnimation::SetName(const std::string _name)
{
  this->name = _name;
}

//////////////////////////////////////////////////
std::string NodeAnimation::GetName() const
{
  return this->name;
}

//////////////////////////////////////////////////
void NodeAnimation::AddKeyFrame(const double _time, const math::Matrix4 _trans)
{
  if (_time > this->length)
    this->length = _time;

  this->keyFrames[_time] = _trans;
}

//////////////////////////////////////////////////
void NodeAnimation::AddKeyFrame(const double _time, const math::Pose _pose)
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
void NodeAnimation::GetKeyFrame(const unsigned int _i, double &_time,
        math::Matrix4 &_trans) const
{
  if (_i > this->keyFrames.size())
  {
    gzerr << "Invalid key frame index " << _i << "\n";
    _time = -1.0;
  }
  else
  {
    std::map<double, math::Matrix4>::const_iterator iter =
                                                      this->keyFrames.begin();
    unsigned int i = 0;
    while (i < _i)
    {
      ++iter;
      i++;
    }
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

  std::map<double, math::Matrix4>::const_iterator nextKey =
                                                  this->keyFrames.begin();

  std::map<double, math::Matrix4>::const_iterator prevKey =
                                                  this->keyFrames.end();
  --prevKey;

  while (nextKey->first < time)
  {
    prevKey = nextKey;
    ++nextKey;
  }
  double nextKeyTime = nextKey->first;
  math::Matrix4 nextKeyT = nextKey->second;
  double prevKeyTime = prevKey->first;
  math::Matrix4 prevKeyT = prevKey->second;

  if (prevKeyTime > nextKeyTime || math::equal(nextKeyTime, time))
  {
    ///  the requested time is before the first key frame
    ///  or the requested time is a key frame
    return nextKeyT;
  }

  double t = (time - prevKeyTime) / (nextKeyTime - prevKeyTime);
  assert(t > 0.0 && t < 1.0);

  math::Vector3 nextPos = nextKeyT.GetTranslation();
  math::Vector3 prevPos = prevKeyT.GetTranslation();
  math::Vector3 pos = math::Vector3(prevPos.x + ((nextPos.x - prevPos.x) * t),
                                    prevPos.y + ((nextPos.y - prevPos.y) * t),
                                    prevPos.z + ((nextPos.z - prevPos.z) * t));

  math::Quaternion nextRot = nextKeyT.GetRotation();
  math::Quaternion prevRot = prevKeyT.GetRotation();
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
SkeletonAnimation::SkeletonAnimation(const std::string _name)
{
  this->name = _name;
}

//////////////////////////////////////////////////
SkeletonAnimation::~SkeletonAnimation()
{
  this->animations.clear();
}

//////////////////////////////////////////////////
void SkeletonAnimation::SetName(const std::string _name)
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
bool SkeletonAnimation::HasNode(const std::string _node) const
{
  return (this->animations.find(_node) != this->animations.end());
}

//////////////////////////////////////////////////
void SkeletonAnimation::AddKeyFrame(const std::string _node, const double _time,
                      const math::Matrix4 _mat)
{
  if (this->animations.find(_node) == this->animations.end())
    this->animations[_node] = new NodeAnimation(_node);

  if (_time > this->length)
    this->length = _time;

  this->animations[_node]->AddKeyFrame(_time, _mat);
}

//////////////////////////////////////////////////
void SkeletonAnimation::AddKeyFrame(const std::string _node, const double _time,
                      const math::Pose _pose)
{
  if (this->animations.find(_node) == this->animations.end())
    this->animations[_node] = new NodeAnimation(_node);

  if (_time > this->length)
    this->length = _time;

  this->animations[_node]->AddKeyFrame(_time, _pose);
}

//////////////////////////////////////////////////
math::Matrix4 SkeletonAnimation::GetNodePoseAt(const std::string _node,
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
  std::map<std::string, math::Matrix4> pose;
  for (std::map<std::string, NodeAnimation*>::const_iterator iter =
          this->animations.begin(); iter != this->animations.end(); ++iter)
  {
    pose[iter->first] = iter->second->GetFrameAt(_time, _loop);
  }

  return pose;
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
