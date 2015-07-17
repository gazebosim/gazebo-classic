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
#include <algorithm>

#include <ignition/math/Spline.hh>
#include <ignition/math/RotationSpline.hh>

#include "gazebo/common/Console.hh"
#include "gazebo/common/KeyFrame.hh"
#include "gazebo/common/Animation.hh"

using namespace gazebo;
using namespace common;

namespace
{
  struct KeyFrameTimeLess
  {
    bool operator() (const common::KeyFrame *_kf,
        const common::KeyFrame *_kf2) const
    {
      return _kf->GetTime() < _kf2->GetTime();
    }
  };
}


/////////////////////////////////////////////////
Animation::Animation(const std::string &_name, double _length, bool _loop)
: name(_name), length(_length), loop(_loop)
{
  this->timePos = 0;
  this->build = false;
}

/////////////////////////////////////////////////
Animation::~Animation()
{
}

/////////////////////////////////////////////////
double Animation::GetLength() const
{
  return this->length;
}

/////////////////////////////////////////////////
void Animation::SetLength(double _len)
{
  this->length = _len;
}

/////////////////////////////////////////////////
void Animation::SetTime(double _time)
{
  if (!ignition::math::equal(_time, this->timePos))
  {
    this->timePos = _time;
    if (this->loop)
    {
      this->timePos = fmod(this->timePos, this->length);
      if (this->timePos < 0)
        this->timePos += this->length;
    }
    else
    {
      if (this->timePos < 0)
        this->timePos = 0;
      else if (this->timePos > this->length)
        this->timePos = this->length;
    }
  }
}

/////////////////////////////////////////////////
void Animation::AddTime(double _time)
{
  this->SetTime(this->timePos + _time);
}

/////////////////////////////////////////////////
double Animation::GetTime() const
{
  return this->timePos;
}

/////////////////////////////////////////////////
unsigned int Animation::GetKeyFrameCount() const
{
  return this->keyFrames.size();
}

/////////////////////////////////////////////////
KeyFrame *Animation::GetKeyFrame(unsigned int _index) const
{
  KeyFrame *result = NULL;

  if (_index < this->keyFrames.size())
    result = this->keyFrames[_index];
  else
  {
    gzerr << "Key frame index[" << _index
          << "] is larger than key frame array size["
          << this->keyFrames.size() << "]\n";
  }

  return result;
}

/////////////////////////////////////////////////
double Animation::GetKeyFramesAtTime(double _time, KeyFrame **_kf1,
    KeyFrame **_kf2,
    unsigned int &_firstKeyIndex) const
{
  // Parametric time
  // t1 = time of previous keyframe
  // t2 = time of next keyframe
  double t1, t2;

  // Find first key frame after or on current time
  while (_time > this->length && this->length > 0.0)
    _time -= this->length;

  KeyFrame_V::const_iterator iter;
  KeyFrame timeKey(_time);
  iter = std::lower_bound(this->keyFrames.begin(), this->keyFrames.end(),
      &timeKey, KeyFrameTimeLess());

  if (iter == this->keyFrames.end())
  {
    // There is no keyframe after this time, wrap back to first
    *_kf2 = this->keyFrames.front();
    t2 = this->length + (*_kf2)->GetTime();

    // Use the last keyframe as the previous keyframe
    --iter;
  }
  else
  {
    *_kf2 = *iter;
    t2 = (*_kf2)->GetTime();

    // Find last keyframe before or on current time
    if (iter != this->keyFrames.begin() && _time < (*iter)->GetTime())
      --iter;
  }

  _firstKeyIndex = std::distance(this->keyFrames.begin(), iter);

  *_kf1 = *iter;
  t1 = (*_kf1)->GetTime();

  if (ignition::math::equal(t1, t2))
    return 0.0;
  else
    return (_time - t1) / (t2 - t1);
}






/////////////////////////////////////////////////
PoseAnimation::PoseAnimation(const std::string &_name,
    double _length, bool _loop)
: Animation(_name, _length, _loop)
{
  this->positionSpline = NULL;
  this->rotationSpline = NULL;
}

/////////////////////////////////////////////////
PoseAnimation::~PoseAnimation()
{
  delete this->positionSpline;
  delete this->rotationSpline;
}

/////////////////////////////////////////////////
PoseKeyFrame *PoseAnimation::CreateKeyFrame(double _time)
{
  PoseKeyFrame *frame = new PoseKeyFrame(_time);
  std::vector<KeyFrame*>::iterator iter =
    std::upper_bound(this->keyFrames.begin(), this->keyFrames.end(),
        reinterpret_cast<KeyFrame*>(frame), KeyFrameTimeLess());

  this->keyFrames.insert(iter, frame);
  this->build = true;

  return frame;
}

/////////////////////////////////////////////////
void PoseAnimation::BuildInterpolationSplines() const
{
  if (!this->positionSpline)
    this->positionSpline = new ignition::math::Spline();

  if (!this->rotationSpline)
    this->rotationSpline = new ignition::math::RotationSpline();

  this->positionSpline->AutoCalculate(false);
  this->rotationSpline->AutoCalculate(false);

  this->positionSpline->Clear();
  this->rotationSpline->Clear();

  for (KeyFrame_V::const_iterator iter = this->keyFrames.begin();
      iter != this->keyFrames.end(); ++iter)
  {
    PoseKeyFrame *pkey = reinterpret_cast<PoseKeyFrame*>(*iter);
    this->positionSpline->AddPoint(pkey->Translation());
    this->rotationSpline->AddPoint(pkey->Rotation());
  }

  this->positionSpline->RecalcTangents();
  this->rotationSpline->RecalcTangents();
  this->build = false;
}

/////////////////////////////////////////////////
void PoseAnimation::GetInterpolatedKeyFrame(PoseKeyFrame &_kf) const
{
  this->GetInterpolatedKeyFrame(this->timePos, _kf);
}

/////////////////////////////////////////////////
void PoseAnimation::GetInterpolatedKeyFrame(double _time,
                                            PoseKeyFrame &_kf) const
{
  KeyFrame *kBase1, *kBase2;
  PoseKeyFrame *k1;
  unsigned int firstKeyIndex;

  if (this->build)
    this->BuildInterpolationSplines();

  double t = this->GetKeyFramesAtTime(_time, &kBase1, &kBase2, firstKeyIndex);

  k1 = reinterpret_cast<PoseKeyFrame*>(kBase1);

  if (ignition::math::equal(t, 0.0))
  {
    _kf.Translation(k1->Translation());
    _kf.Rotation(k1->Rotation());
  }
  else
  {
    _kf.Translation(this->positionSpline->Interpolate(firstKeyIndex, t));
    _kf.Rotation(this->rotationSpline->Interpolate(firstKeyIndex, t));
  }
}








/////////////////////////////////////////////////
NumericAnimation::NumericAnimation(const std::string &_name,
    double _length, bool _loop)
: Animation(_name, _length, _loop)
{
}

/////////////////////////////////////////////////
NumericAnimation::~NumericAnimation()
{
}

/////////////////////////////////////////////////
NumericKeyFrame *NumericAnimation::CreateKeyFrame(double _time)
{
  NumericKeyFrame *frame = new NumericKeyFrame(_time);
  std::vector<KeyFrame*>::iterator iter =
    std::upper_bound(this->keyFrames.begin(), this->keyFrames.end(),
        reinterpret_cast<KeyFrame*>(frame),
        KeyFrameTimeLess());

  this->keyFrames.insert(iter, frame);
  this->build = true;
  return frame;
}

/////////////////////////////////////////////////
void NumericAnimation::GetInterpolatedKeyFrame(NumericKeyFrame &_kf) const
{
  // Keyframe pointers
  KeyFrame *kBase1, *kBase2;
  NumericKeyFrame *k1, *k2;
  unsigned int firstKeyIndex;

  double t;
  t = this->GetKeyFramesAtTime(this->timePos, &kBase1, &kBase2, firstKeyIndex);

  k1 = reinterpret_cast<NumericKeyFrame*>(kBase1);
  k2 = reinterpret_cast<NumericKeyFrame*>(kBase2);

  if (ignition::math::equal(t, 0.0))
  {
    // Just use k1
    _kf.SetValue(k1->GetValue());
  }
  else
  {
    // Interpolate by t
    double diff = k2->GetValue() - k1->GetValue();
    _kf.SetValue(k1->GetValue() + diff * t);
  }
}
