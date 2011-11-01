#include <algorithm>

#include "math/Spline.hh"
#include "math/RotationSpline.hh"
#include "common/KeyFrame.hh"
#include "common/Animation.hh"

using namespace gazebo;
using namespace common;

namespace
{
struct KeyFrameTimeLess
{
  bool operator() (const common::KeyFrame *_kf, const common::KeyFrame *_kf2) const
  {
    return _kf->GetTime() < _kf2->GetTime();
  }
};
}


Animation::Animation(const std::string _name, double _length)
  : name(_name), length(_length)
{
  this->positionSpline = NULL;
  this->rotationSpline = NULL;
  this->timePos = 0;

  this->build = false;
  this->loop = true;
}

Animation::~Animation()
{
  delete this->positionSpline;
  delete this->rotationSpline;
}

double Animation::GetLength() const
{
  return this->length;
}

void Animation::SetLength(double _len)
{
  this->length = _len;
}

KeyFrame *Animation::CreateKeyFrame(double _time)
{
  KeyFrame *frame = new KeyFrame(_time);
  std::vector<KeyFrame*>::iterator iter =
    std::upper_bound(this->keyFrames.begin(), this->keyFrames.end(), frame, 
        KeyFrameTimeLess() );

  this->keyFrames.insert(iter, frame);
  this->build = true;

  return frame;
}

void Animation::SetTime(double _time)
{
  if (_time != this->timePos)
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

void Animation::AddTime(double _time)
{
  this->SetTime(this->timePos + _time);
}


void Animation::BuildInterpolationSplines() const
{
  if (!this->positionSpline)
    this->positionSpline = new math::Spline();

   if (!this->rotationSpline)
    this->rotationSpline = new math::RotationSpline();
  
  this->positionSpline->SetAutoCalculate(false);
  this->rotationSpline->SetAutoCalculate(false);

  this->positionSpline->Clear();
  this->rotationSpline->Clear();

  for (KeyFrame_V::const_iterator iter = this->keyFrames.begin();
       iter != this->keyFrames.end(); iter++)
  {
    this->positionSpline->AddPoint((*iter)->GetTranslate());
    this->rotationSpline->AddPoint((*iter)->GetRotation());
  }

  this->positionSpline->RecalcTangents();
  this->rotationSpline->RecalcTangents();
  this->build = false;
}

void Animation::GetInterpolatedKeyFrame(KeyFrame &_kf) const
{
  this->GetInterpolatedKeyFrame(this->timePos, _kf);
}

void Animation::GetInterpolatedKeyFrame( double _time, KeyFrame &_kf) const
{
  KeyFrame *k1, *k2;
  unsigned int firstKeyIndex;

  if (this->build)
    this->BuildInterpolationSplines();

  double t = this->GetKeyFramesAtTime( _time, &k1, &k2, firstKeyIndex);

  if (t == 0.0)
  {
    _kf.SetTranslate(k1->GetTranslate());
  }
  else
  {
    _kf.SetTranslate(this->positionSpline->Interpolate(firstKeyIndex, t));
    _kf.SetRotation(this->rotationSpline->Interpolate(firstKeyIndex, t));
  }
}

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

  if (t1 == t2)
    return 0.0;
  else
    return (_time - t1) / (t2 - t1);
}


