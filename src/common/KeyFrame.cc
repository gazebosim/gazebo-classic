#include "KeyFrame.hh"

using namespace gazebo;
using namespace common;

KeyFrame::KeyFrame(double _time)
  : time(_time)
{
}

KeyFrame::~KeyFrame()
{
}

double KeyFrame::GetTime() const
{
  return this->time;
}



PoseKeyFrame::PoseKeyFrame(double _time)
  : KeyFrame(_time)
{
}

PoseKeyFrame::~PoseKeyFrame()
{
}

void PoseKeyFrame::SetTranslate(const math::Vector3 &_trans)
{
  this->translate = _trans;
}

const math::Vector3 &PoseKeyFrame::GetTranslate() const
{
  return this->translate;
}

void PoseKeyFrame::SetRotation(const math::Quaternion &_rot)
{
  this->rotate = _rot;
}

const math::Quaternion &PoseKeyFrame::GetRotation() const
{
  return this->rotate;
}

NumericKeyFrame::NumericKeyFrame(double _time)
  : KeyFrame(_time)
{
}

NumericKeyFrame::~NumericKeyFrame()
{
}

void NumericKeyFrame::SetValue(const double &_value)
{
  this->value = _value;
}

const double &NumericKeyFrame::GetValue() const
{
  return this->value;
}
