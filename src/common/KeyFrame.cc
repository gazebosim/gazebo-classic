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

void KeyFrame::SetTranslate(const math::Vector3 &_trans)
{
  this->translate = _trans;
}

const math::Vector3 &KeyFrame::GetTranslate() const
{
  return this->translate;
}

void KeyFrame::SetRotation(const math::Quaternion &_rot)
{
  this->rotate = _rot;
}

const math::Quaternion &KeyFrame::GetRotation() const
{
  return this->rotate;
}
