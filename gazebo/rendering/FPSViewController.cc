/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <ignition/common/Profiler.hh>

#include "gazebo/common/MouseEvent.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/FPSViewController.hh"

#define TYPE_STRING "fps"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
FPSViewController::FPSViewController(UserCameraPtr _camera)
  : ViewController(_camera)
{
  this->xVelocityFactor = 1.0;
  this->yVelocityFactor = 0.8;

  this->typeString = TYPE_STRING;
}

//////////////////////////////////////////////////
FPSViewController::~FPSViewController()
{
}

//////////////////////////////////////////////////
void FPSViewController::Init()
{
  this->xVelocityFactor = 1.0;
  this->yVelocityFactor = 0.8;

  this->xVelocity = ignition::math::Vector3d::Zero;
  this->yVelocity = ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
void FPSViewController::Update()
{
  IGN_PROFILE("rendering::FPSViewController::Update");
  if (this->xVelocity != ignition::math::Vector3d::Zero ||
      this->yVelocity != ignition::math::Vector3d::Zero)
  {
    // Move based on the camera's current velocity
    // Calculate delta based on frame rate
    common::Time interval = common::Time::GetWallTime() -
      this->camera->LastRenderWallTime();
    float dt = interval.Float();

    ignition::math::Vector3d trans = this->xVelocity + this->yVelocity;
    this->camera->Translate(trans * dt);
  }
}

//////////////////////////////////////////////////
void FPSViewController::HandleMouseEvent(const common::MouseEvent &_event)
{
  if (!this->enabled)
    return;

  ignition::math::Vector2i drag = _event.Pos() - _event.PrevPos();

  if (_event.Buttons() & common::MouseEvent::LEFT)
  {
    this->camera->Yaw(static_cast<ignition::math::Angle>(
          IGN_DTOR(-drag.X()*0.1)));
    this->camera->Pitch(static_cast<ignition::math::Angle>(
          IGN_DTOR(drag.Y()*0.1)));
  }
  else if (_event.Type() == common::MouseEvent::SCROLL)
  {
    if (_event.Scroll().Y() < 0)
    {
      this->xVelocityFactor *= 1.05;
      this->yVelocityFactor *= 1.05;

      this->xVelocity *= 1.05;
      this->yVelocity *= 1.05;
    }
    else
    {
      this->xVelocityFactor *= 0.95;
      this->yVelocityFactor *= 0.95;

      this->xVelocity *= 0.95;
      this->yVelocity *= 0.95;
    }
  }
}

//////////////////////////////////////////////////
std::string FPSViewController::GetTypeString()
{
  return TYPE_STRING;
}

//////////////////////////////////////////////////
void FPSViewController::HandleKeyReleaseEvent(const std::string & _key)
{
  if (_key.compare("w") == 0 || _key.compare("s") == 0)
    this->xVelocity = ignition::math::Vector3d::Zero;

  if (_key.compare("a") == 0 || _key.compare("d") == 0)
    this->yVelocity = ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
void FPSViewController::HandleKeyPressEvent(const std::string & _key)
{
  if (_key.compare("w") == 0)
    this->xVelocity = ignition::math::Vector3d(this->xVelocityFactor, 0, 0);
  else if (_key.compare("s") == 0)
    this->xVelocity = ignition::math::Vector3d(-this->xVelocityFactor, 0, 0);

  if (_key.compare("a") == 0)
    this->yVelocity = ignition::math::Vector3d(0, this->yVelocityFactor, 0);
  else if (_key.compare("d") == 0)
    this->yVelocity = ignition::math::Vector3d(0, -this->yVelocityFactor, 0);
}
