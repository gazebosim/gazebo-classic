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
#include <ignition/math/Vector2.hh>

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
  this->typeString = TYPE_STRING;
}

//////////////////////////////////////////////////
FPSViewController::~FPSViewController()
{
}

void FPSViewController::Init()
{
}

//////////////////////////////////////////////////
void FPSViewController::Update()
{
}

//////////////////////////////////////////////////
void FPSViewController::HandleMouseEvent(const common::MouseEvent &_event)
{
  if (!this->enabled)
    return;

  ignition::math::Vector2i drag = _event.pos - _event.prevPos;

  ignition::math::Vector3d directionVec(0, 0, 0);

  if (_event.buttons & common::MouseEvent::LEFT)
  {
    this->camera->RotateYaw(IGN_DTOR(drag.X() * 0.1));
    this->camera->RotatePitch(IGN_DTOR(-drag.Y() * 0.1));
  }
  else if (_event.buttons & common::MouseEvent::RIGHT)
  {
    // interactively pan view
    directionVec.X(0);
    directionVec.Y(drag.X() * _event.moveScale);
    directionVec.Z(drag.Y() * _event.moveScale);
  }
  else if (_event.buttons & common::MouseEvent::MIDDLE)
  {
    directionVec.X(drag.Y() * _event.moveScale);
    directionVec.Y(0);
    directionVec.Z(0);
  }
  else if (_event.type == common::MouseEvent::SCROLL)
  {
    directionVec.X() -= 50.0 * _event.scroll.Y() * _event.moveScale;
    directionVec.Y(0);
    directionVec.Z(0);
  }

  this->camera->Translate(directionVec);
}

//////////////////////////////////////////////////
std::string FPSViewController::GetTypeString()
{
  return TYPE_STRING;
}

//////////////////////////////////////////////////
void FPSViewController::HandleKeyReleaseEvent(const std::string &/*_key*/)
{
}

//////////////////////////////////////////////////
void FPSViewController::HandleKeyPressEvent(const std::string &/*_key*/)
{
}
