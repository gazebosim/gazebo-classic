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
#include "gazebo/math/Vector2i.hh"
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

  math::Vector2i drag = _event.pos - _event.prevPos;

  math::Vector2d directionVec(0, 0);

  //TODO: take care of race conditions
  math::Pose velocity = this->camera->GetVelocity();
  if((_event.buttons & common::MouseEvent::LEFT)){
    float angularVel = 0.075;
    directionVec.x = -angularVel * drag.x;
    directionVec.y = angularVel * drag.y;
    math::Pose velocityIncrement = math::Pose(0, 0, 0, 0, directionVec.x, directionVec.y );
    this->camera->SetVelocity(velocityIncrement);
  }
  else 
  {
    math::Pose newVelocity = velocity;
    newVelocity.rot = math::Quaternion(0, 0, 0);
    this->camera->SetVelocity(newVelocity);
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
  if(_key.compare("w") == 0 || _key.compare("a") == 0 ||
     _key.compare("s") == 0 || _key.compare("d") == 0)
  {
    this->camera->SetVelocity(math::Pose(0, 0, 0, 0, 0, 0));
  }
}

//////////////////////////////////////////////////
void FPSViewController::HandleKeyPressEvent(const std::string & _key)
{
  float linearVelocity = 0.9;
  if(_key.compare("w") == 0)
  {
    this->camera->SetVelocity(math::Pose(linearVelocity, 0, 0, 0, 0, 0));
  }
  else if (_key.compare("a") == 0)
  {
    this->camera->SetVelocity(math::Pose(0, linearVelocity, 0, 0, 0, 0));
  }
  else if (_key.compare("s") == 0)
  {
    this->camera->SetVelocity(math::Pose(-linearVelocity, 0, 0, 0, 0, 0));
  }
  else if (_key.compare("d") == 0)
  {
    this->camera->SetVelocity(math::Pose(0, -linearVelocity, 0, 0, 0, 0));
  }
}
