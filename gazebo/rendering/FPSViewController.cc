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
  : ViewController(_camera), velocity(math::Pose(0, 0, 0, 0, 0, 0))
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
  //std::cout << "Updating" << std::endl;
  if ( this->velocity != math::Pose(0, 0, 0, 0, 0, 0) )
  {
    // Move based on the camera's current velocity
    // Calculate delta based on frame rate
    common::Time interval = common::Time::GetWallTime() -
                            this->camera->GetLastRenderWallTime();
    float dt = interval.Float();

    math::Vector3 translate(velocity.pos[0]*dt, velocity.pos[1]*dt,
                             velocity.pos[2]*dt);
    this->camera->Translate(translate);
    this->camera->RotatePitch(velocity.rot.GetPitch()*dt);
    this->camera->RotateYaw(velocity.rot.GetYaw()*dt);
  }
}


//////////////////////////////////////////////////
math::Pose FPSViewController::GetVelocity() const
{
  return this->velocity;
}

//////////////////////////////////////////////////
void FPSViewController::SetVelocity(const math::Pose &_velocity)
{
  this->velocity = _velocity;
}

//////////////////////////////////////////////////
void FPSViewController::HandleMouseEvent(const common::MouseEvent &_event)
{
  if (!this->enabled)
    return;

  math::Vector2i drag = _event.pos - _event.prevPos;

  math::Vector3 directionVec(0, 0, 0);

  if (_event.buttons & common::MouseEvent::LEFT)
  {
    this->camera->RotateYaw(GZ_DTOR(drag.x * 0.1));
    this->camera->RotatePitch(GZ_DTOR(-drag.y * 0.1));
  }
  else if (_event.buttons & common::MouseEvent::RIGHT)
  {
    // interactively pan view
    directionVec.x = 0;
    directionVec.y =  drag.x * _event.moveScale;
    directionVec.z =  drag.y * _event.moveScale;
  }
  else if (_event.buttons & common::MouseEvent::MIDDLE)
  {
    directionVec.x =  drag.y * _event.moveScale;
    directionVec.y =  0;
    directionVec.z =  0;
  }
  else if (_event.type == common::MouseEvent::SCROLL)
  {
    directionVec.x -=  50.0 * _event.scroll.y * _event.moveScale;
    directionVec.y =  0;
    directionVec.z =  0;
  }

  //if(_event.type == common::MouseEvent::MOVE){

    this->camera->Translate(directionVec);
  //}

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
    this->SetVelocity(math::Pose(0, 0, 0, 0, 0, 0));
  }
}

//////////////////////////////////////////////////
void FPSViewController::HandleKeyPressEvent(const std::string & _key)
{
  float linearVelocity = 0.75;
  if(_key.compare("w") == 0)
  {
    this->SetVelocity(math::Pose(linearVelocity, 0, 0, 0, 0, 0));
  }
  else if (_key.compare("a") == 0)
  {
    this->SetVelocity(math::Pose(0, -linearVelocity, 0, 0, 0, 0));
  }
  else if (_key.compare("s") == 0)
  {
    this->SetVelocity(math::Pose(linearVelocity, 0, 0, 0, 0, 0));
  }
  else if (_key.compare("d") == 0)
  {
    this->SetVelocity(math::Pose(0, -linearVelocity, 0, 0, 0, 0));
  }
}
