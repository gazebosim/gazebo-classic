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

#include <iostream>
#include <sstream>

#include "transport/Node.hh"
#include "gui/GuiEvents.hh"
#include "common/MouseEvent.hh"
#include "rendering/UserCamera.hh"

#include "gui/LightMaker.hh"

using namespace gazebo;
using namespace gui;

unsigned int LightMaker::counter = 0;

LightMaker::LightMaker() : EntityMaker()
{
  this->lightPub = this->node->Advertise<msgs::Light>("~/light");

  this->state = 0;

  msgs::Set(this->msg.mutable_diffuse(), common::Color(0.5, 0.5, 0.5, 1));
  msgs::Set(this->msg.mutable_specular(), common::Color(0.1, 0.1, 0.1, 1));

  this->msg.set_attenuation_constant(0.5);
  this->msg.set_attenuation_linear(0.01);
  this->msg.set_attenuation_quadratic(0.001);
  this->msg.set_range( 20 );

}

void LightMaker::Start(const rendering::UserCameraPtr _camera)
{
  this->camera = _camera;

  std::ostringstream stream;
  stream << "user_" << this->lightTypename << "_light_" << counter++;
  this->msg.set_name( stream.str() );
  this->state = 1;
}

void LightMaker::Stop()
{
  this->state = 0;
  gui::Events::moveModeSignal(true);
}

bool LightMaker::IsActive() const
{
  return this->state > 0;
}

void LightMaker::OnMousePush(const common::MouseEvent &_event)
{
  if (this->state == 0)
    return;

  math::Quaternion orient;
  math::Vector3 norm, point;
  norm.Set(0,0,1);

  point = this->camera->GetWorldPointOnPlane(_event.pressPos.x, 
      _event.pressPos.y, norm, 0);

  msgs::Set(this->msg.mutable_pose()->mutable_position(), point);
  msgs::Set(this->msg.mutable_pose()->mutable_orientation(), orient);
  this->msg.mutable_pose()->mutable_position()->set_z(1);
}

void LightMaker::OnMouseRelease(const common::MouseEvent &/*_event*/)
{
  if (this->state == 0)
    return;

  this->state++;

  this->CreateTheEntity();
  this->Stop();
}

void LightMaker::CreateTheEntity()
{
  this->lightPub->Publish(this->msg);
}
