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

#include "common/Events.hh"
#include "common/MouseEvent.hh"

#include "gui/SpotLightMaker.hh"

using namespace gazebo;
using namespace gui;


unsigned int SpotLightMaker::counter = 0;

SpotLightMaker::SpotLightMaker()
  : EntityMaker()
{
  this->state = 0;

  this->msg.set_type( msgs::Light::SPOT );
  common::Message::Set(this->msg.mutable_diffuse(), common::Color(0.5, 0.5, 0.5, 1));
  common::Message::Set(this->msg.mutable_specular(), common::Color(0.1, 0.1, 0.1, 1));
  common::Message::Set(this->msg.mutable_attenuation(), math::Vector3(0.5, 0.01, 0.0));
  common::Message::Set(this->msg.mutable_direction(), math::Vector3(0,0,-1));
  this->msg.set_range( 20 );
  this->msg.set_cast_shadows( false );
  this->msg.set_spot_inner_angle( 20 );
  this->msg.set_spot_outer_angle( 40 );
  this->msg.set_spot_falloff( 1.0 );
}

SpotLightMaker::~SpotLightMaker()
{
}

void SpotLightMaker::Start()
{
  std::ostringstream stream;
  stream << "user_spot_light_" << counter++;
  this->msg.mutable_header()->set_str_id( stream.str() );
  this->state = 1;
}

void SpotLightMaker::Stop()
{
  this->state = 0;
  event::Events::moveModeSignal(true);
}

bool SpotLightMaker::IsActive() const
{
  return this->state > 0;
}

void SpotLightMaker::MousePushCB(const common::MouseEvent &event)
{
  if (this->state == 0)
    return;

  math::Vector3 norm;
  norm.Set(0,0,1);

  /* NATY: Fix camera issue
  common::Message::Set(this->msg.mutable_pose()->mutable_position(), event.camera->GetWorldPointOnPlane(event.pressPos.x, event.pressPos.y, norm, 0) );
  this->msg.mutable_pose()->mutable_position()->set_z( 1.0 );
  */
}

void SpotLightMaker::MouseReleaseCB(const common::MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->state++;

  this->CreateTheEntity();
  this->Stop();
}

void SpotLightMaker::MouseDragCB(const common::MouseEvent & /*event*/)
{
}

void SpotLightMaker::CreateTheEntity()
{
  common::Message::Stamp(this->msg.mutable_header());
  //Simulator::Instance()->SendMessage(this->msg);
}
