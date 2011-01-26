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

#include "Events.hh"
#include "Camera.hh"
#include "MouseEvent.hh"
#include "Simulator.hh"
#include "Visual.hh"
#include "World.hh"
#include "PointLightMaker.hh"

using namespace gazebo;

unsigned int PointLightMaker::counter = 0;

PointLightMaker::PointLightMaker()
  : EntityMaker()
{
  this->state = 0;

  this->msg.type = LightMsg::POINT;
  this->msg.diffuse.Set(0.5, 0.5, 0.5, 1);
  this->msg.specular.Set(0.1, 0.1, 0.1, 1);
  this->msg.attenuation.Set(0.5, 0.01, 0.001);
  this->msg.range = 20;
  this->msg.castShadows = false;
}

PointLightMaker::~PointLightMaker()
{
}

void PointLightMaker::Start( Scene *scene )
{
  std::ostringstream stream;
  stream << "user_point_light_" << counter++;
  this->msg.id = stream.str();
  this->state = 1;
}

void PointLightMaker::Stop()
{
  this->state = 0;
  Events::moveModeSignal(true);
}

bool PointLightMaker::IsActive() const
{
  return this->state > 0;
}

void PointLightMaker::MousePushCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  Vector3 norm;
  norm.Set(0,0,1);

  this->msg.pose.pos = event.camera->GetWorldPointOnPlane(event.pressPos.x, event.pressPos.y, norm, 0);
  this->msg.pose.pos.z = 1;
}

void PointLightMaker::MouseReleaseCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->state++;

  this->CreateTheEntity();
  this->Stop();
}

void PointLightMaker::MouseDragCB(const MouseEvent & /*event*/)
{
}

void PointLightMaker::CreateTheEntity()
{
  Simulator::Instance()->SendMessage(this->msg);
}
