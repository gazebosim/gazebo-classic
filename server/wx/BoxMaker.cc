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

#include "Messages.hh"
#include "Camera.hh"
#include "Events.hh"
#include "MouseEvent.hh"
#include "Simulator.hh"
#include "Scene.hh"
#include "World.hh"
#include "BoxMaker.hh"

using namespace gazebo;

unsigned int BoxMaker::counter = 0;

BoxMaker::BoxMaker() 
: EntityMaker()
{
  this->state = 0;
  this->visualMsg = new VisualMsg();
  this->visualMsg->render = RENDERING_MESH_RESOURCE;
  this->visualMsg->mesh = "unit_box_U1V1";
  this->visualMsg->material = "Gazebo/TurquoiseGlowOutline";
}

BoxMaker::~BoxMaker()
{
  delete this->visualMsg;
}

void BoxMaker::Start(Scene *scene)
{
  std::ostringstream stream;
  stream << "user_box_" << counter++;
  this->visualMsg->id = stream.str();

  this->state = 1;
}

void BoxMaker::Stop()
{
  this->visualMsg->action = VisualMsg::DELETE;
  Simulator::Instance()->SendMessage( *this->visualMsg );
  this->visualMsg->action = VisualMsg::UPDATE;

  this->state = 0;
  Events::moveModeSignal(true);
}

bool BoxMaker::IsActive() const
{
  return this->state > 0;
}

void BoxMaker::MousePushCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->mousePushPos = event.pressPos;
}

void BoxMaker::MouseReleaseCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->state++;

  if (this->state == 3)
  {
    this->CreateTheEntity();
    this->Stop();
  }
}

void BoxMaker::MouseDragCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  Vector3 norm(0,0,1);
  Vector3 p1, p2;

  p1 = event.camera->GetWorldPointOnPlane(this->mousePushPos.x, this->mousePushPos.y, norm, 0);
  p1 = this->GetSnappedPoint( p1 );

  p2 = event.camera->GetWorldPointOnPlane(event.pos.x, event.pos.y ,norm, 0);
  p2 = this->GetSnappedPoint( p2 );

  if (this->state == 1)
    this->visualMsg->pose.pos = p1;

  Vector3 scale = p1-p2;
  Vector3 p = this->visualMsg->pose.pos;

  if (this->state == 1)
  {
    scale.z = 0.01;
    p.x = p1.x - scale.x/2.0;
    p.y = p1.y - scale.y/2.0;
  }
  else
  {
    scale = this->visualMsg->scale;
    scale.z = (this->mousePushPos.y - event.pos.y)*0.01;
    p.z = scale.z/2.0;
  }

  this->visualMsg->pose.pos = p;
  this->visualMsg->scale = scale;

  Simulator::Instance()->SendMessage(*this->visualMsg);
}

void BoxMaker::CreateTheEntity()
{
  InsertModelMsg msg;

  std::ostringstream newModelStr;

  newModelStr << "<?xml version='1.0'?>";

  newModelStr << "<model type='physical' name='" << this->visualMsg->id << "'>\
    <xyz>" << this->visualMsg->pose.pos << "</xyz>\
    <body name='body'>\
    <geom type='box' name='geom'>\
    <size>" << this->visualMsg->scale << "</size>\
    <mass>0.5</mass>\
    <visual>\
    <mesh>unit_box_U1V1</mesh>\
    <scale>" << this->visualMsg->scale << "</scale>\
    <material>Gazebo/Grey</material>\
    <shader>pixel</shader>\
    </visual>\
    </geom>\
    </body>\
    </model>";

  msg.xmlStr = newModelStr.str();

  this->visualMsg->action = VisualMsg::DELETE;
  Simulator::Instance()->SendMessage( *this->visualMsg );

  Simulator::Instance()->SendMessage( msg );
}
