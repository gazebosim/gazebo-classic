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
#include "Scene.hh"
#include "Events.hh"
#include "MouseEvent.hh"
#include "Simulator.hh"
#include "World.hh"
#include "SphereMaker.hh"

using namespace gazebo;

unsigned int SphereMaker::counter = 0;

SphereMaker::SphereMaker() 
: EntityMaker()
{
  this->state = 0;
  this->visualMsg = new VisualMsg();
  this->visualMsg->render = RENDERING_MESH_RESOURCE;
  this->visualMsg->mesh = "unit_sphere";
  this->visualMsg->material = "Gazebo/TurquoiseGlowOutline";
}

SphereMaker::~SphereMaker()
{
  delete this->visualMsg;
}

void SphereMaker::Start(Scene *scene)
{
  std::ostringstream stream;
  stream << "user_sphere_" << counter++;
  this->visualMsg->id = stream.str();

  this->state = 1;
}

void SphereMaker::Stop()
{
  this->visualMsg->action = VisualMsg::DELETE;
  Simulator::Instance()->SendMessage( *this->visualMsg );
  this->visualMsg->action = VisualMsg::UPDATE;

  Events::moveModeSignal(true);
  this->state = 0;
}

bool SphereMaker::IsActive() const
{
  return this->state > 0;
}

void SphereMaker::MousePushCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->mousePushPos = event.pressPos;
}

void SphereMaker::MouseReleaseCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->state++;

  if (this->state == 2)
  {
    this->CreateTheEntity();
    this->Stop();
  }
}

void SphereMaker::MouseDragCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  Vector3 norm;
  Vector3 p1, p2;

  norm.Set(0,0,1);

  p1 = event.camera->GetWorldPointOnPlane(this->mousePushPos.x, this->mousePushPos.y, norm, 0);
  p1 = this->GetSnappedPoint( p1 );

  p2 = event.camera->GetWorldPointOnPlane(event.pos.x, event.pos.y ,norm, 0);
  p2 = this->GetSnappedPoint( p2 );

  this->visualMsg->pose.pos = p1;

  double scale = p1.Distance(p2);
  Vector3 p = this->visualMsg->pose.pos;

  p.z = scale;

  this->visualMsg->pose.pos = p;
  this->visualMsg->size.Set(scale,scale,scale);
  Simulator::Instance()->SendMessage(*this->visualMsg);
}

void SphereMaker::CreateTheEntity()
{
  InsertModelMsg msg;
  std::ostringstream newModelStr;

  newModelStr << "<?xml version='1.0'?>";

  newModelStr << "<model type='physical' name='" << this->visualMsg->id << "'>\
    <xyz>" << this->visualMsg->pose.pos << "</xyz>\
    <body name='body'>\
    <geom type='sphere' name='geom'>\
    <size>" << this->visualMsg->size.x << "</size>\
    <mass>0.5</mass>\
    <visual>\
    <mesh>unit_sphere</mesh>\
    <size>" << this->visualMsg->size*2 << "</size>\
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
