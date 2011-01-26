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
#include "World.hh"
#include "CylinderMaker.hh"

using namespace gazebo;
unsigned int CylinderMaker::counter = 0;

CylinderMaker::CylinderMaker()
  : EntityMaker()
{
  this->state = 0;
  this->visualMsg = new VisualMsg();
  this->visualMsg->render = RENDERING_MESH_RESOURCE;
  this->visualMsg->mesh = "unit_cylinder";
  this->visualMsg->material = "Gazebo/TurquoiseGlowOutline";
}

CylinderMaker::~CylinderMaker()
{
  delete this->visualMsg;
}

void CylinderMaker::Start(Scene *scene)
{
  std::ostringstream stream;
  stream <<  "user_cylinder_" << counter++;
  this->visualMsg->id =  stream.str();
  this->state = 1;
}

void CylinderMaker::Stop()
{
  this->visualMsg->action = VisualMsg::DELETE;
  Simulator::Instance()->SendMessage( *this->visualMsg );
  this->visualMsg->action = VisualMsg::UPDATE;

  this->state = 0;
  Events::moveModeSignal(true);
}

bool CylinderMaker::IsActive() const
{
  return this->state > 0;
}

void CylinderMaker::MousePushCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->mousePushPos = event.pressPos;
}

void CylinderMaker::MouseReleaseCB(const MouseEvent &event)
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

void CylinderMaker::MouseDragCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  Vector3 norm;
  Vector3 p1, p2;

  if (this->state == 1)
    norm.Set(0,0,1);
  else if (this->state == 2)
    norm.Set(1,0,0);

  p1 = event.camera->GetWorldPointOnPlane(this->mousePushPos.x, this->mousePushPos.y, norm, 0);
  p1 = this->GetSnappedPoint( p1 );

  p2 = event.camera->GetWorldPointOnPlane(event.pos.x, event.pos.y ,norm, 0);
  p2 = this->GetSnappedPoint( p2 );

  if (this->state == 1)
    this->visualMsg->pose.pos = p1;

  Vector3 p = this->visualMsg->pose.pos;
  Vector3 scale;

  if (this->state == 1)
  {
    double dist = p1.Distance(p2);
    scale.x = dist*2;
    scale.y = dist*2;
    scale.z = 0.01;
  }
  else
  {
    scale = this->visualMsg->size;
    scale.z = (this->mousePushPos.y - event.pos.y)*0.01;
    p.z = scale.z/2.0;
  }

  this->visualMsg->pose.pos = p;
  this->visualMsg->size = scale;
  Simulator::Instance()->SendMessage(*this->visualMsg);
}

void CylinderMaker::CreateTheEntity()
{
  InsertModelMsg msg;
  std::ostringstream newModelStr;

  newModelStr << "<?xml version='1.0'?>";

  newModelStr << "<model type='physical' name='" << this->visualMsg->id << "'>\
    <xyz>" << this->visualMsg->pose.pos << "</xyz>\
    <body name='body'>\
    <geom type='cylinder' name='geom'>\
    <size>" << this->visualMsg->size.x*.5 << " " << this->visualMsg->size.z << "</size>\
    <mass>0.5</mass>\
    <visual>\
    <mesh>unit_cylinder</mesh>\
    <size>" << this->visualMsg->size << "</size>\
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

