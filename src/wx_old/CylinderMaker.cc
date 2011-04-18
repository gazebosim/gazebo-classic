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

#include "common/Messages.hh"
#include "common/Events.hh"
#include "common/MouseEvent.hh"

#include "gui/CylinderMaker.hh"

using namespace gazebo;
using namespace gui;

unsigned int CylinderMaker::counter = 0;

CylinderMaker::CylinderMaker()
  : EntityMaker()
{
  this->state = 0;
  this->visualMsg = new msgs::Visual();
  this->visualMsg->set_render_type( msgs::Visual::MESH_RESOURCE );
  this->visualMsg->set_mesh( "unit_cylinder" );
  this->visualMsg->set_material( "Gazebo/TurquoiseGlowOutline" );
}

CylinderMaker::~CylinderMaker()
{
  delete this->visualMsg;
}

void CylinderMaker::Start()
{
  std::ostringstream stream;
  stream <<  "user_cylinder_" << counter++;
  this->visualMsg->mutable_header()->set_str_id( stream.str() );
  this->state = 1;
}

void CylinderMaker::Stop()
{
  this->visualMsg->set_action( msgs::Visual::DELETE );
  //Simulator::Instance()->SendMessage( *this->visualMsg );
  this->visualMsg->set_action( msgs::Visual::UPDATE );

  this->state = 0;
  event::Events::moveModeSignal(true);
}

bool CylinderMaker::IsActive() const
{
  return this->state > 0;
}

void CylinderMaker::MousePushCB(const common::MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->mousePushPos = event.pressPos;
}

void CylinderMaker::MouseReleaseCB(const common::MouseEvent &event)
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

void CylinderMaker::MouseDragCB(const common::MouseEvent &event)
{
  if (this->state == 0)
    return;

  common::Vector3 norm;
  common::Vector3 p1, p2;

  if (this->state == 1)
    norm.Set(0,0,1);
  else if (this->state == 2)
    norm.Set(1,0,0);

  /* NATY: Fix camera issue
  p1 = event.camera->GetWorldPointOnPlane(this->mousePushPos.x, this->mousePushPos.y, norm, 0);
  p1 = this->GetSnappedPoint( p1 );

  p2 = event.camera->GetWorldPointOnPlane(event.pos.x, event.pos.y ,norm, 0);
  p2 = this->GetSnappedPoint( p2 );

  if (this->state == 1)
    common::Message::Set(this->visualMsg->mutable_pose()->mutable_position(), p1 );

  common::Vector3 p( this->visualMsg->pose().position().x(),
             this->visualMsg->pose().position().y(),
             this->visualMsg->pose().position().z() );

  common::Vector3 scale;

  if (this->state == 1)
  {
    double dist = p1.Distance(p2);
    scale.x = dist*2;
    scale.y = dist*2;
    scale.z = 0.01;
  }
  else
  {
    scale.Set( this->visualMsg->scale().x(),
               this->visualMsg->scale().y(),
               this->visualMsg->scale().z() );
    scale.z = (this->mousePushPos.y - event.pos.y)*0.01;
    p.z = scale.z/2.0;
  }

  common::Message::Set(this->visualMsg->mutable_pose()->mutable_position(), p );
  common::Message::Set(this->visualMsg->mutable_scale(), scale );
  //Simulator::Instance()->SendMessage(*this->visualMsg);
*/
}

void CylinderMaker::CreateTheEntity()
{
  msgs::InsertModel msg;
  common::Message::Init(msg,"new cylinder");
  std::ostringstream newModelStr;

  newModelStr << "<?xml version='1.0'?>";

  newModelStr << "<model type='physical' name='" << this->visualMsg->header().str_id() << "'>\
    <xyz>" << this->visualMsg->pose().position().x() << " " 
           << this->visualMsg->pose().position().y() << " " 
           << this->visualMsg->pose().position().z() << "</xyz>\
    <body name='body'>\
    <geom type='cylinder' name='geom'>\
    <size>" << this->visualMsg->scale().x()*.5 << " " 
            << this->visualMsg->scale().z() << "</size>\
    <mass>0.5</mass>\
    <visual>\
    <mesh>unit_cylinder</mesh>\
    <scale>" << this->visualMsg->scale().x() << " "
             << this->visualMsg->scale().y() << " "
             << this->visualMsg->scale().z() << "</scale>\
    <material>Gazebo/Grey</material>\
    <shader>pixel</shader>\
    </visual>\
    </geom>\
    </body>\
    </model>";

  msg.set_xml( newModelStr.str() );

  this->visualMsg->set_action( msgs::Visual::DELETE );
  common::Message::Stamp(this->visualMsg->mutable_header());
  //Simulator::Instance()->SendMessage( *this->visualMsg );
  //Simulator::Instance()->SendMessage( msg );
}

