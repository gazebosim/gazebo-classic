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

#include "gui/SphereMaker.hh"

using namespace gazebo;
using namespace gui;

unsigned int SphereMaker::counter = 0;

SphereMaker::SphereMaker() 
  : EntityMaker()
{
  this->state = 0;
  this->visualMsg = new msgs::Visual();
  this->visualMsg->set_render_type( msgs::Visual::MESH_RESOURCE );
  this->visualMsg->set_mesh( "unit_sphere" );
  this->visualMsg->set_material( "Gazebo/TurquoiseGlowOutline" );
}

SphereMaker::~SphereMaker()
{
  delete this->visualMsg;
}

void SphereMaker::Start()
{
  std::ostringstream stream;
  stream << "user_sphere_" << counter++;
  this->visualMsg->mutable_header()->set_str_id( stream.str() );

  this->state = 1;
}

void SphereMaker::Stop()
{
  this->visualMsg->set_action( msgs::Visual::DELETE );
  //Simulator::Instance()->SendMessage( *this->visualMsg );
  this->visualMsg->set_action( msgs::Visual::UPDATE );

  event::Events::moveModeSignal(true);
  this->state = 0;
}

bool SphereMaker::IsActive() const
{
  return this->state > 0;
}

void SphereMaker::MousePushCB(const common::MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->mousePushPos = event.pressPos;
}

void SphereMaker::MouseReleaseCB(const common::MouseEvent &event)
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

void SphereMaker::MouseDragCB(const common::MouseEvent &event)
{
  if (this->state == 0)
    return;

  common::Vector3 norm;
  common::Vector3 p1, p2;

  norm.Set(0,0,1);

  /* NATY: Fix camera issue
  p1 = event.camera->GetWorldPointOnPlane(this->mousePushPos.x, this->mousePushPos.y, norm, 0);
  p1 = this->GetSnappedPoint( p1 );

  p2 = event.camera->GetWorldPointOnPlane(event.pos.x, event.pos.y ,norm, 0);
  p2 = this->GetSnappedPoint( p2 );

  common::Message::Set(this->visualMsg->mutable_pose()->mutable_position(), p1);

  double scale = p1.Distance(p2);
  common::Vector3 p( this->visualMsg->pose().position().x(),
             this->visualMsg->pose().position().y(),
             this->visualMsg->pose().position().z() );

  p.z = scale;

  common::Message::Set(this->visualMsg->mutable_pose()->mutable_position(), p);
  common::Message::Set(this->visualMsg->mutable_scale(),common::Vector3(scale,scale,scale));
  */
}

void SphereMaker::CreateTheEntity()
{
  msgs::InsertModel msg;
  common::Message::Init(msg, "new_sphere");
  std::ostringstream newModelStr;

  newModelStr << "<?xml version='1.0'?>";

  newModelStr << "<model type='physical' name='" << this->visualMsg->header().str_id() << "'>\
    <xyz>" << this->visualMsg->pose().position().x() << " "
           << this->visualMsg->pose().position().y() << " "
           << this->visualMsg->pose().position().z() << "</xyz>\
    <body name='body'>\
    <geom type='sphere' name='geom'>\
    <size>" << this->visualMsg->scale().x() << " "
            << this->visualMsg->scale().y() << " "
            << this->visualMsg->scale().z() << "</size>\
    <mass>0.5</mass>\
    <visual>\
    <mesh>unit_sphere</mesh>\
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
}
