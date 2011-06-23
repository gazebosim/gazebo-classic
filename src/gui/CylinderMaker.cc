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
#include <sstream>

#include "common/Messages.hh"
#include "common/Events.hh"
#include "common/MouseEvent.hh"
#include "math/Quaternion.hh"

#include "rendering/UserCamera.hh"

#include "transport/Publisher.hh"

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

  common::Message::Set(this->visualMsg->mutable_pose()->mutable_orientation(), math::Quaternion());
}

CylinderMaker::~CylinderMaker()
{
  delete this->visualMsg;
}

void CylinderMaker::Start(const rendering::UserCameraPtr camera,
                          const CreateCallback &cb)
{
  this->createCB = cb;
  this->camera = camera;
  std::ostringstream stream;
  stream <<  "user_cylinder_" << counter++;
  this->visualMsg->mutable_header()->set_str_id( stream.str() );
  this->state = 1;
}

void CylinderMaker::Stop()
{
  this->visualMsg->set_action( msgs::Visual::DELETE );
  this->visPub->Publish(*this->visualMsg);
  this->visualMsg->set_action( msgs::Visual::UPDATE );

  this->state = 0;
  event::Events::moveModeSignal(true);
}

bool CylinderMaker::IsActive() const
{
  return this->state > 0;
}

void CylinderMaker::OnMousePush(const common::MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->mousePushPos = event.pressPos;
}

void CylinderMaker::OnMouseRelease(const common::MouseEvent &/*_event*/)
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

void CylinderMaker::OnMouseDrag(const common::MouseEvent &event)
{
  if (this->state == 0)
    return;

  math::Vector3 norm;
  math::Vector3 p1, p2;

  if (this->state == 1)
    norm.Set(0,0,1);
  else if (this->state == 2)
    norm.Set(1,0,0);

  p1 = this->camera->GetWorldPointOnPlane(this->mousePushPos.x, 
                                          this->mousePushPos.y, norm, 0);
  p1 = this->GetSnappedPoint( p1 );

  p2 = this->camera->GetWorldPointOnPlane(event.pos.x, event.pos.y ,norm, 0);
  p2 = this->GetSnappedPoint( p2 );

  if (this->state == 1)
    common::Message::Set(this->visualMsg->mutable_pose()->mutable_position(), p1 );

  math::Vector3 p( this->visualMsg->pose().position().x(),
             this->visualMsg->pose().position().y(),
             this->visualMsg->pose().position().z() );

  math::Vector3 scale;

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

  this->visPub->Publish(*this->visualMsg);
}

void CylinderMaker::CreateTheEntity()
{
  msgs::Factory msg;
  common::Message::Init(msg,"new cylinder");
  std::ostringstream newModelStr;

  newModelStr << "<?xml version='1.0'?>";

  newModelStr << "<model name='" << this->visualMsg->header().str_id() << "_model'>\
    <static>false</static>\
    <origin xyz='" << this->visualMsg->pose().position().x() << " " 
                   << this->visualMsg->pose().position().y() << " " 
                   << this->visualMsg->pose().position().z() << "'/>\
    <link name='body'>\
      <collision name='geom'>\
        <geometry>\
          <cylinder radius='" << this->visualMsg->scale().x()*.5 << "'\
                    length='" << this->visualMsg->scale().z() << "'/>\
        </geometry>\
        <mass>0.5</mass>\
      </collision>\
      <visual>\
        <geometry>\
          <cylinder radius='" << this->visualMsg->scale().x()*.5 << "'\
                    length='" << this->visualMsg->scale().z() << "'/>\
        </geometry>\
        <material name='Gazebo/Grey'/>\
        <shader>pixel</shader>\
        <cast_shadows>pixel</cast_shadows>\
      </visual>\
    </link>\
  </model>";

  msg.set_xml( newModelStr.str() );

  common::Message::Stamp(this->visualMsg->mutable_header());
  this->visualMsg->set_action( msgs::Visual::DELETE );
  this->visPub->Publish(*this->visualMsg);

  (this->createCB)(
      common::Message::Convert(this->visualMsg->pose().position()), 
      common::Message::Convert(this->visualMsg->scale()) );

  //this->makerPub->Publish(msg);
}

