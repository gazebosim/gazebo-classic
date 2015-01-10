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
#include <sstream>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/common/Console.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/common/MouseEvent.hh"

#include "gazebo/rendering/UserCamera.hh"

#include "gazebo/transport/Publisher.hh"

#include "gazebo/gui/BoxMaker.hh"

using namespace gazebo;
using namespace gui;

unsigned int BoxMaker::counter = 0;

BoxMaker::BoxMaker()
: EntityMaker()
{
  this->state = 0;
  this->leftMousePressed = false;
  this->visualMsg = new msgs::Visual();
  this->visualMsg->mutable_geometry()->set_type(msgs::Geometry::BOX);
  this->visualMsg->mutable_material()->mutable_script()->add_uri(
      "gazebo://media/materials/scripts/gazebo.material");
  this->visualMsg->mutable_material()->mutable_script()->set_name(
      "Gazebo/TurquoiseGlowOutline");
  msgs::Set(this->visualMsg->mutable_pose()->mutable_orientation(),
            math::Quaternion());
}

BoxMaker::~BoxMaker()
{
  this->camera.reset();
  delete this->visualMsg;
}

void BoxMaker::Start(const rendering::UserCameraPtr _camera)
{
  this->camera = _camera;

  std::ostringstream stream;
  stream << "__GZ_USER_box_" << counter++;
  this->visualMsg->set_name(stream.str());

  this->state = 1;
}

void BoxMaker::Stop()
{
  msgs::Request *msg = msgs::CreateRequest("entity_delete",
                                           this->visualMsg->name());

  this->requestPub->Publish(*msg);
  delete msg;

  this->state = 0;
  gui::Events::moveMode(true);
}

bool BoxMaker::IsActive() const
{
  return this->state > 0;
}

void BoxMaker::OnMousePush(const common::MouseEvent &_event)
{
  if (this->state == 0)
    return;

  this->mousePushPos = _event.pressPos;
}

/////////////////////////////////////////////////
void BoxMaker::OnMouseRelease(const common::MouseEvent &_event)
{
  if (this->state == 0)
    return;

  this->state++;

  this->mouseReleasePos = _event.pos;

  if (this->state == 3)
  {
    this->CreateTheEntity();
    this->Stop();
  }
}

/////////////////////////////////////////////////
void BoxMaker::OnMouseMove(const common::MouseEvent &_event)
{
  if (this->state != 2)
    return;

  math::Vector3 p = msgs::Convert(this->visualMsg->pose().position());
  math::Vector3 scale = msgs::Convert(this->visualMsg->geometry().box().size());

  scale.z = (this->mouseReleasePos.y - _event.pos.y)*0.01;
  if (!_event.shift)
    scale.z = rint(scale.z);
  p.z = scale.z/2.0;

  msgs::Set(this->visualMsg->mutable_pose()->mutable_position(), p);
  msgs::Set(this->visualMsg->mutable_geometry()->mutable_box()->mutable_size(),
      scale);

  this->visPub->Publish(*this->visualMsg);
}

/////////////////////////////////////////////////
void BoxMaker::OnMouseDrag(const common::MouseEvent &_event)
{
  if (this->state != 1)
    return;

  math::Vector3 norm(0, 0, 1);
  math::Vector3 p1, p2;

  if (!this->camera->GetWorldPointOnPlane(this->mousePushPos.x,
                                          this->mousePushPos.y,
                                          math::Plane(norm), p1))
  {
    gzerr << "Invalid mouse point\n";
    return;
  }

  p1 = this->GetSnappedPoint(p1);

  if (!this->camera->GetWorldPointOnPlane(
        _event.pos.x, _event.pos.y , math::Plane(norm), p2))
  {
    gzerr << "Invalid mouse point\n";
    return;
  }

  p2 = this->GetSnappedPoint(p2);

  msgs::Set(this->visualMsg->mutable_pose()->mutable_position(), p1);

  math::Vector3 scale = p1-p2;
  math::Vector3 p = msgs::Convert(this->visualMsg->pose().position());

  scale.z = 0.01;
  p.x = p1.x - scale.x/2.0;
  p.y = p1.y - scale.y/2.0;


  msgs::Set(this->visualMsg->mutable_pose()->mutable_position(), p);
  msgs::Set(this->visualMsg->mutable_geometry()->mutable_box()->mutable_size(),
      scale.GetAbs());

  this->visPub->Publish(*this->visualMsg);
}

/////////////////////////////////////////////////
std::string BoxMaker::GetSDFString()
{
  msgs::Model model;
  model.set_name("unit_box_" + std::string(counter));
  msgs::Set(model.mutable_pose(), math::Pose(0, 0, 0.5, 0, 0, 0));
  model.AddBoxLink(1.0, math::Vector3::One);
  msgs::Link *link = model.mutable_link(0);
  msgs::Visual *visual = link->mutable_visual(0);
  msgs::Material::Script *script = visual->mutable_material()->mutable_script();
  script->set_name("Gazebo/Grey");
  script->set_uri("<uri>file://media/materials/scripts/gazebo.material</uri>");

  return msgs::ModelToSDF(model)->ToString("");
}

/////////////////////////////////////////////////
void BoxMaker::CreateTheEntity()
{
  msgs::Factory msg;

  math::Vector3 p = msgs::Convert(this->visualMsg->pose().position());
  math::Vector3 size = msgs::Convert(this->visualMsg->geometry().box().size());

  msg.set_sdf(this->GetSDFString());

  msgs::Request *requestMsg = msgs::CreateRequest("entity_delete",
      this->visualMsg->name());
  this->requestPub->Publish(*requestMsg);
  delete requestMsg;

  this->makerPub->Publish(msg);
  this->camera.reset();
}
