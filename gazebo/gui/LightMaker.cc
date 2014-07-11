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

#include <iostream>
#include <sstream>

#include "gazebo/transport/Node.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/common/MouseEvent.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Light.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/gui/LightMaker.hh"

using namespace gazebo;
using namespace gui;

unsigned int LightMaker::counter = 0;

/////////////////////////////////////////////////
LightMaker::LightMaker() : EntityMaker()
{
  this->lightPub = this->node->Advertise<msgs::Light>("~/light");

  this->state = 0;

  msgs::Set(this->msg.mutable_diffuse(), common::Color(0.5, 0.5, 0.5, 1));
  msgs::Set(this->msg.mutable_specular(), common::Color(0.1, 0.1, 0.1, 1));

  this->msg.set_attenuation_constant(0.5);
  this->msg.set_attenuation_linear(0.01);
  this->msg.set_attenuation_quadratic(0.001);
  this->msg.set_range(20);
}

/////////////////////////////////////////////////
void LightMaker::Start(const rendering::UserCameraPtr _camera)
{
  this->camera = _camera;

  this->light = new rendering::Light(this->camera->GetScene());
  this->light->Load();

  this->light->SetLightType(this->lightTypename);
  this->light->SetPosition(ignition::math::Vector3d(0, 0, 1));
  if (this->lightTypename == "directional")
    this->light->SetDirection(ignition::math::Vector3d(.1, .1, -0.9));

  std::ostringstream stream;
  stream << "user_" << this->lightTypename << "_light_" << counter++;
  this->msg.set_name(stream.str());
  this->state = 1;
}

/////////////////////////////////////////////////
void LightMaker::Stop()
{
  delete this->light;
  this->light = NULL;

  this->state = 0;
  gui::Events::moveMode(true);
}

/////////////////////////////////////////////////
bool LightMaker::IsActive() const
{
  return this->state > 0;
}

/////////////////////////////////////////////////
void LightMaker::OnMousePush(const common::MouseEvent &/*_event*/)
{
}

/////////////////////////////////////////////////
void LightMaker::CreateTheEntity()
{
  msgs::Set(this->msg.mutable_pose()->mutable_position(),
            this->light->GetPosition());
  msgs::Set(this->msg.mutable_pose()->mutable_orientation(),
            ignition::math::Quaterniond());
  this->lightPub->Publish(this->msg);
  this->camera.reset();
}

/////////////////////////////////////////////////
void LightMaker::OnMouseRelease(const common::MouseEvent &_event)
{
  if (_event.button == common::MouseEvent::LEFT && !_event.dragging)
  {
    this->CreateTheEntity();
    this->Stop();
  }
}

/////////////////////////////////////////////////
// \TODO: This was copied from ModelMaker. Figure out a better way to
// prevent code duplication.
void LightMaker::OnMouseMove(const common::MouseEvent &_event)
{
  ignition::math::Vector3d origin1, dir1, p1;

  // Cast two rays from the camera into the world
  this->camera->GetCameraToViewportRay(_event.pos.X(), _event.pos.Y(),
                                       origin1, dir1);

  // Compute the distance from the camera to plane of translation
  ignition::math::Planed plane(ignition::math::Vector3d(0, 0, 1), 0);

  double dist1 = plane.Distance(origin1, dir1);

  // Compute two points on the plane. The first point is the current
  // mouse position, the second is the previous mouse position
  p1 = origin1 + dir1 * dist1;

  if (!_event.shift)
  {
    if (ceil(p1.X()) - p1.X() <= .4)
      p1.X() = ceil(p1.X());
    else if (p1.X() - floor(p1.X()) <= .4)
      p1.X() = floor(p1.X());

    if (ceil(p1.Y()) - p1.Y() <= .4)
      p1.Y() = ceil(p1.Y());
    else if (p1.Y() - floor(p1.Y()) <= .4)
      p1.Y() = floor(p1.Y());
  }
  p1.Z() = this->light->GetPosition().Z();

  this->light->SetPosition(p1);
}
