/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <iostream>
#include <sstream>

#include "gazebo/transport/Node.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Light.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/LightMaker.hh"

using namespace gazebo;
using namespace gui;

unsigned int LightMaker::counter = 0;

/////////////////////////////////////////////////
LightMaker::LightMaker() : EntityMaker()
{
  this->lightPub = this->node->Advertise<msgs::Light>("~/light");

  msgs::Set(this->msg.mutable_diffuse(), common::Color(0.5, 0.5, 0.5, 1));
  msgs::Set(this->msg.mutable_specular(), common::Color(0.1, 0.1, 0.1, 1));

  this->msg.set_attenuation_constant(0.5);
  this->msg.set_attenuation_linear(0.01);
  this->msg.set_attenuation_quadratic(0.001);
  this->msg.set_range(20);
}

/////////////////////////////////////////////////
bool LightMaker::InitFromLight(const std::string & _lightName)
{
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  if (this->light)
  {
    scene->RemoveLight(this->light);
    this->light.reset();
  }

  rendering::LightPtr sceneLight = scene->GetLight(_lightName);
  if (!sceneLight)
  {
    gzerr << "Light: '" << _lightName << "' does not exist." << std::endl;
    return false;
  }

  this->light = sceneLight->Clone(_lightName + "_clone_tmp", scene);

  if (!this->light)
  {
    gzerr << "Unable to clone\n";
    return false;
  }

  this->lightTypename =  this->light->GetType();
  this->light->FillMsg(this->msg);

  std::string newName = _lightName + "_clone";
  int i = 0;
  while (scene->GetLight(newName))
  {
    newName = _lightName + "_clone_" +
      boost::lexical_cast<std::string>(i);
    i++;
  }

  this->msg.set_name(newName);

  return true;
}

/////////////////////////////////////////////////
bool LightMaker::Init()
{
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  this->light.reset(new rendering::Light(scene));
  this->light->Load();
  scene->AddLight(this->light);

  this->light->SetLightType(this->lightTypename);
  this->light->SetPosition(math::Vector3(0, 0, 1));
  if (this->lightTypename == "directional")
    this->light->SetDirection(math::Vector3(.1, .1, -0.9));

  std::ostringstream stream;
  stream << "user_" << this->lightTypename << "_light_" << counter++;
  this->msg.set_name(stream.str());

  return true;
}

/////////////////////////////////////////////////
void LightMaker::Start(const rendering::UserCameraPtr _camera)
{
  if (!this->light)
    this->Init();

  this->camera = _camera;
}

/////////////////////////////////////////////////
void LightMaker::Stop()
{
  if (this->light)
  {
    rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
    scene->RemoveLight(this->light);
    this->light.reset();
  }
  gui::Events::moveMode(true);
}

/////////////////////////////////////////////////
void LightMaker::CreateTheEntity()
{
  msgs::Set(this->msg.mutable_pose()->mutable_position(),
            this->light->GetPosition());
  msgs::Set(this->msg.mutable_pose()->mutable_orientation(),
            math::Quaternion());
  this->lightPub->Publish(this->msg);
  this->camera.reset();
}

/////////////////////////////////////////////////
ignition::math::Vector3d LightMaker::EntityPosition() const
{
  return this->light->GetPosition().Ign();
}

/////////////////////////////////////////////////
void LightMaker::SetEntityPosition(const ignition::math::Vector3d &_pos)
{
  this->light->SetPosition(_pos);
}

