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
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Light.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/LightMakerPrivate.hh"
#include "gazebo/gui/LightMaker.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LightMaker::LightMaker() : EntityMaker(*new LightMakerPrivate)
{
  LightMakerPrivate *dPtr =
      reinterpret_cast<LightMakerPrivate *>(this->dataPtr);

  dPtr->lightPub = dPtr->node->Advertise<msgs::Light>("~/light");

  msgs::Set(dPtr->msg.mutable_diffuse(), common::Color(0.5, 0.5, 0.5, 1));
  msgs::Set(dPtr->msg.mutable_specular(), common::Color(0.1, 0.1, 0.1, 1));

  dPtr->msg.set_attenuation_constant(0.5);
  dPtr->msg.set_attenuation_linear(0.01);
  dPtr->msg.set_attenuation_quadratic(0.001);
  dPtr->msg.set_range(20);
}

/////////////////////////////////////////////////
bool LightMaker::InitFromLight(const std::string &_lightName)
{
  LightMakerPrivate *dPtr =
      reinterpret_cast<LightMakerPrivate *>(this->dataPtr);

  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  if (dPtr->light)
  {
    scene->RemoveLight(dPtr->light);
    dPtr->light.reset();
  }

  rendering::LightPtr sceneLight = scene->GetLight(_lightName);
  if (!sceneLight)
  {
    gzerr << "Light: '" << _lightName << "' does not exist." << std::endl;
    return false;
  }

  dPtr->light = sceneLight->Clone(_lightName + "_clone_tmp", scene);

  if (!dPtr->light)
  {
    gzerr << "Unable to clone\n";
    return false;
  }

  dPtr->lightTypename =  dPtr->light->GetType();
  dPtr->light->FillMsg(dPtr->msg);

  std::string newName = _lightName + "_clone";
  int i = 0;
  while (scene->GetLight(newName))
  {
    newName = _lightName + "_clone_" +
      boost::lexical_cast<std::string>(i);
    i++;
  }

  dPtr->msg.set_name(newName);

  return true;
}

/////////////////////////////////////////////////
bool LightMaker::Init()
{
  LightMakerPrivate *dPtr =
      reinterpret_cast<LightMakerPrivate *>(this->dataPtr);

  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  dPtr->light.reset(new rendering::Light(scene));
  dPtr->light->Load();
  scene->AddLight(dPtr->light);

  dPtr->light->SetLightType(dPtr->lightTypename);
  dPtr->light->SetPosition(math::Vector3(0, 0, 1));
  if (dPtr->lightTypename == "directional")
    dPtr->light->SetDirection(math::Vector3(.1, .1, -0.9));

  // Unique name
  int counter = 0;
  std::ostringstream lightName;
  lightName << "user_" << dPtr->lightTypename << "_light_" << counter;
  while (scene->GetLight(lightName.str()))
  {
    lightName.str("");
    lightName << "user_" << dPtr->lightTypename << "_light_" << counter;
    counter++;
  }
  dPtr->msg.set_name(lightName.str());

  return true;
}

/////////////////////////////////////////////////
void LightMaker::Start()
{
  LightMakerPrivate *dPtr =
      reinterpret_cast<LightMakerPrivate *>(this->dataPtr);

  EntityMaker::Start();

  if (!dPtr->light)
    this->Init();
}

/////////////////////////////////////////////////
void LightMaker::Stop()
{
  LightMakerPrivate *dPtr =
      reinterpret_cast<LightMakerPrivate *>(this->dataPtr);

  if (dPtr->light)
  {
    rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
    scene->RemoveLight(dPtr->light);
    dPtr->light.reset();
  }
  EntityMaker::Stop();
}

/////////////////////////////////////////////////
void LightMaker::CreateTheEntity()
{
  LightMakerPrivate *dPtr =
      reinterpret_cast<LightMakerPrivate *>(this->dataPtr);

  msgs::Set(dPtr->msg.mutable_pose()->mutable_position(),
            dPtr->light->GetPosition().Ign());
  msgs::Set(dPtr->msg.mutable_pose()->mutable_orientation(),
            ignition::math::Quaterniond());
  dPtr->lightPub->Publish(dPtr->msg);
}

/////////////////////////////////////////////////
ignition::math::Vector3d LightMaker::EntityPosition() const
{
  LightMakerPrivate *dPtr =
      reinterpret_cast<LightMakerPrivate *>(this->dataPtr);

  return dPtr->light->GetPosition().Ign();
}

/////////////////////////////////////////////////
void LightMaker::SetEntityPosition(const ignition::math::Vector3d &_pos)
{
  LightMakerPrivate *dPtr =
      reinterpret_cast<LightMakerPrivate *>(this->dataPtr);

  dPtr->light->SetPosition(_pos);
}

/////////////////////////////////////////////////
PointLightMaker::PointLightMaker() : LightMaker()
{
  LightMakerPrivate *dPtr =
      reinterpret_cast<LightMakerPrivate *>(this->dataPtr);

  dPtr->msg.set_type(msgs::Light::POINT);
  dPtr->msg.set_cast_shadows(false);
  dPtr->lightTypename = "point";
}

/////////////////////////////////////////////////
SpotLightMaker::SpotLightMaker() : LightMaker()
{
  LightMakerPrivate *dPtr =
      reinterpret_cast<LightMakerPrivate *>(this->dataPtr);

  dPtr->msg.set_type(msgs::Light::SPOT);
  msgs::Set(dPtr->msg.mutable_direction(),
            ignition::math::Vector3d(0, 0, -1));
  dPtr->msg.set_cast_shadows(false);

  dPtr->msg.set_spot_inner_angle(0.6);
  dPtr->msg.set_spot_outer_angle(1.0);
  dPtr->msg.set_spot_falloff(1.0);
  dPtr->lightTypename  = "spot";
}

/////////////////////////////////////////////////
DirectionalLightMaker::DirectionalLightMaker() : LightMaker()
{
  LightMakerPrivate *dPtr =
      reinterpret_cast<LightMakerPrivate *>(this->dataPtr);

  dPtr->msg.set_type(msgs::Light::DIRECTIONAL);
  msgs::Set(dPtr->msg.mutable_direction(),
            ignition::math::Vector3d(.1, .1, -0.9));
  dPtr->msg.set_cast_shadows(true);

  dPtr->lightTypename  = "directional";
}

