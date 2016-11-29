/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Light.hh"
#include "gazebo/rendering/LightPrivate.hh"

using namespace gazebo;
using namespace rendering;

unsigned int LightPrivate::lightCounter = 0;

//////////////////////////////////////////////////
Light::Light(ScenePtr _scene)
  : dataPtr(new LightPrivate)
{
  this->dataPtr->line = NULL;
  this->dataPtr->scene = _scene;

  this->dataPtr->lightCounter++;

  this->dataPtr->sdf.reset(new sdf::Element);
  sdf::initFile("light.sdf", this->dataPtr->sdf);
}

//////////////////////////////////////////////////
Light::~Light()
{
  if (this->dataPtr->light)
  {
    this->dataPtr->scene->OgreSceneManager()->destroyLight(this->Name());
  }

  this->dataPtr->scene->OgreSceneManager()->destroyEntity(
      this->Name() + "_selection_sphere");

  if (this->dataPtr->visual)
  {
    this->dataPtr->visual->DeleteDynamicLine(this->dataPtr->line);
    this->dataPtr->scene->RemoveVisual(this->dataPtr->visual);
    this->dataPtr->visual.reset();
  }

  this->dataPtr->sdf->Reset();
  this->dataPtr->sdf.reset();

  this->dataPtr->scene.reset();
}

//////////////////////////////////////////////////
void Light::Load(sdf::ElementPtr _sdf)
{
  this->dataPtr->sdf->Copy(_sdf);
  this->Load();
  this->dataPtr->scene->AddLight(shared_from_this());
}

//////////////////////////////////////////////////
void Light::Load()
{
  math::Vector3 vec;

  try
  {
    this->dataPtr->light =
        this->dataPtr->scene->OgreSceneManager()->createLight(this->Name());
  }
  catch(Ogre::Exception &e)
  {
    gzthrow("Ogre Error:" << e.getFullDescription() << "\n" << \
        "Unable to create a light");
  }

  this->Update();

  this->dataPtr->visual.reset(new Visual(this->Name(),
                     this->dataPtr->scene->WorldVisual()));
  this->dataPtr->visual->Load();
  this->dataPtr->visual->AttachObject(this->dataPtr->light);

  this->CreateVisual();
}

//////////////////////////////////////////////////
void Light::Update()
{
  // shadow support is also affected by light type so set type first.
  this->SetLightType(this->dataPtr->sdf->Get<std::string>("type"));
  this->SetCastShadows(this->dataPtr->sdf->Get<bool>("cast_shadows"));

  this->SetDiffuseColor(
      this->dataPtr->sdf->GetElement("diffuse")->Get<common::Color>());
  this->SetSpecularColor(
      this->dataPtr->sdf->GetElement("specular")->Get<common::Color>());
  this->SetDirection(
      this->dataPtr->sdf->Get<ignition::math::Vector3d>("direction"));

  if (this->dataPtr->sdf->HasElement("attenuation"))
  {
    sdf::ElementPtr elem = this->dataPtr->sdf->GetElement("attenuation");

    this->SetAttenuation(elem->Get<double>("constant"),
                         elem->Get<double>("linear"),
                         elem->Get<double>("quadratic"));
    this->SetRange(elem->Get<double>("range"));
  }

  if (this->dataPtr->sdf->HasElement("spot"))
  {
    sdf::ElementPtr elem = this->dataPtr->sdf->GetElement("spot");
    this->SetSpotInnerAngle(elem->Get<double>("inner_angle"));
    this->SetSpotOuterAngle(elem->Get<double>("outer_angle"));
    this->SetSpotFalloff(elem->Get<double>("falloff"));
  }
}

//////////////////////////////////////////////////
void Light::UpdateSDFFromMsg(const msgs::Light &_msg)
{
  msgs::LightToSDF(_msg, this->dataPtr->sdf);
}

//////////////////////////////////////////////////
void Light::UpdateFromMsg(ConstLightPtr &_msg)
{
  this->UpdateSDFFromMsg(*_msg);

  this->Update();

  if (_msg->has_pose())
  {
    this->SetPosition(msgs::ConvertIgn(_msg->pose().position()));
    this->SetRotation(msgs::ConvertIgn(_msg->pose().orientation()));
  }
}

//////////////////////////////////////////////////
void Light::LoadFromMsg(const msgs::Light &_msg)
{
  this->UpdateSDFFromMsg(_msg);

  this->Load();

  if (_msg.has_pose())
  {
    this->SetPosition(msgs::ConvertIgn(_msg.pose().position()));
    this->SetRotation(msgs::ConvertIgn(_msg.pose().orientation()));
  }
}

//////////////////////////////////////////////////
void Light::LoadFromMsg(ConstLightPtr &_msg)
{
  this->LoadFromMsg(*_msg);
}

//////////////////////////////////////////////////
void Light::SetName(const std::string &_name)
{
  this->dataPtr->sdf->GetAttribute("name")->Set(_name);
}

//////////////////////////////////////////////////
std::string Light::Name() const
{
  return this->dataPtr->sdf->Get<std::string>("name");
}

//////////////////////////////////////////////////
std::string Light::Type() const
{
  return this->dataPtr->sdf->Get<std::string>("type");
}

//////////////////////////////////////////////////
// The lines draw a visualization of the camera
void Light::CreateVisual()
{
  if (!this->dataPtr->visual)
    return;

  if (this->dataPtr->line)
    this->dataPtr->line->Clear();
  else
  {
    this->dataPtr->line =
        this->dataPtr->visual->CreateDynamicLine(RENDERING_LINE_LIST);

    this->dataPtr->line->setMaterial("Gazebo/LightOn");

    this->dataPtr->line->setVisibilityFlags(GZ_VISIBILITY_GUI);

    this->dataPtr->visual->SetVisible(true);

    // Create a visual to hold the light selection object.
    VisualPtr lightSelectionVis(new Visual(this->Name() + "_seletion",
        this->dataPtr->visual));
    lightSelectionVis->SetType(Visual::VT_GUI);

    // Make sure the unit_sphere has been inserted.
    lightSelectionVis->InsertMesh("unit_sphere");
    lightSelectionVis->AttachMesh("unit_sphere");
    lightSelectionVis->SetMaterial("Gazebo/White");

    // Make sure the selection object is rendered only in the selection
    // buffer.
    lightSelectionVis->SetVisibilityFlags(GZ_VISIBILITY_SELECTION);
    lightSelectionVis->SetCastShadows(false);

    // Scale the selection object to roughly match the light visual size.
    lightSelectionVis->SetScale(ignition::math::Vector3d(0.25, 0.25, 0.25));
  }

  std::string lightType = this->dataPtr->sdf->Get<std::string>("type");

  if (lightType == "directional")
  {
    float s =.5;
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(-s, -s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(-s, s, 0));

    this->dataPtr->line->AddPoint(ignition::math::Vector3d(-s, s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(s, s, 0));

    this->dataPtr->line->AddPoint(ignition::math::Vector3d(s, s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(s, -s, 0));

    this->dataPtr->line->AddPoint(ignition::math::Vector3d(s, -s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(-s, -s, 0));

    this->dataPtr->line->AddPoint(ignition::math::Vector3d(0, 0, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(0, 0, -s));
  }
  if (lightType == "point")
  {
    float s = 0.1;
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(-s, -s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(-s, s, 0));

    this->dataPtr->line->AddPoint(ignition::math::Vector3d(-s, s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(s, s, 0));

    this->dataPtr->line->AddPoint(ignition::math::Vector3d(s, s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(s, -s, 0));

    this->dataPtr->line->AddPoint(ignition::math::Vector3d(s, -s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(-s, -s, 0));


    this->dataPtr->line->AddPoint(ignition::math::Vector3d(-s, -s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(0, 0, s));

    this->dataPtr->line->AddPoint(ignition::math::Vector3d(-s, s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(0, 0, s));

    this->dataPtr->line->AddPoint(ignition::math::Vector3d(s, s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(0, 0, s));

    this->dataPtr->line->AddPoint(ignition::math::Vector3d(s, -s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(0, 0, s));



    this->dataPtr->line->AddPoint(ignition::math::Vector3d(-s, -s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(0, 0, -s));

    this->dataPtr->line->AddPoint(ignition::math::Vector3d(-s, s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(0, 0, -s));

    this->dataPtr->line->AddPoint(ignition::math::Vector3d(s, s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(0, 0, -s));

    this->dataPtr->line->AddPoint(ignition::math::Vector3d(s, -s, 0));
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(0, 0, -s));
  }
  else if (lightType == "spot")
  {
    double innerAngle =
        this->dataPtr->light->getSpotlightInnerAngle().valueRadians();
    double outerAngle =
        this->dataPtr->light->getSpotlightOuterAngle().valueRadians();

    double angles[2];
    double range = 0.2;
    angles[0] = range * tan(outerAngle);
    angles[1] = range * tan(innerAngle);

    unsigned int i = 0;
    this->dataPtr->line->AddPoint(ignition::math::Vector3d(0, 0, 0));
    this->dataPtr->line->AddPoint(
        ignition::math::Vector3d(angles[i], angles[i], -range));

    for (i = 0; i < 2; i++)
    {
      this->dataPtr->line->AddPoint(ignition::math::Vector3d(0, 0, 0));
      this->dataPtr->line->AddPoint(ignition::math::Vector3d(
            angles[i], angles[i], -range));

      this->dataPtr->line->AddPoint(ignition::math::Vector3d(0, 0, 0));
      this->dataPtr->line->AddPoint(ignition::math::Vector3d(
            -angles[i], -angles[i], -range));

      this->dataPtr->line->AddPoint(ignition::math::Vector3d(0, 0, 0));
      this->dataPtr->line->AddPoint(ignition::math::Vector3d(
            angles[i], -angles[i], -range));

      this->dataPtr->line->AddPoint(ignition::math::Vector3d(0, 0, 0));
      this->dataPtr->line->AddPoint(ignition::math::Vector3d(
            -angles[i], angles[i], -range));

      this->dataPtr->line->AddPoint(ignition::math::Vector3d(
            angles[i], angles[i], -range));
      this->dataPtr->line->AddPoint(ignition::math::Vector3d(
            -angles[i], angles[i], -range));

      this->dataPtr->line->AddPoint(ignition::math::Vector3d(
            -angles[i], angles[i], -range));
      this->dataPtr->line->AddPoint(ignition::math::Vector3d(
            -angles[i], -angles[i], -range));

      this->dataPtr->line->AddPoint(ignition::math::Vector3d(
            -angles[i], -angles[i], -range));
      this->dataPtr->line->AddPoint(ignition::math::Vector3d(
            angles[i], -angles[i], -range));

      this->dataPtr->line->AddPoint(ignition::math::Vector3d(
            angles[i], -angles[i], -range));
      this->dataPtr->line->AddPoint(ignition::math::Vector3d(
            angles[i], angles[i], -range));
    }
  }
}

//////////////////////////////////////////////////
void Light::SetPosition(const ignition::math::Vector3d &_p)
{
  this->dataPtr->visual->SetPosition(_p);
}

//////////////////////////////////////////////////
ignition::math::Vector3d Light::Position() const
{
  return this->dataPtr->visual->GetPosition().Ign();
}

//////////////////////////////////////////////////
void Light::SetRotation(const ignition::math::Quaterniond &_q)
{
  this->dataPtr->visual->SetRotation(_q);
}

//////////////////////////////////////////////////
ignition::math::Quaterniond Light::Rotation() const
{
  return this->dataPtr->visual->GetRotation().Ign();
}

//////////////////////////////////////////////////
bool Light::SetSelected(const bool _s)
{
  if (this->dataPtr->light->getType() != Ogre::Light::LT_DIRECTIONAL)
  {
    if (_s)
      this->dataPtr->line->setMaterial("Gazebo/PurpleGlow");
    else
      this->dataPtr->line->setMaterial("Gazebo/LightOn");
  }

  return true;
}

//////////////////////////////////////////////////
void Light::ToggleShowVisual()
{
  this->dataPtr->visual->ToggleVisible();
}

//////////////////////////////////////////////////
void Light::ShowVisual(const bool _s)
{
  this->dataPtr->visual->SetVisible(_s);
}

//////////////////////////////////////////////////
bool Light::Visible() const
{
  return this->dataPtr->visual->GetVisible();
}

//////////////////////////////////////////////////
void Light::SetLightType(const std::string &_type)
{
  // Set the light _type
  if (_type == "point")
    this->dataPtr->light->setType(Ogre::Light::LT_POINT);
  else if (_type == "directional")
    this->dataPtr->light->setType(Ogre::Light::LT_DIRECTIONAL);
  else if (_type == "spot")
    this->dataPtr->light->setType(Ogre::Light::LT_SPOTLIGHT);
  else
  {
    gzerr << "Unknown light type[" << _type << "]\n";
  }

  if (this->dataPtr->sdf->Get<std::string>("type") != _type)
    this->dataPtr->sdf->GetAttribute("type")->Set(_type);

  this->CreateVisual();
}

//////////////////////////////////////////////////
std::string Light::LightType() const
{
  if (this->dataPtr->sdf)
    return this->dataPtr->sdf->Get<std::string>("type");

  return std::string();
}

//////////////////////////////////////////////////
void Light::SetDiffuseColor(const common::Color &_color)
{
  sdf::ElementPtr elem = this->dataPtr->sdf->GetElement("diffuse");

  if (_color != elem->Get<common::Color>())
    elem->Set(_color);

  this->dataPtr->light->setDiffuseColour(_color.r, _color.g, _color.b);
}

//////////////////////////////////////////////////
common::Color Light::DiffuseColor() const
{
  return this->dataPtr->sdf->GetElement("diffuse")->Get<common::Color>();
}

//////////////////////////////////////////////////
common::Color Light::SpecularColor() const
{
  return this->dataPtr->sdf->GetElement("specular")->Get<common::Color>();
}

//////////////////////////////////////////////////
void Light::SetSpecularColor(const common::Color &_color)
{
  sdf::ElementPtr elem = this->dataPtr->sdf->GetElement("specular");

  if (elem->Get<common::Color>() != _color)
    elem->Set(_color);

  this->dataPtr->light->setSpecularColour(_color.r, _color.g, _color.b);
}

//////////////////////////////////////////////////
void Light::SetDirection(const ignition::math::Vector3d &_dir)
{
  // Set the direction which the light points
  math::Vector3 vec = _dir;
  vec.Normalize();

  if (vec != this->dataPtr->sdf->Get<math::Vector3>("direction"))
    this->dataPtr->sdf->GetElement("direction")->Set(vec);

  this->dataPtr->light->setDirection(vec.x, vec.y, vec.z);
}

//////////////////////////////////////////////////
ignition::math::Vector3d Light::Direction() const
{
  return this->dataPtr->sdf->Get<ignition::math::Vector3d>("direction");
}

//////////////////////////////////////////////////
void Light::SetAttenuation(double constant, double linear, double quadratic)
{
  // Constant factor. 1.0 means never attenuate, 0.0 is complete attenuation
  if (constant < 0)
    constant = 0;
  else if (constant > 1.0)
    constant = 1.0;

  // Linear factor. 1 means attenuate evenly over the distance
  if (linear < 0)
    linear = 0;
  else if (linear > 1.0)
    linear = 1.0;

  sdf::ElementPtr elem = this->dataPtr->sdf->GetElement("attenuation");
  elem->GetElement("constant")->Set(constant);
  elem->GetElement("linear")->Set(linear);
  elem->GetElement("quadratic")->Set(quadratic);

  // Set attenuation
  this->dataPtr->light->setAttenuation(elem->Get<double>("range"),
                              constant, linear, quadratic);
}


//////////////////////////////////////////////////
void Light::SetRange(const double _range)
{
  sdf::ElementPtr elem = this->dataPtr->sdf->GetElement("attenuation");

  elem->GetElement("range")->Set(_range);

  this->dataPtr->light->setAttenuation(elem->Get<double>("range"),
                              elem->Get<double>("constant"),
                              elem->Get<double>("linear"),
                              elem->Get<double>("quadratic"));
}

//////////////////////////////////////////////////
void Light::SetCastShadows(const bool _cast)
{
  if (this->dataPtr->light->getType() == Ogre::Light::LT_DIRECTIONAL)
  {
    this->dataPtr->light->setCastShadows(_cast);
  }
  else
  {
    this->dataPtr->light->setCastShadows(false);
  }
}

//////////////////////////////////////////////////
bool Light::CastShadows() const
{
  if (this->dataPtr->light)
    return this->dataPtr->light->getCastShadows();

  return false;
}

//////////////////////////////////////////////////
void Light::SetSpotInnerAngle(const double _angle)
{
  sdf::ElementPtr elem = this->dataPtr->sdf->GetElement("spot");
  elem->GetElement("inner_angle")->Set(_angle);

  if (this->dataPtr->light->getType() == Ogre::Light::LT_SPOTLIGHT)
  {
    this->dataPtr->light->setSpotlightRange(
        Ogre::Radian(elem->Get<double>("inner_angle")),
        Ogre::Radian(elem->Get<double>("outer_angle")),
        elem->Get<double>("falloff"));
  }
}

//////////////////////////////////////////////////
void Light::SetSpotOuterAngle(const double _angle)
{
  sdf::ElementPtr elem = this->dataPtr->sdf->GetElement("spot");
  elem->GetElement("outer_angle")->Set(_angle);

  if (this->dataPtr->light->getType() == Ogre::Light::LT_SPOTLIGHT)
  {
    this->dataPtr->light->setSpotlightRange(
        Ogre::Radian(elem->Get<double>("inner_angle")),
        Ogre::Radian(elem->Get<double>("outer_angle")),
        elem->Get<double>("falloff"));
  }
}

//////////////////////////////////////////////////
void Light::SetSpotFalloff(const double _angle)
{
  sdf::ElementPtr elem = this->dataPtr->sdf->GetElement("spot");
  elem->GetElement("falloff")->Set(_angle);

  if (this->dataPtr->light->getType() == Ogre::Light::LT_SPOTLIGHT)
  {
    this->dataPtr->light->setSpotlightRange(
        Ogre::Radian(elem->Get<double>("inner_angle")),
        Ogre::Radian(elem->Get<double>("outer_angle")),
        elem->Get<double>("falloff"));
  }
}

//////////////////////////////////////////////////
void Light::FillMsg(msgs::Light &_msg) const
{
  std::string lightType = this->dataPtr->sdf->Get<std::string>("type");

  _msg.set_name(this->Name());

  if (lightType == "point")
    _msg.set_type(msgs::Light::POINT);
  else if (lightType == "spot")
    _msg.set_type(msgs::Light::SPOT);
  else if (lightType == "directional")
    _msg.set_type(msgs::Light::DIRECTIONAL);

  msgs::Set(_msg.mutable_pose()->mutable_position(), this->Position());
  msgs::Set(_msg.mutable_pose()->mutable_orientation(),
      this->Rotation());
  msgs::Set(_msg.mutable_diffuse(), this->DiffuseColor());
  msgs::Set(_msg.mutable_specular(), this->SpecularColor());
  msgs::Set(_msg.mutable_direction(), this->Direction());

  _msg.set_cast_shadows(this->dataPtr->light->getCastShadows());

  sdf::ElementPtr elem = this->dataPtr->sdf->GetElement("attenuation");
  _msg.set_attenuation_constant(elem->Get<double>("constant"));
  _msg.set_attenuation_linear(elem->Get<double>("linear"));
  _msg.set_attenuation_quadratic(elem->Get<double>("quadratic"));
  _msg.set_range(elem->Get<double>("range"));

  if (lightType == "spot")
  {
    elem = this->dataPtr->sdf->GetElement("spot");
    _msg.set_spot_inner_angle(elem->Get<double>("inner_angle"));
    _msg.set_spot_outer_angle(elem->Get<double>("outer_angle"));
    _msg.set_spot_falloff(elem->Get<double>("falloff"));
  }
}

//////////////////////////////////////////////////
LightPtr Light::Clone(const std::string &_name, ScenePtr _scene)
{
  LightPtr result(new Light(_scene));
  sdf::ElementPtr sdfCopy(new sdf::Element);
  sdfCopy->Copy(this->dataPtr->sdf);
  sdfCopy->GetAttribute("name")->Set(_name);
  result->Load(sdfCopy);

  result->SetPosition(this->Position());
  result->SetRotation(this->Rotation());

  return result;
}
