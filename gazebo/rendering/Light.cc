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

/* Desc: A Light
 * Author: Nate Koenig
 * Date: 15 July 2003
 */

#include <boost/bind.hpp>

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Light.hh"

using namespace gazebo;
using namespace rendering;

unsigned int Light::lightCounter = 0;

//////////////////////////////////////////////////
Light::Light(ScenePtr _scene)
{
  this->line = NULL;
  this->scene = _scene;

  this->lightCounter++;

  this->sdf.reset(new sdf::Element);
  sdf::initFile("light.sdf", this->sdf);
}

//////////////////////////////////////////////////
Light::~Light()
{
  if (this->light)
    this->scene->GetManager()->destroyLight(this->GetName());

  this->scene->GetManager()->destroyEntity(
      this->GetName() + "_selection_sphere");

  this->visual->DeleteDynamicLine(this->line);
  delete this->line;
  this->scene->RemoveVisual(this->visual);
  this->visual.reset();

  this->sdf->Reset();
  this->sdf.reset();

  this->scene.reset();
}

//////////////////////////////////////////////////
void Light::Load(sdf::ElementPtr _sdf)
{
  this->sdf->Copy(_sdf);
  this->Load();
}

//////////////////////////////////////////////////
void Light::Load()
{
  math::Vector3 vec;

  try
  {
    this->light = this->scene->GetManager()->createLight(this->GetName());
  }
  catch(Ogre::Exception &e)
  {
    gzthrow("Ogre Error:" << e.getFullDescription() << "\n" << \
        "Unable to create a light");
  }

  this->Update();

  this->visual.reset(new Visual(this->GetName(),
                     this->scene->GetWorldVisual()));
  this->visual->AttachObject(this->light);
  this->scene->AddVisual(this->visual);

  this->CreateVisual();
}

//////////////////////////////////////////////////
void Light::Update()
{
  this->SetCastShadows(this->sdf->Get<bool>("cast_shadows"));

  this->SetLightType(this->sdf->Get<std::string>("type"));
  this->SetDiffuseColor(
      this->sdf->GetElement("diffuse")->Get<common::Color>());
  this->SetSpecularColor(
      this->sdf->GetElement("specular")->Get<common::Color>());
  this->SetDirection(
      this->sdf->Get<math::Vector3>("direction"));

  if (this->sdf->HasElement("attenuation"))
  {
    sdf::ElementPtr elem = this->sdf->GetElement("attenuation");

    this->SetAttenuation(elem->Get<double>("constant"),
                         elem->Get<double>("linear"),
                         elem->Get<double>("quadratic"));
    this->SetRange(elem->Get<double>("range"));
  }

  if (this->sdf->HasElement("spot"))
  {
    sdf::ElementPtr elem = this->sdf->GetElement("spot");
    this->SetSpotInnerAngle(elem->Get<double>("inner_angle"));
    this->SetSpotOuterAngle(elem->Get<double>("outer_angle"));
    this->SetSpotFalloff(elem->Get<double>("falloff"));
  }
}

//////////////////////////////////////////////////
void Light::UpdateSDFFromMsg(ConstLightPtr &_msg)
{
  this->sdf->GetAttribute("name")->Set(_msg->name());

  if (_msg->has_type() && _msg->type() == msgs::Light::POINT)
    this->sdf->GetAttribute("type")->Set("point");
  else if (_msg->has_type() && _msg->type() == msgs::Light::SPOT)
    this->sdf->GetAttribute("type")->Set("spot");
  else if (_msg->has_type() && _msg->type() == msgs::Light::DIRECTIONAL)
    this->sdf->GetAttribute("type")->Set("directional");

  if (_msg->has_diffuse())
  {
    this->sdf->GetElement("diffuse")->Set(
        msgs::Convert(_msg->diffuse()));
  }

  if (_msg->has_specular())
  {
    this->sdf->GetElement("specular")->Set(
        msgs::Convert(_msg->specular()));
  }

  if (_msg->has_direction())
  {
    this->sdf->GetElement("direction")->Set(
        msgs::Convert(_msg->direction()));
  }

  if (_msg->has_attenuation_constant())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("attenuation");
    elem->GetElement("constant")->Set(_msg->attenuation_constant());
  }

  if (_msg->has_attenuation_linear())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("attenuation");
    elem->GetElement("linear")->Set(_msg->attenuation_linear());
  }

  if (_msg->has_attenuation_quadratic())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("attenuation");
    elem->GetElement("quadratic")->Set(_msg->attenuation_quadratic());
  }

  if (_msg->has_range())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("attenuation");
    elem->GetElement("range")->Set(_msg->range());
  }

  if (_msg->has_cast_shadows())
    this->sdf->GetElement("cast_shadows")->Set(_msg->cast_shadows());

  if (_msg->has_spot_inner_angle())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("spot");
    elem->GetElement("inner_angle")->Set(_msg->spot_inner_angle());
  }

  if (_msg->has_spot_outer_angle())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("spot");
    elem->GetElement("outer_angle")->Set(_msg->spot_outer_angle());
  }

  if (_msg->has_spot_falloff())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("spot");
    elem->GetElement("falloff")->Set(_msg->spot_falloff());
  }
}

//////////////////////////////////////////////////
void Light::UpdateFromMsg(ConstLightPtr &_msg)
{
  this->UpdateSDFFromMsg(_msg);

  this->Update();

  if (_msg->has_pose())
    this->SetPosition(msgs::Convert(_msg->pose().position()));
}

//////////////////////////////////////////////////
void Light::LoadFromMsg(ConstLightPtr &_msg)
{
  this->UpdateSDFFromMsg(_msg);

  this->Load();

  if (_msg->has_pose())
    this->SetPosition(msgs::Convert(_msg->pose().position()));
}

//////////////////////////////////////////////////
void Light::SetName(const std::string &_name)
{
  this->sdf->GetAttribute("name")->Set(_name);
}

//////////////////////////////////////////////////
std::string Light::GetName() const
{
  return this->sdf->Get<std::string>("name");
}

//////////////////////////////////////////////////
std::string Light::GetType() const
{
  return this->sdf->Get<std::string>("type");
}

//////////////////////////////////////////////////
// The lines draw a visualization of the camera
void Light::CreateVisual()
{
  if (!this->visual)
    return;

  if (this->line)
    this->line->Clear();
  else
  {
    this->line = this->visual->CreateDynamicLine(RENDERING_LINE_LIST);

    this->line->setMaterial("Gazebo/LightOn");

    this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);

    this->visual->SetVisible(true);

    // Create a scene node to hold the light selection object.
    Ogre::SceneNode *visSceneNode;
    visSceneNode = this->visual->GetSceneNode()->createChildSceneNode(
        this->GetName() + "_SELECTION_NODE_");

    // Make sure the unit_sphere has been inserted.
    this->visual->InsertMesh("unit_sphere");

    Ogre::Entity *ent =
        visSceneNode->getCreator()->createEntity(this->GetName() +
        "_selection_sphere", "unit_sphere");

    ent->setMaterialName("Gazebo/White");

    // Create the selection object.
    Ogre::MovableObject *obj = static_cast<Ogre::MovableObject*>(ent);

    // Attach the selection object to the light visual
    visSceneNode->attachObject(obj);

    // Make sure the selection object is rendered only in the selection
    // buffer.
    obj->setVisibilityFlags(GZ_VISIBILITY_SELECTION);
    obj->setUserAny(Ogre::Any(this->GetName()));
    obj->setCastShadows(false);

    // Scale the selection object to roughly match the light visual size.
    visSceneNode->setScale(0.25, 0.25, 0.25);
  }

  std::string lightType = this->sdf->Get<std::string>("type");

  if (lightType == "directional")
  {
    float s =.5;
    this->line->AddPoint(math::Vector3(-s, -s, 0));
    this->line->AddPoint(math::Vector3(-s, s, 0));

    this->line->AddPoint(math::Vector3(-s, s, 0));
    this->line->AddPoint(math::Vector3(s, s, 0));

    this->line->AddPoint(math::Vector3(s, s, 0));
    this->line->AddPoint(math::Vector3(s, -s, 0));

    this->line->AddPoint(math::Vector3(s, -s, 0));
    this->line->AddPoint(math::Vector3(-s, -s, 0));

    this->line->AddPoint(math::Vector3(0, 0, 0));
    this->line->AddPoint(math::Vector3(0, 0, -s));
  }
  if (lightType == "point")
  {
    float s = 0.1;
    this->line->AddPoint(math::Vector3(-s, -s, 0));
    this->line->AddPoint(math::Vector3(-s, s, 0));

    this->line->AddPoint(math::Vector3(-s, s, 0));
    this->line->AddPoint(math::Vector3(s, s, 0));

    this->line->AddPoint(math::Vector3(s, s, 0));
    this->line->AddPoint(math::Vector3(s, -s, 0));

    this->line->AddPoint(math::Vector3(s, -s, 0));
    this->line->AddPoint(math::Vector3(-s, -s, 0));


    this->line->AddPoint(math::Vector3(-s, -s, 0));
    this->line->AddPoint(math::Vector3(0, 0, s));

    this->line->AddPoint(math::Vector3(-s, s, 0));
    this->line->AddPoint(math::Vector3(0, 0, s));

    this->line->AddPoint(math::Vector3(s, s, 0));
    this->line->AddPoint(math::Vector3(0, 0, s));

    this->line->AddPoint(math::Vector3(s, -s, 0));
    this->line->AddPoint(math::Vector3(0, 0, s));



    this->line->AddPoint(math::Vector3(-s, -s, 0));
    this->line->AddPoint(math::Vector3(0, 0, -s));

    this->line->AddPoint(math::Vector3(-s, s, 0));
    this->line->AddPoint(math::Vector3(0, 0, -s));

    this->line->AddPoint(math::Vector3(s, s, 0));
    this->line->AddPoint(math::Vector3(0, 0, -s));

    this->line->AddPoint(math::Vector3(s, -s, 0));
    this->line->AddPoint(math::Vector3(0, 0, -s));
  }
  else if (lightType == "spot")
  {
    double innerAngle = this->light->getSpotlightInnerAngle().valueRadians();
    double outerAngle = this->light->getSpotlightOuterAngle().valueRadians();

    double angles[2];
    double range = 0.2;
    angles[0] = range * tan(outerAngle);
    angles[1] = range * tan(innerAngle);

    unsigned int i = 0;
    this->line->AddPoint(math::Vector3(0, 0, 0));
    this->line->AddPoint(math::Vector3(angles[i], angles[i], -range));

    for (i = 0; i < 2; i++)
    {
      this->line->AddPoint(math::Vector3(0, 0, 0));
      this->line->AddPoint(math::Vector3(angles[i], angles[i], -range));

      this->line->AddPoint(math::Vector3(0, 0, 0));
      this->line->AddPoint(math::Vector3(-angles[i], -angles[i], -range));

      this->line->AddPoint(math::Vector3(0, 0, 0));
      this->line->AddPoint(math::Vector3(angles[i], -angles[i], -range));

      this->line->AddPoint(math::Vector3(0, 0, 0));
      this->line->AddPoint(math::Vector3(-angles[i], angles[i], -range));

      this->line->AddPoint(math::Vector3(angles[i], angles[i], -range));
      this->line->AddPoint(math::Vector3(-angles[i], angles[i], -range));

      this->line->AddPoint(math::Vector3(-angles[i], angles[i], -range));
      this->line->AddPoint(math::Vector3(-angles[i], -angles[i], -range));

      this->line->AddPoint(math::Vector3(-angles[i], -angles[i], -range));
      this->line->AddPoint(math::Vector3(angles[i], -angles[i], -range));

      this->line->AddPoint(math::Vector3(angles[i], -angles[i], -range));
      this->line->AddPoint(math::Vector3(angles[i], angles[i], -range));
    }
  }
}

//////////////////////////////////////////////////
void Light::SetPosition(const math::Vector3 &_p)
{
  this->visual->SetPosition(_p);
}

//////////////////////////////////////////////////
math::Vector3 Light::GetPosition() const
{
  return this->visual->GetPosition();
}

//////////////////////////////////////////////////
bool Light::SetSelected(bool _s)
{
  if (this->light->getType() != Ogre::Light::LT_DIRECTIONAL)
  {
    if (_s)
      this->line->setMaterial("Gazebo/PurpleGlow");
    else
      this->line->setMaterial("Gazebo/LightOn");
  }

  return true;
}

//////////////////////////////////////////////////
void Light::ToggleShowVisual()
{
  this->visual->ToggleVisible();
}

//////////////////////////////////////////////////
void Light::ShowVisual(bool _s)
{
  this->visual->SetVisible(_s);
}

//////////////////////////////////////////////////
bool Light::GetVisible() const
{
  return this->visual->GetVisible();
}

//////////////////////////////////////////////////
void Light::SetLightType(const std::string &_type)
{
  // Set the light _type
  if (_type == "point")
    this->light->setType(Ogre::Light::LT_POINT);
  else if (_type == "directional")
    this->light->setType(Ogre::Light::LT_DIRECTIONAL);
  else if (_type == "spot")
    this->light->setType(Ogre::Light::LT_SPOTLIGHT);
  else
  {
    gzerr << "Unknown light type[" << _type << "]\n";
  }

  if (this->sdf->Get<std::string>("type") != _type)
    this->sdf->GetAttribute("type")->Set(_type);

  this->CreateVisual();
}

//////////////////////////////////////////////////
void Light::SetDiffuseColor(const common::Color &_color)
{
  sdf::ElementPtr elem = this->sdf->GetElement("diffuse");

  if (_color != elem->Get<common::Color>())
    elem->Set(_color);

  this->light->setDiffuseColour(_color.r, _color.g, _color.b);
}

//////////////////////////////////////////////////
common::Color Light::GetDiffuseColor() const
{
  return this->sdf->GetElement("diffuse")->Get<common::Color>();
}

//////////////////////////////////////////////////
common::Color Light::GetSpecularColor() const
{
  return this->sdf->GetElement("specular")->Get<common::Color>();
}

//////////////////////////////////////////////////
void Light::SetSpecularColor(const common::Color &_color)
{
  sdf::ElementPtr elem = this->sdf->GetElement("specular");

  if (elem->Get<common::Color>() != _color)
    elem->Set(_color);

  this->light->setSpecularColour(_color.r, _color.g, _color.b);
}

//////////////////////////////////////////////////
void Light::SetDirection(const math::Vector3 &_dir)
{
  // Set the direction which the light points
  math::Vector3 vec = _dir;
  vec.Normalize();

  if (vec != this->sdf->Get<math::Vector3>("direction"))
    this->sdf->GetElement("direction")->Set(vec);

  this->light->setDirection(vec.x, vec.y, vec.z);
}

//////////////////////////////////////////////////
math::Vector3 Light::GetDirection() const
{
  return this->sdf->Get<math::Vector3>("direction");
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

  sdf::ElementPtr elem = this->sdf->GetElement("attenuation");
  elem->GetElement("constant")->Set(constant);
  elem->GetElement("linear")->Set(linear);
  elem->GetElement("quadratic")->Set(quadratic);

  // Set attenuation
  this->light->setAttenuation(elem->Get<double>("range"),
                              constant, linear, quadratic);
}


//////////////////////////////////////////////////
void Light::SetRange(const double &_range)
{
  sdf::ElementPtr elem = this->sdf->GetElement("attenuation");

  elem->GetElement("range")->Set(_range);

  this->light->setAttenuation(elem->Get<double>("range"),
                              elem->Get<double>("constant"),
                              elem->Get<double>("linear"),
                              elem->Get<double>("quadratic"));
}

//////////////////////////////////////////////////
void Light::SetCastShadows(const bool & /*_cast*/)
{
    this->light->setCastShadows(true);
  /*if (this->light->getType() == Ogre::Light::LT_SPOTLIGHT ||
      this->light->getType() == Ogre::Light::LT_DIRECTIONAL)
  {
    this->light->setCastShadows(_cast);
  }
  else
  {
    this->light->setCastShadows(false);
  }*/
}

//////////////////////////////////////////////////
void Light::SetSpotInnerAngle(const double &_angle)
{
  sdf::ElementPtr elem = this->sdf->GetElement("spot");
  elem->GetElement("inner_angle")->Set(_angle);

  if (this->light->getType() == Ogre::Light::LT_SPOTLIGHT)
  {
    this->light->setSpotlightRange(
        Ogre::Radian(elem->Get<double>("inner_angle")),
        Ogre::Radian(elem->Get<double>("outer_angle")),
        elem->Get<double>("falloff"));
  }
}

//////////////////////////////////////////////////
void Light::SetSpotOuterAngle(const double &_angle)
{
  sdf::ElementPtr elem = this->sdf->GetElement("spot");
  elem->GetElement("outer_angle")->Set(_angle);

  if (this->light->getType() == Ogre::Light::LT_SPOTLIGHT)
  {
    this->light->setSpotlightRange(
        Ogre::Radian(elem->Get<double>("inner_angle")),
        Ogre::Radian(elem->Get<double>("outer_angle")),
        elem->Get<double>("falloff"));
  }
}

//////////////////////////////////////////////////
void Light::SetSpotFalloff(const double &_angle)
{
  sdf::ElementPtr elem = this->sdf->GetElement("spot");
  elem->GetElement("falloff")->Set(_angle);

  if (this->light->getType() == Ogre::Light::LT_SPOTLIGHT)
  {
    this->light->setSpotlightRange(
        Ogre::Radian(elem->Get<double>("inner_angle")),
        Ogre::Radian(elem->Get<double>("outer_angle")),
        elem->Get<double>("falloff"));
  }
}

//////////////////////////////////////////////////
void Light::FillMsg(msgs::Light &_msg) const
{
  std::string lightType = this->sdf->Get<std::string>("type");

  _msg.set_name(this->GetName());

  if (lightType == "point")
    _msg.set_type(msgs::Light::POINT);
  else if (lightType == "spot")
    _msg.set_type(msgs::Light::SPOT);
  else if (lightType == "directional")
    _msg.set_type(msgs::Light::DIRECTIONAL);

  msgs::Set(_msg.mutable_pose()->mutable_position(), this->GetPosition());
  msgs::Set(_msg.mutable_pose()->mutable_orientation(), math::Quaternion());
  msgs::Set(_msg.mutable_diffuse(), this->GetDiffuseColor());
  msgs::Set(_msg.mutable_specular(), this->GetSpecularColor());
  msgs::Set(_msg.mutable_direction(), this->GetDirection());

  _msg.set_cast_shadows(this->light->getCastShadows());

  sdf::ElementPtr elem = this->sdf->GetElement("attenuation");
  _msg.set_attenuation_constant(elem->Get<double>("constant"));
  _msg.set_attenuation_linear(elem->Get<double>("linear"));
  _msg.set_attenuation_quadratic(elem->Get<double>("quadratic"));
  _msg.set_range(elem->Get<double>("range"));

  if (lightType == "spot")
  {
    elem = this->sdf->GetElement("spot");
    _msg.set_spot_inner_angle(elem->Get<double>("inner_angle"));
    _msg.set_spot_outer_angle(elem->Get<double>("outer_angle"));
    _msg.set_spot_falloff(elem->Get<double>("falloff"));
  }
}
