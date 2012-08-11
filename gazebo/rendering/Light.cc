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

/* Desc: A Light
 * Author: Nate Koenig
 * Date: 15 July 2003
 */

#include <boost/bind.hpp>

#include "rendering/ogre_gazebo.h"

#include "sdf/sdf.hh"
#include "msgs/msgs.hh"

#include "common/Events.hh"
#include "common/Exception.hh"
#include "common/Console.hh"

#include "rendering/Scene.hh"
#include "rendering/DynamicLines.hh"
#include "rendering/Visual.hh"
#include "rendering/Light.hh"

using namespace gazebo;
using namespace rendering;

unsigned int Light::lightCounter = 0;

//////////////////////////////////////////////////
Light::Light(Scene *scene_)
{
  this->scene = scene_;

  this->lightCounter++;

  this->sdf.reset(new sdf::Element);
  sdf::initFile("light.sdf", this->sdf);
}

//////////////////////////////////////////////////
Light::~Light()
{
  if (this->light)
    this->scene->GetManager()->destroyLight(this->GetName());

  this->sdf->Reset();
  this->sdf.reset();
  this->visual.reset();
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

  this->SetCastShadows(this->sdf->GetValueBool("cast_shadows"));

  this->SetLightType(this->sdf->GetValueString("type"));
  this->SetDiffuseColor(
      this->sdf->GetElement("diffuse")->GetValueColor());
  this->SetSpecularColor(
      this->sdf->GetElement("specular")->GetValueColor());
  this->SetDirection(
      this->sdf->GetValueVector3("direction"));

  if (this->sdf->HasElement("attenuation"))
  {
    sdf::ElementPtr elem = this->sdf->GetElement("attenuation");

    this->SetAttenuation(elem->GetValueDouble("constant"),
                         elem->GetValueDouble("linear"),
                         elem->GetValueDouble("quadratic"));
    this->SetRange(elem->GetValueDouble("range"));
  }

  if (this->sdf->HasElement("spot"))
  {
    sdf::ElementPtr elem = this->sdf->GetElement("spot");
    this->SetSpotInnerAngle(elem->GetValueDouble("inner_angle"));
    this->SetSpotOuterAngle(elem->GetValueDouble("outer_angle"));
    this->SetSpotFalloff(elem->GetValueDouble("falloff"));
  }

  this->visual.reset(new Visual(this->GetName(),
                     this->scene->GetWorldVisual()));
  this->visual->AttachObject(this->light);
  this->scene->RegisterVisual(this->visual);

  this->CreateVisual();
}

//////////////////////////////////////////////////
void Light::LoadFromMsg(ConstLightPtr &msg)
{
  this->sdf->GetAttribute("name")->Set(msg->name());

  if (msg->has_type() && msg->type() == msgs::Light::POINT)
    this->sdf->GetAttribute("type")->Set("point");
  else if (msg->has_type() && msg->type() == msgs::Light::SPOT)
    this->sdf->GetAttribute("type")->Set("spot");
  else
    this->sdf->GetAttribute("type")->Set("directional");

  if (msg->has_diffuse())
  {
    this->sdf->GetElement("diffuse")->Set(
        msgs::Convert(msg->diffuse()));
  }

  if (msg->has_specular())
  {
    this->sdf->GetElement("specular")->Set(
        msgs::Convert(msg->specular()));
  }

  if (msg->has_direction())
  {
    this->sdf->GetElement("direction")->Set(
        msgs::Convert(msg->direction()));
  }

  if (msg->has_attenuation_constant())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("attenuation");
    elem->GetElement("constant")->Set(msg->attenuation_constant());
  }

  if (msg->has_attenuation_linear())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("attenuation");
    elem->GetElement("linear")->Set(msg->attenuation_linear());
  }

  if (msg->has_attenuation_quadratic())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("attenuation");
    elem->GetElement("quadratic")->Set(msg->attenuation_quadratic());
  }

  if (msg->has_range())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("attenuation");
    elem->GetElement("range")->Set(msg->range());
  }

  if (msg->has_cast_shadows())
    this->sdf->GetElement("cast_shadows")->Set(msg->cast_shadows());

  if (msg->has_spot_inner_angle())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("spot");
    elem->GetElement("inner_angle")->Set(msg->spot_inner_angle());
  }

  if (msg->has_spot_outer_angle())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("spot");
    elem->GetElement("outer_angle")->Set(msg->spot_outer_angle());
  }

  if (msg->has_spot_falloff())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("spot");
    elem->GetElement("falloff")->Set(msg->spot_falloff());
  }

  this->Load();

  if (msg->has_pose())
    this->SetPosition(msgs::Convert(msg->pose().position()));
}

//////////////////////////////////////////////////
void Light::SetName(const std::string &name)
{
  this->sdf->GetAttribute("name")->Set(name);
}

//////////////////////////////////////////////////
std::string Light::GetName() const
{
  return this->sdf->GetValueString("name");
}

//////////////////////////////////////////////////
std::string Light::GetType() const
{
  return this->sdf->GetValueString("type");
}

//////////////////////////////////////////////////
void Light::CreateVisual()
{
  // The lines draw a visualization of the camera
  this->line = this->visual->CreateDynamicLine(RENDERING_LINE_LIST);

  std::string lightType = this->sdf->GetValueString("type");

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

  this->line->setMaterial("Gazebo/LightOn");
  this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);

  // turn off light source box visuals by default
  this->visual->SetVisible(true);
}

//////////////////////////////////////////////////
void Light::SetPosition(const math::Vector3 &_p)
{
  this->visual->SetPosition(_p);
}

//////////////////////////////////////////////////
bool Light::SetSelected(bool s)
{
  if (this->light->getType() != Ogre::Light::LT_DIRECTIONAL)
  {
    if (s)
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
void Light::ShowVisual(bool s)
{
  this->visual->SetVisible(s);
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

  if (this->sdf->GetValueString("type") != _type)
    this->sdf->GetAttribute("type")->Set(_type);
}

//////////////////////////////////////////////////
void Light::SetDiffuseColor(const common::Color &_color)
{
  sdf::ElementPtr elem = this->sdf->GetElement("diffuse");

  if (elem->GetValueColor() != _color)
    elem->Set(_color);

  this->light->setDiffuseColour(_color.r, _color.g, _color.b);
}

//////////////////////////////////////////////////
common::Color Light::GetDiffuseColor() const
{
  return this->sdf->GetElement("diffuse")->GetValueColor();
}

//////////////////////////////////////////////////
void Light::SetSpecularColor(const common::Color &_color)
{
  sdf::ElementPtr elem = this->sdf->GetElement("specular");

  if (elem->GetValueColor() != _color)
    elem->Set(_color);

  this->light->setSpecularColour(_color.r, _color.g, _color.b);
}

//////////////////////////////////////////////////
void Light::SetDirection(const math::Vector3 &_dir)
{
  // Set the direction which the light points
  math::Vector3 vec = _dir;
  vec.Normalize();

  if (this->sdf->GetValueVector3("direction") != vec)
    this->sdf->GetElement("direction")->Set(vec);

  this->light->setDirection(vec.x, vec.y, vec.z);
}

//////////////////////////////////////////////////
math::Vector3 Light::GetDirection() const
{
  return this->sdf->GetElement("direction")->GetValueVector3("xyz");
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
  this->light->setAttenuation(elem->GetValueDouble("range"),
                              constant, linear, quadratic);
}


//////////////////////////////////////////////////
void Light::SetRange(const double &range)
{
  sdf::ElementPtr elem = this->sdf->GetElement("attenuation");

  elem->GetElement("range")->Set(range);

  this->light->setAttenuation(elem->GetValueDouble("range"),
                              elem->GetValueDouble("constant"),
                              elem->GetValueDouble("linear"),
                              elem->GetValueDouble("quadratic"));
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
void Light::SetSpotInnerAngle(const double &angle)
{
  sdf::ElementPtr elem = this->sdf->GetElement("spot");
  elem->GetElement("inner_angle")->Set(angle);

  if (this->light->getType() == Ogre::Light::LT_SPOTLIGHT)
  {
    this->light->setSpotlightRange(
        Ogre::Radian(elem->GetValueDouble("inner_angle")),
        Ogre::Radian(elem->GetValueDouble("outer_angle")),
        elem->GetValueDouble("falloff"));
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
        Ogre::Radian(elem->GetValueDouble("inner_angle")),
        Ogre::Radian(elem->GetValueDouble("outer_angle")),
        elem->GetValueDouble("falloff"));
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
        Ogre::Radian(elem->GetValueDouble("inner_angle")),
        Ogre::Radian(elem->GetValueDouble("outer_angle")),
        elem->GetValueDouble("falloff"));
  }
}
