/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include "gazebo/common/Color.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/common/Console.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Material.hh"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
void Material::CreateMaterials()
{
  Ogre::MaterialPtr mat;
  Ogre::Technique *tech;
  Ogre::Pass *pass;
  Ogre::TextureUnitState *texState;

  mat = Ogre::MaterialManager::getSingleton().create(
      "__GAZEBO_PURPLE_MATERIAL__", "General");
  tech = mat->getTechnique(0);
  pass = tech->getPass(0);
  tech->setLightingEnabled(false);
  pass->setAmbient(1.0, 0.0, 1.0);
  pass->setDiffuse(1.0, 0.0, 1.0, 0.5);
  texState = pass->createTextureUnitState();
  texState->setColourOperationEx(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, Ogre::ColourValue(1, 0, 1));

  mat = Ogre::MaterialManager::getSingleton().create(
      "__GAZEBO_RED_MATERIAL__", "General");
  tech = mat->getTechnique(0);
  pass = tech->getPass(0);
  tech->setLightingEnabled(false);
  pass->setAmbient(1.0, 0.0, 0.0);
  pass->setDiffuse(1.0, 0.0, 0.0, 0.5);
  texState = pass->createTextureUnitState();
  texState->setColourOperationEx(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, Ogre::ColourValue(1, 0, 0));

  mat = Ogre::MaterialManager::getSingleton().create(
      "__GAZEBO_GREEN_MATERIAL__", "General");
  tech = mat->getTechnique(0);
  pass = tech->getPass(0);
  tech->setLightingEnabled(false);
  pass->setAmbient(0.0, 1.0, 0.0);
  pass->setDiffuse(0.0, 1.0, 0.0, 0.5);
  texState = pass->createTextureUnitState();
  texState->setColourOperationEx(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, Ogre::ColourValue(0, 1, 0));

  mat = Ogre::MaterialManager::getSingleton().create(
      "__GAZEBO_BLUE_MATERIAL__", "General");
  tech = mat->getTechnique(0);
  pass = tech->getPass(0);
  tech->setLightingEnabled(false);
  pass->setAmbient(0.0, 0.0, 1.0);
  pass->setDiffuse(0.0, 0.0, 1.0, 0.5);
  texState = pass->createTextureUnitState();
  texState->setColourOperationEx(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, Ogre::ColourValue(0, 0, 1));

  mat = Ogre::MaterialManager::getSingleton().create(
      "__GAZEBO_TRANS_RED_MATERIAL__", "General");
  tech = mat->getTechnique(0);
  pass = tech->getPass(0);
  tech->setLightingEnabled(false);
  tech->setDepthWriteEnabled(false);
  pass->setAmbient(1.0, 0.0, 0.0);
  pass->setDiffuse(1.0, 0.0, 0.0, 0.5);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  texState = pass->createTextureUnitState();
  texState->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
                               Ogre::LBS_CURRENT, 0.5);
  texState->setColourOperationEx(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, Ogre::ColourValue(1, 0, 0));

  mat = Ogre::MaterialManager::getSingleton().create(
      "__GAZEBO_TRANS_GREEN_MATERIAL__", "General");
  tech = mat->getTechnique(0);
  pass = tech->getPass(0);
  tech->setLightingEnabled(false);
  tech->setDepthWriteEnabled(false);
  pass->setAmbient(0.0, 1.0, 0.0);
  pass->setDiffuse(0.0, 1.0, 0.0, 0.5);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  texState = pass->createTextureUnitState();
  texState->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
                               Ogre::LBS_CURRENT, 0.5);
  texState->setColourOperationEx(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, Ogre::ColourValue(0, 1, 0));

  mat = Ogre::MaterialManager::getSingleton().create(
      "__GAZEBO_TRANS_BLUE_MATERIAL__", "General");
  tech = mat->getTechnique(0);
  pass = tech->getPass(0);
  tech->setLightingEnabled(false);
  tech->setDepthWriteEnabled(false);
  pass->setAmbient(0.0, 0.0, 1.0);
  pass->setDiffuse(0.0, 0.0, 1.0, 0.5);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  texState = pass->createTextureUnitState();
  texState->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
                               Ogre::LBS_CURRENT, 0.5);
  texState->setColourOperationEx(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, Ogre::ColourValue(0, 0, 1));

  mat = Ogre::MaterialManager::getSingleton().create(
      "__GAZEBO_TRANS_PURPLE_MATERIAL__", "General");
  tech = mat->getTechnique(0);
  pass = tech->getPass(0);
  tech->setLightingEnabled(false);
  tech->setDepthWriteEnabled(false);
  pass->setAmbient(1.0, 0.0, 1.0);
  pass->setDiffuse(1.0, 0.0, 1.0, 0.5);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  texState = pass->createTextureUnitState();
  texState->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
                               Ogre::LBS_CURRENT, 0.5);
  texState->setColourOperationEx(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
      Ogre::LBS_CURRENT, Ogre::ColourValue(1, 0, 1));
}

//////////////////////////////////////////////////
void Material::Update(const gazebo::common::Material *_mat)
{
  Ogre::MaterialPtr matPtr;

  if (Ogre::MaterialManager::getSingleton().resourceExists(_mat->GetName()))
    matPtr = Ogre::MaterialManager::getSingleton().getByName(
        _mat->GetName(), "General");
  else
    matPtr = Ogre::MaterialManager::getSingleton().create(
        _mat->GetName(), "General");

  Ogre::Pass *pass = matPtr->getTechnique(0)->getPass(0);

  auto ambient =  _mat->Ambient();
  auto diffuse =  _mat->Diffuse();
  auto specular = _mat->Specular();
  auto emissive = _mat->Emissive();
  float transparency = _mat->GetTransparency();

  // use transparency value if specified otherwise use diffuse alpha value
  double alpha = transparency > 0 ? 1.0 - transparency : diffuse.A();
  diffuse.A() = alpha;
  pass->setDiffuse(diffuse.R(), diffuse.G(), diffuse.B(), diffuse.A());
  pass->setAmbient(ambient.R(), ambient.G(), ambient.B());
  pass->setDepthWriteEnabled(_mat->GetDepthWrite());

  if (diffuse.A() < 1.0)
  {
    // set up pass for rendering transparency
    pass->setDepthWriteEnabled(false);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  }

  pass->setSpecular(specular.R(), specular.G(), specular.B(), specular.A());
  pass->setSelfIllumination(emissive.R(), emissive.G(), emissive.B());
  pass->setShininess(_mat->GetShininess());
  pass->setLightingEnabled(_mat->GetLighting());

  // Only add the texture unit if it's not present in the material
  if (!_mat->GetTextureImage().empty() &&
      pass->getTextureUnitState(_mat->GetTextureImage()) == NULL)
  {
    // Make sure to add the path to the texture image.
    RenderEngine::Instance()->AddResourcePath(_mat->GetTextureImage());
    Ogre::TextureUnitState *texState = pass->createTextureUnitState(
        _mat->GetTextureImage());
    texState->setTextureName(_mat->GetTextureImage());
    texState->setName(_mat->GetTextureImage());
  }
}

//////////////////////////////////////////////////
bool Material::GetMaterialAsColor(const std::string &_materialName,
          common::Color &_ambient, common::Color &_diffuse,
          common::Color &_specular, common::Color &_emissive)
{
  ignition::math::Color ambient;
  ignition::math::Color diffuse;
  ignition::math::Color specular;
  ignition::math::Color emissive;
  bool success = MaterialAsColor(_materialName, ambient, diffuse, specular,
      emissive);
#ifndef _WIN32
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  _ambient = ambient;
  _diffuse = diffuse;
  _specular = specular;
  _emissive = emissive;
#ifndef _WIN32
  #pragma GCC diagnostic pop
#endif
  return success;
}

//////////////////////////////////////////////////
bool Material::MaterialAsColor(const std::string &_materialName,
          ignition::math::Color &_ambient, ignition::math::Color &_diffuse,
          ignition::math::Color &_specular, ignition::math::Color &_emissive)
{
  Ogre::MaterialPtr matPtr;

  if (Ogre::MaterialManager::getSingleton().resourceExists(_materialName))
  {
    matPtr = Ogre::MaterialManager::getSingleton().getByName(_materialName,
        "General");

    if (matPtr.isNull())
      return false;

    Ogre::Technique *technique = matPtr->getTechnique(0);
    if (technique)
    {
      Ogre::Pass *pass = technique->getPass(0);
      if (pass)
      {
        _ambient = Conversions::Convert(pass->getAmbient());
        _diffuse = Conversions::Convert(pass->getDiffuse());
        _specular = Conversions::Convert(pass->getSpecular());
        _emissive = Conversions::Convert(pass->getSelfIllumination());
        return true;
      }
    }
  }

  return false;
}
