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
#include "rendering/ogre.h"
#include "rendering/Material.hh"

using namespace gazebo;
using namespace rendering;

////////////////////////////////////////////////////////////////////////////////
void Material::Update()
{
  Ogre::MaterialPtr matPtr;

  if (Ogre::MaterialManager::getSingleton().resourceExists(this->GetName()))
    matPtr = Ogre::MaterialManager::getSingleton().getByName(
        this->GetName(), "General");
  else
   matPtr = Ogre::MaterialManager::getSingleton().create(
                  this->GetName(),"General");

  Ogre::Pass *pass = matPtr->getTechnique(0)->getPass(0);

  Color ambient =  this->GetAmbient();
  Color diffuse =  this->GetDiffuse();
  Color specular = this->GetSpecular();
  Color emissive = this->GetEmissive();

  matPtr->getTechnique(0)->setLightingEnabled(true);
  pass->setDiffuse(diffuse.R(), diffuse.G(), diffuse.B(), diffuse.A());
  pass->setAmbient(ambient.R(), ambient.G(), ambient.B());
  pass->setPointSize(this->GetPointSize());

  pass->setSpecular(specular.R(), specular.G(), specular.B(), specular.A());
  pass->setSelfIllumination(emissive.R(), emissive.G(), emissive.B());
  pass->setShininess(this->GetShininess());

  if (!this->GetTextureImage().empty())
  {
    Ogre::TextureUnitState *texState = pass->createTextureUnitState();
    texState->setTextureName( this->GetTextureImage() );
  }
}
