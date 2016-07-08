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

#include "gazebo/common/Console.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/selection_buffer/MaterialSwitcher.hh"
#include "gazebo/rendering/RenderTypes.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
MaterialSwitcher::MaterialSwitcher()
: lastTechnique(nullptr)
{
  this->currentColor = common::Color(0.0, 0.0, 0.1);
}

/////////////////////////////////////////////////
MaterialSwitcher::~MaterialSwitcher()
{
}

/////////////////////////////////////////////////
Ogre::Technique *MaterialSwitcher::handleSchemeNotFound(
    uint16_t /*_schemeIndex*/, const Ogre::String & /*_schemeName*/,
    Ogre::Material *_originalMaterial, uint16_t /*_lodIndex*/,
    const Ogre::Renderable *_rend)
{
  if (_rend)
  {
    if (typeid(*_rend) == typeid(Ogre::SubEntity))
    {
      const Ogre::SubEntity *subEntity =
        static_cast<const Ogre::SubEntity *>(_rend);

      if (!(subEntity->getParent()->getVisibilityFlags() &
          GZ_VISIBILITY_SELECTABLE))
      {
        const_cast<Ogre::SubEntity *>(subEntity)->setCustomParameter(1,
            Ogre::Vector4(0, 0, 0, 0));
        return nullptr;
      }

      if (this->lastEntity == subEntity->getParent()->getName())
      {
        const_cast<Ogre::SubEntity *>(subEntity)->setCustomParameter(1,
            Ogre::Vector4(this->currentColor.r, this->currentColor.g,
                          this->currentColor.b, 1.0));
      }
      else
      {
        // load the selection buffer material
        if (this->plainTechnique == nullptr)
        {
          // plain opaque material
          Ogre::ResourcePtr res =
            Ogre::MaterialManager::getSingleton().load("gazebo/plain_color",
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

          // OGRE 1.9 changes the shared pointer definition
          #if (OGRE_VERSION < ((1 << 16) | (9 << 8) | 0))
          Ogre::MaterialPtr plainMaterial = static_cast<Ogre::MaterialPtr>(res);
          #else
          Ogre::MaterialPtr plainMaterial = res.staticCast<Ogre::Material>();
          #endif

          this->plainTechnique = plainMaterial->getTechnique(0);
          Ogre::Pass *plainPass = this->plainTechnique->getPass(0);
          plainPass->setDepthCheckEnabled(true);
          plainPass->setDepthWriteEnabled(true);

          // overlay material
          Ogre::MaterialPtr overlayMaterial =
              plainMaterial->clone("plain_color_overlay");
          this->overlayTechnique =
              overlayMaterial->getTechnique(0);
          if (!this->overlayTechnique || !this->overlayTechnique->getPass(0))
          {
            gzerr << "Problem creating the selection buffer overlay material"
                << std::endl;
            return nullptr;
          }
          Ogre::Pass *overlayPass = this->overlayTechnique->getPass(0);
          overlayPass->setDepthCheckEnabled(false);
          overlayPass->setDepthWriteEnabled(false);
        }

        // Make sure we keep the same depth properties so that
        // certain overlay objects can be picked by the mouse.
        Ogre::Technique *newTechnique = this->plainTechnique;

        Ogre::Technique *originalTechnique = _originalMaterial->getTechnique(0);
        if (originalTechnique)
        {
          Ogre::Pass *originalPass = originalTechnique->getPass(0);
          if (originalPass)
          {
            // check if it's an overlay material by assuming the
            // depth check and depth write properties are off.
            bool depthCheck = originalPass->getDepthCheckEnabled();
            bool depthWrite = originalPass->getDepthWriteEnabled();
            if (!depthCheck && !depthWrite)
              newTechnique = this->overlayTechnique;
          }
        }

        this->lastTechnique = newTechnique;

        this->GetNextColor();

        const_cast<Ogre::SubEntity *>(subEntity)->setCustomParameter(1,
            Ogre::Vector4(this->currentColor.r, this->currentColor.g,
              this->currentColor.b, 1.0));

        this->lastEntity = subEntity->getParent()->getName();
        this->colorDict[this->currentColor.GetAsRGBA()] = this->lastEntity;
      }

      return this->lastTechnique;
    }
    // else
    //    gzerr << "Object is not a SubEntity["
    //          << _rend->getMaterial()->getName() << "] Type[" <<
    //          typeid(*_rend).name() << "]\n";
  }
  // else
  //  gzerr << "Rendering scheme without a Renderable: " << _schemeName
  //        << ", " + _originalMaterial->getName() << std::endl;
  return nullptr;
}

/////////////////////////////////////////////////
const std::string &MaterialSwitcher::GetEntityName(
    const common::Color &_color) const
{
  ColorMapConstIter iter = this->colorDict.find(_color.GetAsRGBA());

  if (iter != this->colorDict.end())
    return (*iter).second;
  else
    return this->emptyString;
}

/////////////////////////////////////////////////
void MaterialSwitcher::GetNextColor()
{
  common::Color::ARGB color = this->currentColor.GetAsARGB();
  color++;
  this->currentColor.SetFromARGB(color);
}

/////////////////////////////////////////////////
void MaterialSwitcher::Reset()
{
  this->currentColor = common::Color(0.0, 0.0, 0.1);
  this->lastEntity.clear();
  this->colorDict.clear();
}
