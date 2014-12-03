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
#ifndef _MERGESCHEMEHANDLER_HH_
#define _MERGESCHEMEHANDLER_HH_

#include <OgreMaterialManager.h>
#include <string>
#include <vector>

#include "gazebo/rendering/deferred_shading/MergeMaterialGenerator.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// Class for handling materials who did not specify techniques for the
    /// merging step of deferred/inferred lighting.
    class GZ_RENDERING_DEFERRED_VISIBLE MergeSchemeHandler :
      public Ogre::MaterialManager::Listener
    {
      public: MergeSchemeHandler(bool _useDSF) : useDSF(_useDSF)
      {
        if (this->useDSF)
        {
          this->materialGenerator =
            new MergeMaterialGenerator("InferredLighting", true);
          this->techName = "InferredLighting";
        }
        else
        {
          this->materialGenerator =
            new MergeMaterialGenerator("DeferredLighting", false);
          this->techName = "DeferredLighting";
        }
      }

      /// @copydoc MaterialManager::Listener::handleSchemeNotFound
      public: virtual Ogre::Technique *handleSchemeNotFound(
                  uint16_t _schemeIndex,
                  const Ogre::String &_schemeName,
                  Ogre::Material *_originalMaterial,
                  uint16_t _lodIndex,
                  const Ogre::Renderable* rend);

      /// The string that will be checked in textures to determine whether
      /// they are normal maps
      protected: static const std::string normal_map_pattern;
      protected: bool useDSF;
      protected: Ogre::String techName;
      protected: MergeMaterialGenerator *materialGenerator;

      /// A structure for containing the properties of a material
      protected: struct PassProperties
      {
        PassProperties() : isDeferred(true), normalMap(0), isSkinned(false),
                           hasDiffuseColor(true) {}

        bool isDeferred;
        Ogre::TextureUnitState *normalMap;
        bool isSkinned;
        bool hasDiffuseColor;
        Ogre::vector<Ogre::TextureUnitState*>::type regularTextures;
      };

      /// Inspect a technique and return its relevant properties
      protected: PassProperties InspectPass(Ogre::Pass *_pass,
        uint16_t _lodIndex, const Ogre::Renderable *_rend);

      /// Get the permutation of material flags that fit a certain
      /// property sheet
      protected: MaterialGenerator::Perm GetPermutation(
                     const PassProperties &_props);

      /// Fill a pass with the specific data from the pass it is based on
      protected: void FillPass(Ogre::Pass *_gBufferPass,
                     Ogre::Pass *_originalPass, const PassProperties &_props);

      /// Check if a texture is a normal map, and fill property
      /// sheet accordingly
      protected: bool CheckNormalMap(Ogre::TextureUnitState *_tus,
                                     PassProperties &_props);
    };
  }
}
#endif
