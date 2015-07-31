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
#ifndef _GBUFFERSCHEMEHANDLER_HH_
#define _GBUFFERSCHEMEHANDLER_HH_
#include <OgreMaterialManager.h>

#include <vector>
#include <string>

#include "gazebo/rendering/deferred_shading/GBufferMaterialGenerator.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Class for handling materials who did not specify techniques
    /// for rendering themselves into the GBuffer. This class allows deferred
    /// shading to be used, without having to specify new techniques for all
    /// the objects in the scene.
    /// @note This does not support all the possible rendering techniques
    /// out there. in order to support more, either expand this class or
    /// specify the techniques in the materials.
    class GZ_RENDERING_VISIBLE GBufferSchemeHandler :
      public Ogre::MaterialManager::Listener
    {
      public: GBufferSchemeHandler(GBufferMaterialGenerator::GBufferType _type)
              : type(_type), materialGenerator(_type) {}

      public: virtual Ogre::Technique* handleSchemeNotFound(
                  uint16_t _schemeIndex,
                  const Ogre::String &_schemeName,
                  Ogre::Material *_originalMaterial,
                  uint16_t _lodIndex,
                  const Ogre::Renderable *_rend);

      /// \brief A structure for containing the properties of a material,
      ///        relevant to GBuffer rendering.
      ///        You might need to expand this class to support more options
      protected: struct PassProperties
                 {
                   PassProperties() : isDeferred(true), normalMap(0),
                                      isSkinned(false), hasDiffuseColor(true)
                   {}

                   bool isDeferred;
                   std::vector<Ogre::TextureUnitState*> regularTextures;
                   Ogre::TextureUnitState *normalMap;
                   bool isSkinned;
                   bool hasDiffuseColor;

                   // Example of possible extension : vertex colours
                   // Ogre::TrackVertexColourType vertexColorType;
                 };

      /// \brief Inspect a technique and return its relevant properties
      protected: PassProperties InspectPass(Ogre::Pass *_pass,
                  uint16_t _lodIndex, const Ogre::Renderable *_rend);

      /// \brief Get the permutation of material flags that fit a certain
      ///        property sheet
      protected: MaterialGenerator::Perm GetPermutation(
                     const PassProperties &_props);

      /// \brief Fill a pass with the specific data from the pass it is based on
      protected: void FillPass(Ogre::Pass *_gBufferPass,
                               Ogre::Pass *_originalPass,
                               const PassProperties &_props);

      /// \brief Check if a texture is a normal map, and fill property sheet
      ///        accordingly
      protected: bool CheckNormalMap(Ogre::TextureUnitState *_tus,
                                     PassProperties &_props);

      protected: GBufferMaterialGenerator::GBufferType type;

      /// \brief The material generator
      protected: GBufferMaterialGenerator materialGenerator;

      // The string that will be checked in textures to determine whether
      // they are normal maps
      protected: static const std::string normal_map_pattern;
    };
  }
}
#endif
