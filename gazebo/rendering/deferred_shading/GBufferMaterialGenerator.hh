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
#ifndef _GBUFFER_MATERIAL_GENERATOR_HH_
#define _GBUFFER_MATERIAL_GENERATOR_HH_

#include "gazebo/rendering/deferred_shading/MaterialGenerator.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Class for generating materials for objects to render themselves
    /// to the GBuffer
    /// @note This does not support all the possible rendering techniques out
    /// there. in order to support more, either expand this class or make sure
    /// that objects that will not get treated correctly will not have
    /// materials generated for them.
    class GZ_RENDERING_DEFERRED_VISIBLE GBufferMaterialGenerator : public MaterialGenerator
    {
      /// \brief The types of g buffers
      public: enum GBufferType
              {
                GBT_NORMAL_AND_DEPTH,
                GBT_DSF,
                GBT_FAT
              };

      /// \brief The relevant options for objects that are rendered to the
      ///        GBuffer
      public: enum GBufferPermutations
        {
          // (Regular) Textures
          GBP_NO_TEXTURES =    0x00000000,
          GBP_ONE_TEXTURE =    0x00000001,
          GBP_TWO_TEXTURES =   0x00000002,
          GBP_THREE_TEXTURES = 0x00000003,
          GBP_TEXTURE_MASK =   0x0000000F,

          // Material properties
          GBP_HAS_DIFFUSE_COLOUR = 0x00000010,

          // The number of texture coordinate sets
          GBP_NO_TEXCOORDS =  0x00000000,
          GBP_ONE_TEXCOORD =  0x00000100,
          GBP_TWO_TEXCOORDS = 0x00000200,
          GBP_TEXCOORD_MASK = 0x00000700,

          // Do we have a normal map
          GBP_NORMAL_MAP = 0x00000800,

          // Are we skinned?
          GBP_SKINNED = 0x00010000
        };

      /// \brief Constructor
      public: GBufferMaterialGenerator(GBufferType _type);

      // The mask of the flags that matter for generating the fragment shader
      public: static const uint32_t FS_MASK = 0x0000FFFF;

      // The mask of the flags that matter for generating the vertex shader
      public: static const uint32_t VS_MASK = 0x00FFFF00;

      // The mask of the flags that matter for generating the material
      public: static const uint32_t MAT_MASK = 0xFF00FFFF;
    };
  }
}

#endif
