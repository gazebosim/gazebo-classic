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
#ifndef _MERGEMATERIALGENERATOR_HH_
#define _MERGEMATERIALGENERATOR_HH_

#include "gazebo/rendering/deferred_shading/MaterialGenerator.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// Class for generating materials for objects to render themselves in the
    /// merging step of deferred\inferred lighting. This stage takes the LBuffer
    /// as an input and performs an extra geometry pass. It uses the material
    /// info from the geometry pass, and the light info from the LBuffer to
    /// recreate the Blinn-Phong lighting model. Alternatively, different
    /// lighting models can be recreated(like adding a fresnel term during
    /// merging).
    /// @note This does not support all the possible rendering techniques out
    /// there.  In order to support more, either expand this class or make sure
    /// that objects that will not get treated correctly will not have materials
    /// generated for them.
    class GZ_RENDERING_DEFERRED_VISIBLE MergeMaterialGenerator :
      public MaterialGenerator
    {
      /// The relevant options for materials
      public: enum MaterialPermutations
              {
                // (Regular) Textures
                MP_NO_TEXTURES =      0x00000000,
                MP_ONE_TEXTURE =      0x00000001,
                MP_TWO_TEXTURES =     0x00000002,
                MP_THREE_TEXTURES =   0x00000003,
                MP_TEXTURE_MASK =     0x0000000F,

                // Material properties
                MP_HAS_DIFFUSE_COLOUR =     0x00000010,

                // The number of texture coordinate sets
                MP_NO_TEXCOORDS =      0x00000000,
                MP_ONE_TEXCOORD =      0x00000100,
                MP_TWO_TEXCOORDS =      0x00000200,
                MP_TEXCOORD_MASK =      0x00000700,

                // Do we have a normal map
                MP_NORMAL_MAP =      0x00000800,

                // Are we skinned?
                MP_SKINNED =        0x00010000
              };

      /// The mask of the flags that matter for generating the fragment shader
      public: static const uint32_t FS_MASK =    0x0000FFFF;

      /// The mask of the flags that matter for generating the vertex shader
      public: static const uint32_t VS_MASK =    0x00FFFF00;

      /// The mask of the flags that matter for generating the material
      public: static const uint32_t MAT_MASK =  0xFF00FFFF;

      public: MergeMaterialGenerator(Ogre::String _matName, bool _useDSF);
    };
  }
}

#endif
