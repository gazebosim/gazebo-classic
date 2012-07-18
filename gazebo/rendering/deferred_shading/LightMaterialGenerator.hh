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
#ifndef _LIGHTMATERIALGENERATOR_HH_
#define _LIGHTMATERIALGENERATOR_HH_

#include "gazebo/rendering/deferred_shading/MaterialGenerator.hh"

namespace gazebo
{
  namespace rendering
  {
    class LightMaterialGenerator: public MaterialGenerator
    {
      /// Permutation of light materials
      public: enum MaterialID
              {
                MI_POINT = 0x01,
                MI_SPOTLIGHT = 0x02,
                MI_DIRECTIONAL = 0x04,
                MI_ATTENUATED = 0x08,
                MI_SPECULAR = 0x10,
                MI_SHADOW_CASTER = 0x20
              };

      public: LightMaterialGenerator();
      public: virtual ~LightMaterialGenerator();
    };
  }
}
#endif
