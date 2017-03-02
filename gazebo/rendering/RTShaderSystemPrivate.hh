/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_RTSHADERSYSTEM_PRIVATE_HH_
#define _GAZEBO_RTSHADERSYSTEM_PRIVATE_HH_

#include <string>
#include <vector>

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/gazebo_config.h"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the RTShaderSystem class
    class RTShaderSystemPrivate
    {
#if OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 7
      /// \brief The shader generator.
      public: Ogre::RTShader::ShaderGenerator *shaderGenerator;

      /// \brief Used to generate shadows.
      public: Ogre::RTShader::SubRenderState *shadowRenderState;
#endif

      /// \brief True if initialized.
      public: bool initialized;

      /// \brief True if shadows have been applied.
      public: bool shadowsApplied;

      /// \brief All the scenes.
      public: std::vector<ScenePtr> scenes;

      /// \brief Parallel Split Shadow Map (PSSM) camera setup
      public: Ogre::ShadowCameraSetupPtr pssmSetup;

      /// \brief Flag to indicate that shaders need to be updated.
      public: bool updateShaders;
    };
  }
}
#endif
