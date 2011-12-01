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
/* Desc: Wrapper around the OGRE RTShader system
 * Author: Nate Koenig
 * Date: 27 Jan 2010
 */

#ifndef RTSHADERSYSTEM_HH
#define RTSHADERSYSTEM_HH

#include "rendering/ogre.h"

#include <list>
#include "gazebo_config.h"

#include "rendering/Camera.hh"
#include "common/SingletonT.hh"

namespace gazebo
{
	namespace rendering
  {
    class Visual;
    class Scene;
  
    /// \addtogroup gazebo_rendering 
    /// \{

    /// \brief Implements Ogre's Run-Time Shader system
    class RTShaderSystem : public SingletonT<RTShaderSystem>
    {
      public: enum LightingModel
              {
                SSLM_PerVertexLighting,
                SSLM_PerPixelLighting,
                SSLM_NormalMapLightingTangentSpace,
                SSLM_NormalMapLightingObjectSpace
              };
  
      /// \brief Constructor
      private: RTShaderSystem();
  
      /// \brief Destructor
      private: virtual ~RTShaderSystem();
  
      /// \brief Init the run time shader system
      public: void Init();
  
      /// \brief Finalize the shader system
      public: void Fini();
  
      public: void Clear();

      /// \brief Add a scene manager
      public: void AddScene(Scene *_scene);
  
      /// \brief Remove a scene
      public: void RemoveScene( Scene *scene );
  
      /// \brief Update the shaders
      public: void UpdateShaders();
  
      /// \brief Set an Ogre::Entity to use RT shaders
      public: void AttachEntity(Visual *vis);
  
      /// \brief Remove and entity
      public: void DetachEntity(Visual *vis);
  
      /// \brief Set a viewport to use shaders
      public: static void AttachViewport(Ogre::Viewport *viewport, Scene *scene);
  
      /// Set the lighting model to per pixel or per vertex
      public: void SetPerPixelLighting( bool s);
  
      /// \brief Generate shaders for an entity
      public: void GenerateShaders(Visual *vis);
  
      public: void ApplyShadows(Scene *scene);
      public: void RemoveShadows(Scene *_scene);
  
      /// \brief Get paths for the shader system
      private: bool GetPaths(std::string &coreLibsPath, std::string &cachePath);
  
  #if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
      private: Ogre::RTShader::ShaderGenerator *shaderGenerator;
      private: std::list<Visual*> entities;
  #endif
  
      private: bool initialized;
  
      private: std::vector<Scene *> scenes;
      private: Ogre::RTShader::SubRenderState *shadowRenderState;
 
      private: boost::mutex *entityMutex; 
      private: friend class SingletonT<RTShaderSystem>;
    };
    /// \}
  
  }

}
#endif
