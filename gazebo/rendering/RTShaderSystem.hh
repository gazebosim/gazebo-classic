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

#ifndef _GAZEBO_RTSHADERSYSTEM_HH_
#define _GAZEBO_RTSHADERSYSTEM_HH_

#include <list>
#include <string>

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/gazebo_config.h"

#include "gazebo/rendering/Camera.hh"
#include "gazebo/common/SingletonT.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    class RTShaderSystemPrivate;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class RTShaderSystem RTShaderSystem.hh rendering/rendering.hh
    /// \brief Implements Ogre's Run-Time Shader system.
    ///
    /// This class allows Gazebo to generate per-pixel shaders for every
    /// material at run-time.
    class GZ_RENDERING_VISIBLE RTShaderSystem :
      public SingletonT<RTShaderSystem>
    {
      /// \enum LightingModel.
      /// \brief The type of lighting.
      public: enum LightingModel
              {
                /// \brief Per-Vertex lighting: best performance.
                SSLM_PerVertexLighting,
                /// \brief Per-Pixel lighting: best look.
                SSLM_PerPixelLighting,
                /// \brief Normal Map lighting: lighting calculations have
                /// been stored in a light map (texture) using tangent space.
                SSLM_NormalMapLightingTangentSpace,
                /// \brief Normal Map lighting: lighting calculations have
                /// been stored in a light map (texture) using object space.
                SSLM_NormalMapLightingObjectSpace
              };

      /// \brief Constructor.
      private: RTShaderSystem();

      /// \brief Destructor.
      private: virtual ~RTShaderSystem();

      /// \brief Init the run time shader system.
      public: void Init();

      /// \brief Finalize the shader system
      public: void Fini();

      /// \brief Clear the shader system
      public: void Clear() GAZEBO_DEPRECATED(7.0);

      /// \brief Add a scene manager
      /// \param[in] _scene The scene to process
      public: void AddScene(ScenePtr _scene);

      /// \brief Remove a scene
      /// \param[in] The scene to remove
      public: void RemoveScene(ScenePtr _scene);

      /// \brief Remove a scene
      /// \param[in] Name of the scene to remove.
      public: void RemoveScene(const std::string &_scene);

      /// \brief Queue a call to update the shaders.
      public: void UpdateShaders();

      /// \brief Set an Ogre::Entity to use RT shaders.
      /// \param[in] _vis Visual that will use the RTShaderSystem.
      /// \deprecated This function is no longer needed, and has no
      /// implementation.
      public: void AttachEntity(Visual *vis) GAZEBO_DEPRECATED(7.0);

      /// \brief Remove and entity.
      /// \param[in] _vis Remove this visual.
      /// \deprecated This function is no longer needed, and has no
      /// implementation.
      public: void DetachEntity(Visual *_vis) GAZEBO_DEPRECATED(7.0);

      /// \brief Set a viewport to use shaders.
      /// \param[in] _viewport The viewport to add.
      /// \param[in] _scene The scene that the viewport uses.
      public: static void AttachViewport(Ogre::Viewport *_viewport,
                                         ScenePtr _scene);

      /// \brief Set a viewport to not use shaders.
      /// \param[in] _viewport The viewport to remove.
      /// \param[in] _scene The scene that the viewport uses.
      public: static void DetachViewport(Ogre::Viewport *_viewport,
                                         ScenePtr _scene);

      /// \brief Set the lighting model to per pixel or per vertex.
      /// \param[in] _set True means to use per-pixel shaders.
      public: void SetPerPixelLighting(bool _set);

      /// \brief Generate shaders for an entity
      /// \param[in] _vis The visual to generate shaders for.
      /// \sa GenerateShaders(const VisualPtr &_vis)
      public: void GenerateShaders(Visual *_vis) GAZEBO_DEPRECATED(7.0);

      /// \brief Generate shaders for an entity
      /// \param[in] _vis The visual to generate shaders for.
      public: void GenerateShaders(const VisualPtr &_vis);

      /// \brief Apply shadows to a scene.
      /// \param[in] _scene The scene to receive shadows.
      public: void ApplyShadows(ScenePtr _scene);

      /// \brief Remove shadows from a scene.
      /// \param[in] _scene The scene to remove shadows from.
      public: void RemoveShadows(ScenePtr _scene);

      /// \brief Get the Ogre PSSM Shadows camera setup.
      /// \return The Ogre PSSM Shadows camera setup.
      public: Ogre::PSSMShadowCameraSetup *GetPSSMShadowCameraSetup() const;

      /// \brief Update the RT shaders. This should not be called frequently.
      public: void Update();

      /// \brief Get paths for the shader system
      /// \param[out] _coreLibsPath Path to the core libraries.
      /// \param[out] _cachePath Path to where the generated shaders are
      /// stored.
      private: bool GetPaths(std::string &_coreLibsPath,
                             std::string &_cachePath);

      /// \brief Update the shaders for a visual.
      /// \param[in] _vis Pointer to the visual to update.
      private: void UpdateShaders(VisualPtr _vis);

      /// \brief Make the RTShader system a singleton.
      private: friend class SingletonT<RTShaderSystem>;

      /// \internal
      /// \brief Pointer to private data.
      private: RTShaderSystemPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
