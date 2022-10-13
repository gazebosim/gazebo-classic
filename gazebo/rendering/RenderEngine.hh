/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef _GAZEBO_RENDERING_RENDERENGINE_HH_
#define _GAZEBO_RENDERING_RENDERENGINE_HH_

#include <memory>
#include <string>
#include <vector>

#include "gazebo/common/SingletonT.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

/// \brief Explicit instantiation for typed SingletonT.
GZ_SINGLETON_DECLARE(GZ_RENDERING_VISIBLE, gazebo, rendering, RenderEngine)

namespace Ogre
{
  class Root;
  class OverlaySystem;
}

namespace gazebo
{
  /// \ingroup gazebo_rendering
  /// \brief Rendering namespace
  namespace rendering
  {
    // Forward declare private data.
    class RenderEnginePrivate;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class RenderEngine RenderEngine.hh rendering/rendering.hh
    /// \brief Adaptor to Ogre3d
    ///
    /// Provides the interface to load, initialize the rendering engine.
    class GZ_RENDERING_VISIBLE RenderEngine : public SingletonT<RenderEngine>
    {
      /// \enum RenderPathType
      /// \brief The type of rendering path used by the rendering engine.
      public: enum RenderPathType
              {
                /// \brief No rendering is done.
                NONE = 0,
                /// \brief Most basic rendering, with least fidelity.
                VERTEX = 1,
                /// \brief Utilizes the RTT shader system.
                FORWARD = 2,
                /// \brief Utilizes deferred rendering. Best fidelity.
                DEFERRED = 3,
                /// \brief Count of the rendering path enums.
                RENDER_PATH_COUNT
              };

      /// \brief Constructor. This is a singleton, use
      /// RenderEngine::Instance() to access the render engine.
      private: RenderEngine();

      /// \brief Destructor
      private: virtual ~RenderEngine();

      /// \brief Load the parameters for Ogre. Load must happen before Init.
      public: void Load();

      /// \brief Initialize Ogre. Load must happen before Init.
      public: void Init();

      /// \brief Tears down the rendering engine
      public: void Fini();

      /// \brief Create a scene
      /// \param[in] _name The name of the scene.
      /// \param[in] _enableVisualizations True enables visualization
      /// elements such as laser lines.
      public: ScenePtr CreateScene(const std::string &_name,
                                   bool _enableVisualizations,
                                   bool _isServer = false);

      /// \brief Remove a scene
      /// \param[in] _name The name of the scene to remove.
      public: void RemoveScene(const std::string &_name);

      /// \brief Get a scene by name
      /// \param[in] _name Name of the scene to retreive.
      /// \return A pointer to the Scene, or NULL if the scene doesn't
      /// exist.
      public: ScenePtr GetScene(const std::string &_name="");

      /// \brief Get a scene by index. The index should be between 0 and
      /// GetSceneCount().
      /// \param[in] _index The index of the scene.
      /// \return A pointer to a Scene, or NULL if the index was invalid.
      public: ScenePtr GetScene(unsigned int _index);

      /// \brief Get the number of scenes.
      /// \return The number of scenes created by the RenderEngine.
      public: unsigned int SceneCount() const;

      /// \brief Add a new path for Ogre to search for resources.
      /// \param[in] _uri URI of the path. The uri should be of the form
      /// file:// or model://
      public: void AddResourcePath(const std::string &_uri);

      /// \brief Get the type of rendering path to use. This is
      /// automatically determined based on the computers capabilities
      /// \return The RenderPathType
      public: RenderPathType GetRenderPathType() const;

      /// \brief Get a pointer to the window manager.
      /// \return Pointer to the window manager.
      public: WindowManagerPtr GetWindowManager() const;

      /// \brief Get a pointer to the Ogre root object.
      /// \return Pointer to the Ogre root object.
      public: Ogre::Root *Root() const;

      /// \brief Get a list of all supported FSAA levels for this render system
      /// \return a list of FSAA levels
      public: std::vector<unsigned int> FSAALevels() const;

#if OGRE_VERSION_MAJOR > 1 || OGRE_VERSION_MINOR >= 9
      /// \internal
      /// \brief Get a pointer to the Ogre overlay system.
      /// \return Pointer to the OGRE overlay system.
      public: Ogre::OverlaySystem *OverlaySystem() const;
#endif

      /// \brief Create a render context.
      /// \return True if the context was created.
      private: bool CreateContext();

      /// \brief Load all OGRE plugins.
      private: void LoadPlugins();

      /// \brief Setup initial resource paths.
      private: void SetupResources();

      /// \brief Setup the render system.
      private: void SetupRenderSystem();

      /// \brief Execute prerender on all scenes
      private: void PreRender();

      /// \brief Execute render on all scenes
      private: void Render();

      /// \brief Execute post-render on all scenes
      private: void PostRender();

      /// \brief Check the rendering capabilities of the system.
      private: void CheckSystemCapabilities();

      /// \brief Returns a pointer to the unique (static) instance
      public: static RenderEngine* Instance();

      /// \brief ID for a dummy window. Used for gui-less operation
      protected: uint64_t dummyWindowId;

      /// \brief Pointer to the dummy display.Used for gui-less operation
      protected: void *dummyDisplay;

      /// \brief GLX context used to render the scenes.Used for gui-less
      /// operation.
      protected: void *dummyContext;

      /// \brief Makes this class a singleton.
      private: friend class SingletonT<RenderEngine>;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<RenderEnginePrivate> dataPtr;
    };
    /// \}
  }
}
#endif
