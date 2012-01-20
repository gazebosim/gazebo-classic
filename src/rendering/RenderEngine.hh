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
/* Desc: Middleman between OGRE and Gazebo
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#ifndef RENDERENGINE_HH
#define RENDERENGINE_HH

#include <vector>
#include <string>

#include "msgs/msgs.h"
#include "common/SingletonT.hh"
#include "common/Event.hh"
#include "transport/TransportTypes.hh"
#include "rendering/RenderTypes.hh"

namespace Ogre
{
  class Root;
  class Node;
  class LogManager;
}


namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{
    /// \brief Adaptor to Ogre3d
    class RenderEngine : public SingletonT<RenderEngine>
    {
      /// \brief Constructor
      private: RenderEngine();

      /// \brief Destructor
      private: virtual ~RenderEngine();

      /// \brief Load the parameters for Ogre
      public: void Load();

      /// \brief Initialize ogre
      public: void Init();

      /// \brief Finalize
      public: void Fini();

      /// \brief Save Ogre settings
      public: void Save(std::string &prefix, std::ostream &stream);

      /// \brief Get the desired update rate
      public: double GetUpdateRate();

      /// \brief Create a scene
      public: ScenePtr CreateScene(const std::string &name,
                                   bool _enableVisualizations);

      /// \brief Remove a scene
      public: void RemoveScene(const std::string &name);

      /// \brief Get a scene
      public: ScenePtr GetScene(const std::string &_name);

      /// \brief Get a scene manager
      public: ScenePtr GetScene(unsigned int index);

      /// \brief Get the number of scene managers
      public: unsigned int GetSceneCount() const;

      /// \brief Returns true if the graphics card support GLSL
      public: bool HasGLSL();

      public: void AddResourcePath(const std::string &_path);

      private: void CreateContext();

      private: void LoadPlugins();

      private: void SetupResources();
      private: void SetupRenderSystem();

      private: void PreRender();
      private: void Render();
      private: void PostRender();

      /// Pointer to the root scene node
      public: Ogre::Root *root;

      /// All of the scenes
      private: std::vector< ScenePtr > scenes;

      private: Ogre::LogManager *logManager;

      /// ID for a dummy window. Used for gui-less operation
      protected: uint64_t dummyWindowId;

      /// Pointer to the dummy display.Used for gui-less operation
      protected: void *dummyDisplay;

      /// GLX context used to render the scenes.Used for gui-less operation
      protected: void* dummyContext;

      /// True if the GUI is enabled
      private: bool headless;

      private: bool initialized;

      private: std::vector<event::ConnectionPtr> connections;
      private: friend class SingletonT<RenderEngine>;

      private: transport::NodePtr node;

      private: bool removeScene;
      private: std::string removeSceneName;
    };
    /// \}
  }
}
#endif


