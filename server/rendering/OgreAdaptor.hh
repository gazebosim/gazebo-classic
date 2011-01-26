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

#ifndef OGREADAPTOR
#define OGREADAPTOR

#include <iostream>

#include "SingletonT.hh"

namespace Ogre
{
  class Root;
  class SceneManager;
  class RenderWindow;
  class Viewport;
  class InputReader;
  class Window;
  class Camera;
  class SceneNode;
  class Node;
  class LogManager;
  class Overlay;
  class OverlayContainer;
  class SceneNode;
  class RenderTarget;
  class ColourValue;
  class RenderSystem;
  class RaySceneQuery; 
  class Mesh;
}


namespace gazebo
{
  /// \addtogroup gazebo_rendering
  /// \{
  
  
  class XMLConfigNode;
  class OgreFrameListener;
  class Entity;
  class UserCamera;
  class Camera;
  class Visual;
  class Scene;
  
  /// \brief Adptor to Ogre3d
  class OgreAdaptor : public SingletonT<OgreAdaptor>
  {
  
    /// \brief Constructor
    private: OgreAdaptor();
  
    /// \brief Destructor
    private: virtual ~OgreAdaptor();
  
    /// \brief Load the parameters for Ogre
    public: void Load(XMLConfigNode *rootNode);

    /// \brief Initialize ogre
    public: void Init(XMLConfigNode *rootNode);

    /// \brief Finalize
    public: void Fini();
  
    /// \brief Save Ogre settings 
    public: void Save(std::string &prefix, std::ostream &stream);
  
    /// \brief Get the desired update rate
    public: double GetUpdateRate();

    /// \brief Create a scene
    //public: Scene *CreateScene(const std::string &name);

    /// \brief Remove a scene
    //public: void RemoveScene(const std::string &name);

    /// \brief Get a scene manager
    //public: Scene *GetScene(unsigned int index);

    /// \brief Get the number of scene managers
    //public: unsigned int GetSceneCount() const;

    /// \brief Update all the scenes 
    public: void UpdateScenes();

    /// \brief Returns true if the graphics card support GLSL
    public: bool HasGLSL();

    private: void LoadPlugins();
    private: void SetupResources();
    private: void SetupRenderSystem();

    /// Pointer to the root scene node
    public: Ogre::Root *root;
  
    /// Pointer to the scene manager
    //private: std::vector<Scene *> scenes;
  
    /// Pointer to the rendering system
    public: Ogre::RenderSystem *renderSys;
 
    private: Ogre::LogManager *logManager;
 
    /// ID for a dummy window. Used for gui-less operation
    protected: unsigned long dummyWindowId;

    /// Pointer to the dummy display.Used for gui-less operation
    protected: void *dummyDisplay;
    
    /// GLX context used to render the scenes.Used for gui-less operation
    protected: void* dummyContext;

    private: friend class DestroyerT<OgreAdaptor>;
    private: friend class SingletonT<OgreAdaptor>;
  };
  
 
  /// \}

}
#endif
