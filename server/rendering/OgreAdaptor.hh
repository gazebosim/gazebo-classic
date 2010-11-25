/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Middleman between OGRE and Gazebo
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#ifndef OGREADAPTOR
#define OGREADAPTOR

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
