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
 * SVN: $Id$
 */

#ifndef OGREADAPTOR
#define OGREADAPTOR

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/glx.h>

#include "Vector4.hh"
#include "Param.hh"
#include "Vector2.hh"
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
}


namespace gazebo
{
  /// \addtogroup gazebo_rendering
  /// \{
  
  
  class XMLConfigNode;
  class OgreFrameListener;
  class Entity;
  class UserCamera;
  class OgreCamera;
  
  /// \brief Adptor to Ogre3d
  class OgreAdaptor : public SingletonT<OgreAdaptor>
  {
  
    /// \brief Constructor
    private: OgreAdaptor();
  
    /// \brief Destructor
    private: virtual ~OgreAdaptor();
  
    /// \brief Closes the present simulation, frees the resources 
    public: void Close();

    /// \brief Load the parameters for Ogre
    public: void Load(XMLConfigNode *rootNode);

    /// \brief Initialize ogre
    public: void Init(XMLConfigNode *rootNode);
  
    /// \brief Save Ogre settings 
    public: void Save(std::string &prefix, std::ostream &stream);
  
    /// \brief Get the desired update rate
    public: double GetUpdateRate();

    /// \brief Update all the cameras 
    public: void UpdateCameras();

    /// \brief Get an entity at a pixel location using a camera. Used for
    ///        mouse picking. 
    /// \param camera The ogre camera, used to do mouse picking
    /// \param mousePos The position of the mouse in screen coordinates
    /// \return The selected entity, or NULL
    public: Entity *GetEntityAt(OgreCamera *camera, Vector2<int> mousePos);

    /// \brief Register a user camera
    public: void RegisterCamera( OgreCamera *cam );

    private: void LoadPlugins();
    private: void SetupResources();
    private: void SetupRenderSystem();
  
    /// Pointer to the root scene node
    public: Ogre::Root *root;
  
    /// Pointer to the scene manager
    public: Ogre::SceneManager *sceneMgr;
  
    /// Pointer to the rendering system
    public: Ogre::RenderSystem *renderSys;
 
    private: Ogre::LogManager *logManager;
  
    // Our custom frame listener
    private: OgreFrameListener *frameListener;
  
    public: Ogre::ColourValue *backgroundColor;
  
    private: Ogre::RaySceneQuery *raySceneQuery;

    //bsp attributes saved to write XML file back
    private: int sceneType;
    private: std::string worldGeometry;
  
    //private: Vector3 terrainSize;
    //private: unsigned int terrainVertSize;
    //private: std::string terrainImage;
  
    private: friend class DestroyerT<OgreAdaptor>;
    private: friend class SingletonT<OgreAdaptor>;

    /// ID for a dummy window. Used for gui-less operation
    protected: Window dummyWindowId;

    /// Pointer to the dummy Xvisual.Used for gui-less operation
    protected: XVisualInfo *dummyVisual;

    /// Pointer to the dummy display.Used for gui-less operation
    protected: Display *dummyDisplay;
    
    /// GLX context used to render the scenes.Used for gui-less operation
    protected: GLXContext dummyContext;

    private: ParamT<Vector4> *ambientP;
    private: ParamT<std::string> *shadowTechniqueP;
    private: ParamT<int> *shadowTextureSizeP;
    private: ParamT<int> *shadowIndexSizeP;
    private: ParamT<bool> *drawGridP;
    private: ParamT<std::string> *skyMaterialP;
    private: std::vector<Param*> parameters;

    private: std::vector<OgreCamera*> cameras;
  };
  
 
  /// \}

}
#endif
