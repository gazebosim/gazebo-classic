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
 * CVS: $Id$
 */

#ifndef OGREADAPTOR
#define OGREADAPTOR

#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include "SingletonT.hh"
#include "Pose3d.hh"

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
  class LogManager;
  class Overlay;
  class OverlayContainer;
  class SceneNode;
  class RenderTarget;
  class ColourValue;
}

namespace gazebo
{
/// \addtogroup gazebo_rendering
/// \{


class XMLConfigNode;
class OgreFrameListener;
class Entity;

/// \brief Adptor to Ogre3d
class OgreAdaptor : public SingletonT<OgreAdaptor>
{
  /// \brief Constructor
  private: OgreAdaptor();

  /// \brief Destructor
  private: virtual ~OgreAdaptor();

  /// \brief Default initialization. 
  ///        Let OGRE create the window and rendering context
  public: void Init(XMLConfigNode *node);

  /// \brief Initialize Ogre Rendering engine
  public: void Init(Display *display, XVisualInfo *visual, Window windowId, int width, int height); 
  /// \brief Render a single frame
  public: int Render();

  /// \brief Create a light source and attach it to the entity
  public: void CreateLight(XMLConfigNode *node, Entity *entity);

  /// \brief Use this function to set the pose of a scene node
  public: void SetSceneNodePose( Ogre::SceneNode *node, const Pose3d &pose );

  /// \brief Use this function to set the position of a scene node
  public: void SetSceneNodePosition(Ogre::SceneNode *node, const Vector3 &pos);

  /// \brief Use this function to set the rotation of a scene node
  public: void SetSceneNodeRotation(Ogre::SceneNode *node, const Quatern &rot);

  /// \brief Helper function to create a camera
  public: Ogre::Camera *CreateCamera(const std::string &name, double nearClip, 
              double farClip, Ogre::RenderTarget *renderTarget);

  /// \brief Resize the rendering window
  public: void ResizeWindow(unsigned int w, unsigned int h);

  private: void LoadPlugins(const std::string &path);
  private: void SetupResources(const std::string &path);
  private: void SetupRenderSystem(bool create);
  private: void CreateWindow();

  public: Ogre::Root *root;
  public: Ogre::SceneManager *sceneMgr;
  public: Ogre::RenderWindow *window;
  public: Ogre::Camera *camera;
  public: Ogre::Viewport *viewport;
  public: Ogre::InputReader *inputDevice;

  private: Ogre::LogManager *logManager;

  // Our custom frame listener
  private: OgreFrameListener *frameListener;

  private: Ogre::ColourValue *backgroundColor;

  private: std::string videoMode;

  private: bool ogreWindow;

  private: Vector3 terrainSize;
  private: unsigned int terrainVertSize;
  private: std::string terrainImage;

  private: friend class DestroyerT<OgreAdaptor>;
  private: friend class SingletonT<OgreAdaptor>;
};

/*/// \brief 
class OgreGLXWindowInterface
{
  public: virtual ~OgreGLXWindowInterface() = 0;

  // Call this with true when the window is mapped/visible, false when the window is unmapped/invisible
  public: virtual void exposed(bool active) = 0;

  // Call this to notify the window was resized
  public: virtual void resized(size_t width, size_t height) = 0;
};
*/

/// \}

}
#endif
