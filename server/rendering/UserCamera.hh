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
/* Desc: Camera for viewing the world
 * Author: Nate Koenig
 * Date: 19 Jun 2008
 * SVN: $Id$
 */

#ifndef USERCAMERA_HH
#define USERCAMERA_HH

#include "XMLConfig.hh"
#include "OgreCamera.hh"

namespace Ogre
{
  class RenderWindow;
}

namespace gazebo
{
  class GLWindow;
  class XMLConfigNode;

  class UserCamera : public OgreCamera
  {
    /// \brief Constructor
    public: UserCamera( GLWindow *parentWindow);

    /// \brief Destructor
    public: virtual ~UserCamera();

    /// \brief Load child
    public: void Load( XMLConfigNode *node );

    /// \brief Initialize
    public: void Init();

    /// \brief Update
    public: void Update();

    /// \brief Finialize
    public: void Fini();
  
    /// \brief Get the name of the camera
    public: std::string GetName() const;

    /// \brief Resize the camera
    public: void Resize(unsigned int w, unsigned int h);

    /// \brief Set the dimensions of the viewport
    public: void SetViewportDimensions(float x, float y, float w, float h);

    /// \brief Get the average FPS
    public: virtual float GetAvgFPS();

    /// Pointer to the viewport
    protected: Ogre::Viewport *viewport;

    /// Pointer to the render window
    private: Ogre::RenderWindow *window;

    private: std::string name;
    private: static unsigned int cameraCount;
    private: static int count;
  };
}

#endif
