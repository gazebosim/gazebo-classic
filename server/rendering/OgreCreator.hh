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
/* Desc: Some functions that creates Ogre objects together
 * Author: Jordi Polo	
 * Date: 27 Dec 2007
 */

#ifndef OGRECREATOR
#define OGRECREATOR
#include <string>

//#include <Ogre.h>

namespace Ogre
{
  class Camera;
  class RenderTarget;
}

namespace gazebo
{
  class XMLConfigNode;
  class Entity;
  class OgreVisual;

/// \addtogroup gazebo_rendering
/// \{


/// \brief Functions that creates Ogre3d objects
  class OgreCreator
  {

    /// \brief Constructor
    public: OgreCreator();

    /// \brief Destructor
    public: virtual ~OgreCreator();

    /// \brief Create some simple general shapes
    public: static void CreateBasicShapes();

    ///\brief Helper function to create a Plane
    public: static OgreVisual *CreatePlane(XMLConfigNode *node, Entity *parent);

    /// \brief Create a light source and attach it to the entity
    public: static void CreateLight(XMLConfigNode *node, Entity *entity);

    /// \brief Helper function to create a camera
    public: static Ogre::Camera *CreateCamera(const std::string &name, double nearClip, 
              double farClip, Ogre::RenderTarget *renderTarget);

    /// \brief Helper function to create fog
    public: static void CreateFog(XMLConfigNode *node);

    /// \brief Helper function to save the fog settings
    public: static void SaveFog(XMLConfigNode *node);

    /// \brief Helper function to create the sky 
    public: static void CreateSky(XMLConfigNode *node);
    
    public: static void DrawGrid();


/// \}

  };
}
#endif
