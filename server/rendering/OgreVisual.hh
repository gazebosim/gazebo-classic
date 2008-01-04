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
/* Desc: Ogre Visual Class
 * Author: Nate Koenig
 * Date: 14 Dec 2007
 * SVN: $Id$
 */

#ifndef OGREVISUAL_HH
#define OGREVISUAL_HH

#include <Ogre.h>

#include "Pose3d.hh"
#include "Quatern.hh"
#include "Vector3.hh"

namespace gazebo
{ 

  class XMLConfigNode;
  
  /// \brief Ogre Visual Object
  class OgreVisual
  {
    /// \brief Constructor
    public: OgreVisual (Ogre::SceneNode *node);

    /// \brief Destructor
    public: virtual ~OgreVisual();

    /// \brief Attach a renerable object to the visual
    public: void AttachObject( Ogre::MovableObject *obj);

    /// \brief Attach a mesh to this visual by name
    public: void AttachMesh( const std::string &meshName );

    /// \brief Load the visual
    public: void Load(XMLConfigNode *node);

    /// \brief Set the transparency
    public: void SetTransparency( float trans );

    /// \brief Set highlighted or no
    public: void SetHighlight( bool highlight);

    /// \brief Set whether the visual should cast shadows
    public: void SetCastShadows(bool shadows);

    /// \brief Set the material
    public: void SetMaterial(const std::string &materialName);

    /// \brief Set whether the visual is visible
    public: void SetVisible(bool visible);

    /// \brief Set the position of the visual
    public: void SetPosition( const Vector3 &pos);

    /// \brief Set the rotation of the visual
    public: void SetRotation( const Quatern &rot);

    /// \brief Set the pose of the visual
    public: void SetPose( const Pose3d &pose);

    /// \brief Create a bounding box for this visual
    public: void AttachBoundingBox(const Vector3 &min, const Vector3 &max);

    private: Ogre::MaterialPtr origMaterial;
    private: Ogre::MaterialPtr myMaterial;
    private: Ogre::SceneBlendType sceneBlendType;

    private: Ogre::SceneNode *parentNode;
    private: Ogre::SceneNode *sceneNode;
    private: Ogre::SceneNode *boundingBoxNode;

    private: float transparency;
  };
}

#endif
