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

#include <stdint.h>
#include <string>
#include <vector>

#include "Mesh.hh"
#include "OgreDynamicRenderable.hh"
#include "SingletonT.hh"
#include "Vector3.hh"
#include "Vector2.hh"

namespace Ogre
{
  class Camera;
  class RenderTarget;
  class RenderWindow;
}

typedef struct _XDisplay Display;

namespace gazebo
{
  class RenderControl;
  class XMLConfigNode;
  class Entity;
  class OgreVisual;
  class OgreMovableText;
  class OgreDynamicLines;
  class Light;
  class Scene;

/// \addtogroup gazebo_rendering
/// \{


   /// \brief Functions that creates Ogre3d objects
  class OgreCreator : public SingletonT<OgreCreator>
  {

    /// \brief Constructor
    private: OgreCreator();

    /// \brief Destructor
    private: virtual ~OgreCreator();

    /// \brief Create a Plane
    /// It adds itself to the Visual node parent, it will those change parent
    /// properties if needed, to avoid this create a child visual node for the 
    /// plane
    public: static std::string CreatePlane(const Vector3 &normal, 
                const Vector2<double> &size, const Vector2<double> &segments, 
                const Vector2<double> &uvTile, const std::string &material, 
                bool castShadows, OgreVisual *parent, const std::string &name);

    /// \brief Create a new window
    public: Ogre::RenderWindow *CreateWindow(RenderControl *window, 
                                             unsigned int width, 
                                             unsigned int height);


    /// \brief Create a window for Ogre
    public: Ogre::RenderWindow *CreateWindow(std::string ogreHandle, 
                                             unsigned int width, 
                                             unsigned int height);

    /// \brief Insert a mesh into Ogre 
    public: static void InsertMesh( const Mesh *mesh);

   /// \brief Remove a mesh by name
    public: static void RemoveMesh(const std::string &name);

    /// \brief Create a material from a color definition
    /// \return The name of the material
    public: static std::string CreateMaterial(float r, float g, 
                                              float b, float a);

    /// \brief Create a material from a gazebo material
    public: static std::string CreateMaterial(const Material *mat);

    /// \brief Create a material from a texture file
    public: static std::string CreateMaterialFromTexFile(const std::string &filename);

    /// \brief Create a new ogre visual 
    /// \param name Unique name for the new visual. Leave empty to generate
    //              a unique name
    /// \param parent Parent visual
    /// \param owner The entity that owns the visual
    /// \return The new ogre visual
    public: OgreVisual *CreateVisual( const std::string &name,
                                      OgreVisual *parent=NULL, 
                                      Entity *owner = NULL,
                                      Scene *scene = NULL );
    /// \brief Get a visual
    public: OgreVisual *GetVisual( const std::string &name );

    /// \brief Delete a visual
    public: void DeleteVisual( OgreVisual *visual );

    /// \brief Delete a visual
    public: void DeleteVisual( const std::string &visname );

    /// \brief Create a movable text object
    /// \return A new movable text object
    public: OgreMovableText *CreateMovableText();

    /// \brief Update all the entities
    public: void Update();

    // \brief Get the mesh information for the given mesh.
    // Code found in Wiki: www.ogre3d.org/wiki/index.php/RetrieveVertexData
    public: static void GetMeshInformation(const Ogre::MeshPtr mesh,
                                size_t &vertex_count,
                                Ogre::Vector3* &vertices,
                                size_t &index_count,
                                unsigned long* &indices,
                                const Ogre::Vector3 &position,
                                const Ogre::Quaternion &orient,
                                const Ogre::Vector3 &scale);

    /// \brief Get the world bounding box for a visual
    public: static void GetVisualBounds(OgreVisual *vis, Vector3 &min, 
                                        Vector3 &max);

    public: static void GetSceneNodeBounds(Ogre::SceneNode *node, Ogre::AxisAlignedBox &box);

    private: static unsigned int windowCounter;

    // List of all the movable text
    private: std::list<OgreMovableText*> text;

    // All the visuals 
    private: std::list<OgreVisual*> visuals;

    // All the windows
    private: std::list<Ogre::RenderWindow *> windows;

    private: friend class DestroyerT<OgreCreator>;
    private: friend class SingletonT<OgreCreator>;

/// \}

  };
}
#endif
