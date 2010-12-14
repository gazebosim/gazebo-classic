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

#include <string>

#include "Box.hh"
#include "RenderTypes.hh"
#include "Pose3d.hh"
#include "Quatern.hh"
#include "Vector3.hh"
#include "Common.hh"
#include "Param.hh"

namespace Ogre
{
  class MovableObject;
  class SceneNode;
  class StaticGeometry;
  class RibbonTrail;
  class AxisAlignedBox;
}

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{ 

  class XMLConfigNode;
  class Entity;
  class SelectionObj;
  class Scene;
  class OgreDynamicLines;
  class VisualMsg;
  class Mesh;


  /// \brief Ogre Visual Object
  class Visual : public Common //, public Ogre::Any
  {
    /// \brief Constructor
    public: Visual (const std::string &name, Visual *parent);

    /// \brief Constructor
    public: Visual (const std::string &name, Ogre::SceneNode *parent);

    /// \brief Constructor
    public: Visual (const std::string &name, Scene *scene);

    /// \brief Destructor
    public: virtual ~Visual();

    /// \brief Helper for the contructor
    public: void Init();

    /// \brief Load from a message
    public: void LoadFromMsg(VisualMsg *msg);

    /// \brief Load the visual
    public: void Load(XMLConfigNode *node);

    /// \brief Update the visual.
    public: void Update();

    /// \brief Set the owner
    public: void SetOwner(Common *common);

    /// \brief Get the owner
    public: Common *GetOwner() const;

    /// \brief Attach a visual
    public: void AttachVisual(Visual *vis);
           
    /// \brief Detach a visual 
    public: void DetachVisual(Visual *vis);

    /// \brief Attach a renerable object to the visual
    public: void AttachObject( Ogre::MovableObject *obj);

    /// \brief Detach all objects
    public: void DetachObjects();

    /// \brief Get the number of attached objects
    public: unsigned short GetNumAttached();

    /// \brief Get an attached object
    public: Ogre::MovableObject *GetAttached(unsigned short num);

    /// \brief Attach a mesh to this visual by name
    public: void AttachMesh( const std::string &meshName );

    /// \brief Save the visual
    public: void Save(std::string &prefix, std::ostream &stream);
    
    /// \brief Set the scale
    public: void SetScale( const Vector3 &scale );

    /// \brief Get the scale
    public: Vector3 GetScale();

    /// \brief Set the material
    public: void SetMaterial(const std::string &materialName);

    public: void AttachAxes();

    /// \brief Set the transparency
    public: void SetTransparency( float trans );

    /// \brief Get the transparency
    public: float GetTransparency();

    /// \brief Set highlighted or no
    public: void SetHighlight( bool highlight);

    /// \brief Set whether the visual should cast shadows
    public: void SetCastShadows(const bool &shadows);

    /// \brief Set whether the visual is visible
    /// \param visible set this node visible
    /// \param cascade setting this parameter in children too
    public: void SetVisible(bool visible, bool cascade=true);

    /// \brief Toggle whether this visual is visible
    public: void ToggleVisible();

    /// \brief Get whether the visual is visible
    public: bool GetVisible() const;

    /// \brief Set the position of the visual
    public: void SetPosition( const Vector3 &pos);

    /// \brief Set the rotation of the visual
    public: void SetRotation( const Quatern &rot);

    /// \brief Set the pose of the visual
    public: void SetPose( const Pose3d &pose);

    /// \brief Get the position of the visual
    public: Vector3 GetPosition() const;

    /// \brief Get the rotation of the visual
    public: Quatern GetRotation() const;

    /// \brief Get the pose of the visual
    public: Pose3d GetPose() const;

    /// \brief Get the global pose of the node
    public: Pose3d GetWorldPose() const;

    /// \brief Return the scene Node of this visual entity
    public: Ogre::SceneNode *GetSceneNode() const;

    /// \brief Create a bounding box for this visual
    public: void AttachBoundingBox(const Vector3 &min, const Vector3 &max);

    /// \brief Set the material of the bounding box
    public: void SetBoundingBoxMaterial(const std::string &materialName );

    /// \brief Make the visual objects static renderables
    public: void MakeStatic();

    /// \brief Set to true to show a white bounding box, used to indicate 
    //         user selection
    public: void ShowSelectionBox( bool value );

    /// \brief Return true if the  visual is a static geometry
    public: bool IsStatic() const;

    /// \brief Set one visual to track/follow another
    public: void EnableTrackVisual( Visual *vis );

    /// \brief Disable tracking of a visual
    public: void DisableTrackVisual();

    /// \brief Get the normal map
    public: std::string GetNormalMap() const;

    /// \brief Set the normal map
    public: void SetNormalMap(const std::string &nmap);

    /// \brief Get the shader
    public: std::string GetShader() const;

    /// \brief Set the shader
    public: void SetShader(const std::string &shader);

    /// \brief True on or off a ribbon trail
    public: void SetRibbonTrail(bool value);

    /// \brief Get the size of the bounding box
    public: Vector3 GetBoundingBoxSize() const;

    /// \brief Set whether to use the RT Shader system
    public: void SetUseRTShader(bool value);

    /// \brief Get whether to user the RT shader system
    public: bool GetUseRTShader() const;

    /// \brief Add a line to the visual
    public: OgreDynamicLines *AddDynamicLine(OperationType type);

    /// \brief Delete a dynamic line
    public: void DeleteDynamicLine(OgreDynamicLines *line);

    /// \brief Get the name of the material
    public: std::string GetMaterialName() const;

    /// \brief Get the bounding box for the visual
    public: Box GetBounds() const;

    /// \brief Insert a mesh into Ogre 
    public: static void InsertMesh( const Mesh *mesh);

    private: void GetBoundsHelper(Ogre::SceneNode *node, Ogre::AxisAlignedBox &box) const;

    private: std::string myMaterialName;
    private: std::string origMaterialName;

    private: Ogre::SceneNode *parentNode;
    private: Ogre::SceneNode *sceneNode;
    private: Ogre::SceneNode *boundingBoxNode;

    private: float transparency;

    private: static unsigned int visualCounter;

    private: Common *owner;

    private: ParamT<Vector3> *xyzP;
    private: ParamT<Quatern> *rpyP;
    private: ParamT<std::string> *meshNameP;
    private: ParamT<std::string> *materialNameP;
    private: ParamT<std::string> *normalMapNameP;
    private: ParamT<std::string> *shaderP;
    private: ParamT<bool> *castShadowsP;
    private: ParamT<Vector3> *sizeP;
    private: ParamT<Vector3> *scaleP;
    private: ParamT<Vector2<double> > *meshTileP;

    // NATY
    //private: boost::recursive_mutex *mutex;

    private: bool isStatic;
    private: Ogre::StaticGeometry *staticGeom;
    private: bool visible;

    private: static SelectionObj *selectionObj;

    private: Ogre::RibbonTrail *ribbonTrail;

    private: bool useRTShader;

    // List of all the lines created
    private: std::list<OgreDynamicLines*> lines;
  };
}

#endif
