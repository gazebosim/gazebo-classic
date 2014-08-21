/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/* Desc: Ogre Visual Class
 * Author: Nate Koenig
 * Date: 14 Dec 2007
 */

#ifndef _VISUAL_HH_
#define _VISUAL_HH_

#include <boost/enable_shared_from_this.hpp>
#include <string>
#include <utility>
#include <vector>

#include <sdf/sdf.hh>

#include "gazebo/common/Color.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Time.hh"

#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/math/Box.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/math/Vector3.hh"

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace Ogre
{
  class MovableObject;
  class SceneNode;
}

namespace gazebo
{
  namespace rendering
  {
    class VisualPrivate;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class Visual Visual.hh rendering/rendering.hh
    /// \brief A renderable object
    class GAZEBO_VISIBLE Visual : public boost::enable_shared_from_this<Visual>
    {
      /// \brief Constructor
      /// \param[in] _name Name of the visual.
      /// \param[in] _parent Parent of the visual.
      /// \param[in] _useRTShader True if the visual should use the
      /// real-time shader system (RTShader).
      public: Visual(const std::string &_name, VisualPtr _parent,
                     bool _useRTShader = true);

      /// \brief Constructor
      /// \param[in] _name Name of the visual.
      /// \param[in] _scene Scene containing the visual.
      /// \param[in] _useRTShader True if the visual should use the
      /// real-time shader system (RTShader).
      public: Visual(const std::string &_name, ScenePtr _scene,
                     bool _useRTShader = true);

      /// \brief Destructor
      public: virtual ~Visual();

      /// \brief Helper for the contructor
      public: void Init();

      /// \brief Helper for the destructor
      public: void Fini();

      /// \brief Clone the visual with a new name.
      /// \param[in] _name Name of the cloned Visual.
      /// \param[in] _newParent Parent of the cloned Visual.
      /// \return The visual.
      public: VisualPtr Clone(const std::string &_name, VisualPtr _newParent);

      /// \brief Load from a message.
      /// \param[in] _msg A visual message.
      public: void LoadFromMsg(ConstVisualPtr &_msg);

      /// \brief Load the visual with a set of parameters.
      /// \param[in] _sdf Load from an SDF element.
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Load the visual with default parameters.
      public: virtual void Load();

      /// \brief Update the visual.
      public: void Update();

      /// \brief Set the name of the visual
      /// \param[in] _name Name of the visual
      public: void SetName(const std::string &_name);

      /// \brief Get the name of the visual.
      /// \return The name of the visual.
      public: std::string GetName() const;

      /// \brief Attach a visual to this visual.
      /// \param[in] _vis Visual to attach.
      public: void AttachVisual(VisualPtr _vis);

      /// \brief Detach a visual.
      /// \param[in] _vis Visual to detach.
      public: void DetachVisual(VisualPtr _vis);

      /// \brief Detach a visual.
      /// \param[in] _name Name of the visual to detach.
      public: void DetachVisual(const std::string &_name);

      /// \brief Attach a renerable object to the visual.
      /// \param[in] _obj A movable object to attach to the visual.
      public: void AttachObject(Ogre::MovableObject *_obj);

      /// \brief Returns true if an object with _name is attached
      /// \param[in] _name Name of an object to find.
      public: bool HasAttachedObject(const std::string &_name);

      /// \brief Return the number of attached movable objects.
      /// \return The number of attached movable objects.
      public: unsigned int GetAttachedObjectCount() const;

      /// \brief Detach all objects.
      public: void DetachObjects();

      /// \brief Get the number of attached visuals.
      /// \return The number of children.
      public: unsigned int GetChildCount();

      /// \brief Get an attached visual based on an index. Index should be
      /// between 0 and Visual::GetChildCount.
      /// \param[in] _index Index of the child to retreive.
      /// \return Pointer to the child visual, NULL if index is invalid.
      public: VisualPtr GetChild(unsigned int _index);

      /// \brief Attach a mesh to this visual by name.
      /// \param[in] _meshName Name of the mesh.
      /// \param[in] _subMesh Name of the submesh. Empty string to use all
      /// submeshes.
      /// \param[in] _centerSubmesh True to center a submesh.
      /// \param[in] _objName Name of the attached Object to put the mesh
      /// onto.
      public: Ogre::MovableObject *AttachMesh(const std::string &_meshName,
                  const std::string &_subMesh="",
                  bool _centerSubmesh = false,
                  const std::string &_objName="");

      /// \brief Set the scale.
      /// \param[in] _scale The scaling factor for the visual.
      public: void SetScale(const math::Vector3 &_scale);

      /// \brief Get the scale.
      /// \return The scaling factor.
      public: math::Vector3 GetScale();

      /// \brief Set whether or not to enable or disable lighting.
      /// \param[in] _lighting True to enable lighting.
      public: void SetLighting(bool _lighting);

      /// \brief Set the material.
      /// \param[in] _materialName The name of the material.
      /// \param[in] _unique True to make the material unique, which
      /// allows the material to change without changing materials that
      /// originally had the same name.
      public: void SetMaterial(const std::string &_materialName,
                               bool _unique = true);

      /// \brief Set the ambient color of the visual.
      /// \param[in] _color The ambient color.
      public: void SetAmbient(const common::Color &_color);

      /// \brief Set the diffuse color of the visual.
      /// \param[in] _color Set the diffuse color.
      public: void SetDiffuse(const common::Color &_color);

      /// \brief Set the specular color of the visual.
      /// \param[in] _color Specular color.
      public: void SetSpecular(const common::Color &_color);

      /// \brief Attach visualization axes
      public: void AttachAxes();

      /// \brief Enable or disable wireframe for this visual.
      /// \param[in] _show True to enable wireframe for this visual.
      public: void SetWireframe(bool _show);

      /// \brief Set the transparency.
      /// \param[in] _trans The transparency, between 0 and 1 where 0 is no
      /// transparency.
      public: void SetTransparency(float _trans);

      /// \brief Get the transparency.
      /// \return The transparency.
      public: float GetTransparency();

      /// \brief Set the visual to be visually highlighted. This is most
      /// often used when an object is selected by a user via the GUI.
      /// \param[in] _highlighted True to enable the highlighting.
      public: void SetHighlighted(bool _highlighted);

      /// \brief Get whether or not the visual is visually highlighted. This is
      /// most often means that an object is selected by a user via the GUI.
      /// \return True if the visual is highlighted.
      public: bool GetHighlighted() const;

      /// \brief Set the emissive value.
      /// \param[in] _color The emissive color.
      public: virtual void SetEmissive(const common::Color &_color);

      /// \brief Set whether the visual should cast shadows.
      /// \param[in] _shadows True to enable shadows.
      public: void SetCastShadows(bool _shadows);

      /// \brief Set whether the visual is visible.
      /// \param[in] _visible set this node visible.
      /// \param[in] _cascade setting this parameter in children too.
      public: void SetVisible(bool _visible, bool _cascade = true);

      /// \brief Toggle whether this visual is visible.
      public: void ToggleVisible();

      /// \brief Get whether the visual is visible.
      /// \return True if the visual is visible.
      public: bool GetVisible() const;

      /// \brief Set the position of the visual.
      /// \param[in] _pos The position to set the visual to.
      public: void SetPosition(const math::Vector3 &_pos);

      /// \brief Set the rotation of the visual.
      /// \param[in] _rot The rotation of the visual.
      public: void SetRotation(const math::Quaternion &_rot);

      /// \brief Set the pose of the visual.
      /// \param[in] _pose The new pose of the visual.
      public: void SetPose(const math::Pose &_pose);

      /// \brief Get the position of the visual.
      /// \return The visual's position.
      public: math::Vector3 GetPosition() const;

      /// \brief Get the rotation of the visual.
      /// \return The visual's rotation.
      public: math::Quaternion GetRotation() const;

      /// \brief Get the pose of the visual.
      /// \return The Visual's pose.
      public: math::Pose GetPose() const;

      /// \brief Get the global pose of the node.
      /// \return The pose in the world coordinate frame.
      public: math::Pose GetWorldPose() const;

      /// \brief Set the world pose of the visual.
      /// \param[in] _pose Pose of the visual in the world coordinate frame.
      public: void SetWorldPose(const math::Pose &_pose);

      /// \brief Set the world linear position of the visual.
      /// \param[in] _pose Position in the world coordinate frame.
      public: void SetWorldPosition(const math::Vector3 &_pos);

      /// \brief Set the world orientation of the visual
      /// \param[in] _rot Rotation in the world coordinate frame.
      public: void SetWorldRotation(const math::Quaternion &_rot);

      /// \brief Return the scene Node of this visual entity.
      /// \return The Ogre scene node.
      public: Ogre::SceneNode *GetSceneNode() const;

      /// \brief Make the visual objects static renderables.
      public: void MakeStatic();

      /// \brief Return true if the  visual is a static geometry.
      /// \return True if the visual is static.
      public: bool IsStatic() const;

      /// \brief Set one visual to track/follow another.
      /// \param[in] _vis Visual to track.
      public: void EnableTrackVisual(VisualPtr _vis);

      /// \brief Disable tracking of a visual.
      public: void DisableTrackVisual();

      /// \brief Get the normal map.
      /// \return The name of the normal map material.
      public: std::string GetNormalMap() const;

      /// \brief Set the normal map.
      /// \param[in] _nmap Name of the normal map material.
      public: void SetNormalMap(const std::string &_nmap);

      /// \brief True on or off a ribbon trail.
      /// \param[in] _value True to enable ribbon trail.
      /// \param[in] _initialColor The initial color of the ribbon trail.
      /// \param[in] _changeColor Color to change too as the trail grows.
      public: void SetRibbonTrail(bool _value,
                  const common::Color &_initialColor,
                  const common::Color &_changeColor);

      /// \brief Get the bounding box for the visual.
      /// \return The bounding box in world coordinates.
      public: math::Box GetBoundingBox() const;

      /// \brief Add a line to the visual.
      /// \param[in] _type The type of line to make.
      /// \return A pointer to the new dynamic line.
      public: DynamicLines *CreateDynamicLine(
                  RenderOpType _type = RENDERING_LINE_STRIP);

      /// \brief Delete a dynamic line.
      /// \param[in] _line Pointer to the line to delete.
      public: void DeleteDynamicLine(DynamicLines *_line);

      /// \brief Attach a vertex of a line to the position of the visual.
      /// \param[in] _line Line to attach to this visual.
      /// \param[in] _index Index of the line vertex to attach.
      public: void AttachLineVertex(DynamicLines *_line,
                                     unsigned int _index);

      /// \brief Get the name of the material.
      /// \return The name of the visual applied to this visual.
      public: std::string GetMaterialName() const;

      /// \brief Insert a mesh into Ogre.
      /// \param[in] _meshName Name of the mesh to insert.
      /// \param[in] _subMesh Name of the mesh within _meshName to insert.
      /// \param[in] _centerSubmesh True to center the submesh.
      public: void InsertMesh(const std::string &_meshName,
                  const std::string &_subMesh = "",
                  bool _centerSubmesh = false);

      /// \brief Insert a mesh into Ogre.
      /// \param[in] _mesh Pointer to the mesh to insert.
      /// \param[in] _subMesh Name of the mesh within _meshName to insert.
      /// \param[in] _centerSubmesh True to center the submesh.
      public: static void InsertMesh(const common::Mesh *_mesh,
                  const std::string &_subMesh = "",
                  bool _centerSubmesh = false);

      /// \brief Update a visual based on a message.
      /// \param[in] _msg The visual message.
      public: void UpdateFromMsg(ConstVisualPtr &_msg);

      /// \brief Return true if the visual is a plane
      /// \return True if a plane.
      public: bool IsPlane() const;

      /// \brief Get the parent visual, if one exists.
      /// \return Pointer to the parent visual, NULL if no parent.
      public: VisualPtr GetParent() const;

      /// \brief Get the root visual
      /// \return The root visual, which is one level below the world
      /// visual.
      public: VisualPtr GetRootVisual();

      /// \brief Get the shader type.
      /// \return String of the shader type: "vertex", "pixel",
      /// "normal_map_object_space", "normal_map_tangent_space".
      public: std::string GetShaderType() const;

      /// \brief Set the shader type for the visual's material.
      /// \param[in] _type Shader type string: "vertex", "pixel",
      /// "normal_map_object_space", "normal_map_tangent_space".
      public: void SetShaderType(const std::string &_type);

      /// \brief Move to a pose and over a given time.
      /// \param[in] _pose Pose the visual will end at.
      /// \param[in] _time Time it takes the visual to move to the pose.
      public: void MoveToPosition(const math::Pose &_pose, double _time);

      /// \brief Move to a series of pose and over a given time.
      /// \param[in] _poses Series of poses the visual will move to.
      /// \param[in] _time Time it takes the visual to move to the pose.
      /// \param[in] _onComplete Callback used when the move is complete.
      public: void MoveToPositions(const std::vector<math::Pose> &_pts,
                                   double _time,
                                   boost::function<void()> _onComplete = NULL);

      /// \brief Set visibility flags for this visual and all children.
      /// \param[in] _flags The visiblity flags.
      /// \sa GZ_VISIBILITY_ALL
      /// \sa GZ_VISIBILITY_GUI
      /// \sa GZ_VISIBILITY_SELECTABLE
      public: void SetVisibilityFlags(uint32_t _flags);

      /// \brief Get visibility flags for this visual and all children.
      /// \return The visiblity flags.
      /// \sa GZ_VISIBILITY_ALL
      /// \sa GZ_VISIBILITY_GUI
      /// \sa GZ_VISIBILITY_SELECTABLE
      public: uint32_t GetVisibilityFlags();

      /// \brief Display the bounding box visual.
      public: void ShowBoundingBox();

      /// \brief Display the collision visuals
      /// \param[in] _show True to show visuals labeled as collision
      /// objects.
      public: void ShowCollision(bool _show);

      /// \brief Display the skeleton visuals.
      /// \param[in] _show True to show skeleton visuals.
      public: void ShowSkeleton(bool _show);

      /// \brief Set current scene.
      /// \param[in] _scene Pointer to the scene.
      public: void SetScene(ScenePtr _scene);

      /// \brief Get current.
      /// \return Pointer to the scene.
      public: ScenePtr GetScene() const;

      /// \brief Display joint visuals.
      /// \param[in] _show True to show joint visualizations.
      public: void ShowJoints(bool _show);

      /// \brief Display Center of Mass visuals.
      /// \param[in] _show True to show center of mass visualizations.
      public: void ShowCOM(bool _show);

      /// \brief Set animation skeleton pose.
      /// \param[in] _pose Skelton message
      public: void SetSkeletonPose(const msgs::PoseAnimation &_pose);

      /// \brief Load a plugin
      /// \param _filename The filename of the plugin
      /// \param _name A unique name for the plugin
      /// \param _sdf The SDF to pass into the plugin.
      public: void LoadPlugin(const std::string &_filename,
                               const std::string &_name,
                               sdf::ElementPtr _sdf);

      /// \brief Remove a running plugin
      /// \param _name The unique name of the plugin to remove
      public: void RemovePlugin(const std::string &_name);

      /// \brief Get the id associated with this visual
      public: uint32_t GetId() const;

      /// \brief Set the id associated with this visual
      public: void SetId(uint32_t _id);

      /// \brief The name of the mesh set in the visual's SDF.
      /// \return Name of the mesh.
      public: std::string GetMeshName() const;

      /// \brief Get the name of the sub mesh set in the visual's SDF.
      /// \return Name of the submesh. Empty string if no submesh is
      /// specified.
      public: std::string GetSubMeshName() const;

      /// \brief Clear parents.
      public: void ClearParent();

      /// \internal
      /// \brief Constructor used by inherited classes
      /// \param[in] _dataPtr Pointer to private data.
      /// \param[in] _name Name of the visual.
      /// \param[in] _parent Parent of the visual.
      /// \param[in] _useRTShader True if the visual should use the
      /// real-time shader system (RTShader).
      protected: Visual(VisualPrivate &_dataPtr,
                        const std::string &_name, VisualPtr _parent,
                        bool _useRTShader = true);

      /// \internal
      /// \brief Constructor used by inherited classes
      /// \param[in] _dataPtr Pointer to private data.
      /// \param[in] _name Name of the visual.
      /// \param[in] _scene Scene containing the visual.
      /// \param[in] _useRTShader True if the visual should use the
      /// real-time shader system (RTShader).
      protected: Visual(VisualPrivate &_dataPtr,
                        const std::string &_name, ScenePtr _scene,
                        bool _useRTShader = true);

      /// \brief Helper function for initializing the visual with a scene as
      /// its parent.
      /// \param[in] _name Name of the visual.
      /// \param[in] _scene Scene containing the visual.
      /// \param[in] _useRTShader True if the visual should use the
      /// real-time shader system (RTShader).
      private: void Init(const std::string &_name, ScenePtr _scene,
          bool _useRTShader);

      /// \brief Helper function for initializing the visual with a visual as
      /// its parent.
      /// \param[in] _name Name of the visual.
      /// \param[in] _scene Scene containing the visual.
      /// \param[in] _useRTShader True if the visual should use the
      /// real-time shader system (RTShader).
      private: void Init(const std::string &_name, VisualPtr _parent,
          bool _useRTShader);

      /// \brief Load all plugins
      /// Load all plugins specified in the SDF for the model.
      private: void LoadPlugins();

      private: void LoadPlugin(sdf::ElementPtr _sdf);

      /// \brief Helper function to get the bounding box for a visual.
      /// \param[in] _node Pointer to the Ogre Node to process.
      /// \param[in] _box Current bounding box information.
      private: void GetBoundsHelper(Ogre::SceneNode *_node,
                                    math::Box &_box) const;

      /// \brief Return true if the submesh should be centered.
      /// \return True if the submesh should be centered when it's inserted
      /// into OGRE.
      private: bool GetCenterSubMesh() const;

      /// \brief Destroy all the movable objects attached to a scene node.
      /// \param[in] _sceneNode Pointer to the scene node to process.
      private: void DestroyAllAttachedMovableObjects(
                   Ogre::SceneNode *_sceneNode);

      /// \brief Helper function to update the geometry object size based on
      /// the scale of the visual.
      /// \param[in] _scale Scale of visual
      private: void UpdateGeomSize(const math::Vector3 &_scale);

      /// \internal
      /// \brief Pointer to private data.
      protected: VisualPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
