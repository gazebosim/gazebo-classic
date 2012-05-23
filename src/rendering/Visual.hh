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
/* Desc: Ogre Visual Class
 * Author: Nate Koenig
 * Date: 14 Dec 2007
 */

#ifndef VISUAL_HH
#define VISUAL_HH

#include <boost/enable_shared_from_this.hpp>
#include <string>
#include <utility>
#include <list>
#include <vector>

#include "common/Event.hh"
#include "math/Box.hh"
#include "math/Pose.hh"
#include "math/Quaternion.hh"
#include "math/Vector3.hh"
#include "math/Vector2d.hh"

#include "sdf/sdf.hh"
#include "msgs/msgs.hh"
#include "rendering/RenderTypes.hh"
#include "common/CommonTypes.hh"

namespace Ogre
{
  class MovableObject;
  class SceneNode;
  class StaticGeometry;
  class RibbonTrail;
  class AxisAlignedBox;
  class AnimationState;
  class SkeletonInstance;
}

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{
    /// \brief A renderable object
    class Visual : public boost::enable_shared_from_this<Visual>
    {
      /// \brief Constructor
      public: Visual(const std::string &_name, VisualPtr _parent,
                     bool _useRTShader = true);

      /// \brief Constructor
      public: Visual(const std::string &_name, ScenePtr _scene,
                     bool _useRTShader = true);

      /// \brief Destructor
      public: virtual ~Visual();

      /// \brief Helper for the contructor
      public: void Init();

      public: void Fini();

      /// \brief Clone the visual with a new name
      public: VisualPtr Clone(const std::string &_name, VisualPtr _newParent);

      /// \brief Load from a message
      public: void LoadFromMsg(ConstVisualPtr &_msg);

      /// \brief Load the visual with a set of parameters
      public: void Load(sdf::ElementPtr sdf);

      /// \brief Load the visual with default parameters
      public: virtual void Load();

      /// \brief Update the visual.
      public: void Update();

      /// \brief Set the name of the visual
      public: void SetName(const std::string &name);

      /// \brief Get the name of the visual
      public: std::string GetName() const;

      /// \brief Attach a visual
      public: void AttachVisual(VisualPtr _vis);

      /// \brief Detach a visual
      public: void DetachVisual(VisualPtr _vis);
      public: void DetachVisual(const std::string &_name);

      /// \brief Attach a renerable object to the visual
      public: void AttachObject(Ogre::MovableObject *obj);

      /// \brief Returns true if an object with _name is attached
      public: bool HasAttachedObject(const std::string &_name);

      /// \brief Detach all objects
      public: void DetachObjects();

      /// \brief Get the number of attached visuals
      public: unsigned int GetChildCount();

      /// \brief Get an attached visual
      public: VisualPtr GetChild(unsigned int _num);

      /// \brief Attach a mesh to this visual by name
      public: Ogre::MovableObject *AttachMesh(const std::string &_meshName,
                                              const std::string &_objName="");

      /// \brief Set the scale
      public: void SetScale(const math::Vector3 &scale);

      /// \brief Get the scale
      public: math::Vector3 GetScale();

      /// \brief Set the material
      public: void SetMaterial(const std::string &materialName,
                               bool _unique = true);

      /// \brief Set the ambient color of the visual
      public: void SetAmbient(const common::Color &_color);

      /// \brief Set the diffuse color of the visual
      public: void SetDiffuse(const common::Color &_color);

      /// \brief Set the specular color of the visual
      public: void SetSpecular(const common::Color &_color);

      public: void AttachAxes();

      /// \brief Set the transparency
      public: void SetTransparency(float trans);

      /// \brief Get the transparency
      public: float GetTransparency();

      /// \brief Set the emissive value
      public: void SetEmissive(const common::Color &_color);

      /// \brief Set whether the visual should cast shadows
      public: void SetCastShadows(const bool &shadows);

      /// \brief Set whether the visual is visible
      /// \param visible set this node visible
      /// \param cascade setting this parameter in children too
      public: void SetVisible(bool visible, bool cascade = true);

      /// \brief Toggle whether this visual is visible
      public: void ToggleVisible();

      /// \brief Get whether the visual is visible
      public: bool GetVisible() const;

      /// \brief Set the position of the visual
      public: void SetPosition(const math::Vector3 &pos);

      /// \brief Set the rotation of the visual
      public: void SetRotation(const math::Quaternion &rot);

      /// \brief Set the pose of the visual
      public: void SetPose(const math::Pose &pose);

      /// \brief Get the position of the visual
      public: math::Vector3 GetPosition() const;

      /// \brief Get the rotation of the visual
      public: math::Quaternion GetRotation() const;

      /// \brief Get the pose of the visual
      public: math::Pose GetPose() const;

      /// \brief Get the global pose of the node
      public: math::Pose GetWorldPose() const;

      /// \brief Set the world pose of the visual
      public: void SetWorldPose(const math::Pose _pose);
      public: void SetWorldPosition(const math::Vector3 &_pos);
      public: void SetWorldRotation(const math::Quaternion &_q);

      /// \brief Return the scene Node of this visual entity
      public: Ogre::SceneNode *GetSceneNode() const;

      /// \brief Make the visual objects static renderables
      public: void MakeStatic();

      /// \brief Return true if the  visual is a static geometry
      public: bool IsStatic() const;

      /// \brief Set one visual to track/follow another
      public: void EnableTrackVisual(Visual *vis);

      /// \brief Disable tracking of a visual
      public: void DisableTrackVisual();

      /// \brief Get the normal map
      public: std::string GetNormalMap() const;

      /// \brief Set the normal map
      public: void SetNormalMap(const std::string &nmap);

      /// \brief True on or off a ribbon trail
      public: void SetRibbonTrail(bool value,
                  const common::Color &_initialColor,
                  const common::Color &_changeColor);

      /// \brief Get the bounding box for the visual
      public: math::Box GetBoundingBox() const;

      /// \brief Add a line to the visual
      public: DynamicLines *CreateDynamicLine(
                  RenderOpType type = RENDERING_LINE_STRIP);

      /// \brief Delete a dynamic line
      public: void DeleteDynamicLine(DynamicLines *line);

      /// \brief Attach a vertex of a line to the position of the visual
      public: void AttachLineVertex(DynamicLines *_line,
                                     unsigned int _index);

      /// \brief Get the name of the material
      public: std::string GetMaterialName() const;

      /// \brief Insert a mesh into Ogre
      public: void InsertMesh(const std::string &_meshName);

      /// \brief Insert a mesh into Ogre
      public: static void InsertMesh(const common::Mesh *mesh);

      /// \brief Update a visual based on a message
      public: void UpdateFromMsg(
                  const boost::shared_ptr< msgs::Visual const> &msg);

      /// \brief Return true if the visual is a plane
      public: bool IsPlane() const;

      /// \brief Get the parent visual, if one exists
      public: VisualPtr GetParent() const;

      /// \brief Get the shader type
      public: std::string GetShaderType() const;

      /// \brief Set the shader type
      public: void SetShaderType(const std::string &_type);

      public: void MoveToPosition(const math::Vector3 &_end,
                                   double _pitch, double _yaw, double _time);

      public: void MoveToPositions(const std::vector<math::Pose> &_pts,
                                   double _time,
                                   boost::function<void()> _onComplete = NULL);

      /// \brief Set visibility flags for this visual and all children
      public: void SetVisibilityFlags(uint32_t _flags);

      public: void ShowBoundingBox();
      public: void ShowCollision(bool _show);

      public: void ShowSkeleton(bool _show);

      public: void SetScene(ScenePtr _scene);
      public: ScenePtr GetScene() const;
      public: void ShowJoints(bool _show);
      public: void ShowCOM(bool _show);

      public: void SetSkeletonPose(const msgs::PoseAnimation &_pose);

      private: void GetBoundsHelper(Ogre::SceneNode *node,
                                    math::Box &box) const;

      private: std::string GetMeshName() const;

      public: void ClearParent();

      private: void DestroyAllAttachedMovableObjects(
                   Ogre::SceneNode* i_pSceneNode);

      private: sdf::ElementPtr sdf;

      private: std::string myMaterialName;
      private: std::string origMaterialName;

      protected: Ogre::SceneNode *sceneNode;

      private: float transparency;

      private: static unsigned int visualCounter;

      private: bool isStatic;
      private: Ogre::StaticGeometry *staticGeom;
      private: bool visible;

      private: static SelectionObj *selectionObj;

      private: Ogre::RibbonTrail *ribbonTrail;

      private: Ogre::SkeletonInstance *skeleton;

      private: event::ConnectionPtr preRenderConnection;

      // List of all the lines created
      private: std::list<DynamicLines*> lines;

      private: std::list< std::pair<DynamicLines*, unsigned int> > lineVertices;

      private: std::string name;
      public: VisualPtr parent;
      public: std::vector<VisualPtr> children;

      private: Ogre::AnimationState *animState;
      private: common::Time prevAnimTime;
      private: boost::function<void()> onAnimationComplete;
      protected: ScenePtr scene;

      private: bool useRTShader;
    };
    /// \}
  }
}
#endif
