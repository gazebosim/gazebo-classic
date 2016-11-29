/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_RENDERING_SCENE_HH_
#define _GAZEBO_RENDERING_SCENE_HH_

#include <memory>
#include <string>
#include <vector>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

#include <sdf/sdf.hh>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/Events.hh"
#include "gazebo/common/Color.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"

namespace SkyX
{
  class SkyX;
}

namespace Ogre
{
  class SceneManager;
  class Node;
  class Entity;
  class Mesh;
  class Vector3;
  class Quaternion;
}

namespace gazebo
{
  namespace rendering
  {
    class Visual;
    class Grid;
    class Heightmap;
    class ScenePrivate;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class Scene Scene.hh rendering/rendering.hh
    /// \brief Representation of an entire scene graph.
    ///
    /// Maintains all the Visuals, Lights, and Cameras for a World.
    class GZ_RENDERING_VISIBLE Scene :
      public boost::enable_shared_from_this<Scene>
    {
      public: enum SkyXMode {
        GZ_SKYX_ALL = 0x0FFFFFFF,
        GZ_SKYX_CLOUDS = 0x0000001,
        GZ_SKYX_MOON = 0x0000002,
        GZ_SKYX_NONE = 0
      };

      /// \brief Constructor.
      private: Scene();

      /// \brief Constructor.
      /// \param[in] _name Name of the scene.
      /// \param[in] _enableVisualizations True to enable visualizations,
      /// this should be set to true for user interfaces, and false for
      /// sensor generation.
      public: Scene(const std::string &_name,
                    const bool _enableVisualizations = false,
                    const bool _isServer = false);

      /// \brief Destructor
      public: virtual ~Scene();

      /// \brief Load the scene from a set of parameters.
      /// \param[in] _scene SDF scene element to load.
      public: void Load(sdf::ElementPtr _scene);

      /// \brief Load the scene with default parameters.
      public: void Load();

      /// \brief Init rendering::Scene.
      public: void Init();

      /// \brief Process all received messages.
      public: void PreRender();

      /// \brief Get the OGRE scene manager.
      /// \return Pointer to the Ogre SceneManager.
      public: Ogre::SceneManager *OgreSceneManager() const;

      /// \brief Get the name of the scene.
      /// \return Name of the scene.
      public: std::string Name() const;

      /// \brief Set the ambient color.
      /// \param[in] _color The ambient color to use.
      public: void SetAmbientColor(const common::Color &_color);

      /// \brief Get the ambient color.
      /// \return The scene's ambient color.
      public: common::Color AmbientColor() const;

      /// \brief Set the background color.
      /// \param[in] _color The background color.
      public: void SetBackgroundColor(const common::Color &_color);

      /// \brief Get the background color.
      /// \return The background color.
      public: common::Color BackgroundColor() const;

      /// \brief Create a square grid of cells.
      /// \param[in] _cellCount Number of grid cells in one direction.
      /// \param[in] _cellLength Length of one grid cell.
      /// \param[in] _lineWidth Width of the grid lines.
      /// \param[in] _color Color of the grid lines.
      public: void CreateGrid(const uint32_t _cellCount,
          const float _cellLength, const float _lineWidth,
          const common::Color &_color);

      /// \brief Get a grid based on an index. Index must be between 0 and
      /// Scene::GetGridCount.
      /// \param[in] _index Index of the grid.
      public: Grid *GetGrid(uint32_t _index) const;

      /// \brief Get the number of grids.
      /// \return The number of grids.
      public: uint32_t GridCount() const;

      /// \brief Create a camera
      /// \param[in] _name Name of the new camera.
      /// \param[in] _autoRender True to allow Gazebo to automatically
      /// render the camera. This should almost always be true.
      /// \return Pointer to the new camera.
      public: CameraPtr CreateCamera(const std::string &_name,
                                     const bool _autoRender = true);

      /// \brief Create a wide-angle camera
      /// \param[in] _name Name of the new camera.
      /// \param[in] _autoRender True to allow Gazebo to automatically
      /// render the camera. This should almost always be true.
      /// \return Pointer to the new camera.
      public: WideAngleCameraPtr CreateWideAngleCamera(const std::string &_name,
          const bool _autoRender = true);

#ifdef HAVE_OCULUS
      /// \brief Create an oculus rift camera
      /// \param[in] _name Name of the new camera.
      /// \return Pointer to the new camera.
      public: OculusCameraPtr CreateOculusCamera(const std::string &_name);

      /// \brief Get the number of cameras in this scene
      /// \return Number of cameras.
      public: uint32_t OculusCameraCount() const;
#endif

      /// \brief Create depth camera
      /// \param[in] _name Name of the new camera.
      /// \param[in] _autoRender True to allow Gazebo to automatically
      /// render the camera. This should almost always be true.
      /// \return Pointer to the new camera.
      public: DepthCameraPtr CreateDepthCamera(const std::string &_name,
                                               const bool _autoRender = true);

      /// \brief Create laser that generates data from rendering.
      /// \param[in] _name Name of the new laser.
      /// \param[in] _autoRender True to allow Gazebo to automatically
      /// render the camera. This should almost always be true.
      /// \return Pointer to the new laser.
      public: GpuLaserPtr CreateGpuLaser(const std::string &_name,
                                         const bool _autoRender = true);

      /// \brief Get the number of cameras in this scene
      /// \return Number of cameras.
      public: uint32_t CameraCount() const;

      /// \brief Get a camera based on an index. Index must be between
      /// 0 and Scene::GetCameraCount.
      /// \param[in] _index Index of the camera to get.
      /// \return Pointer to the camera. Or NULL if the index is invalid.
      public: CameraPtr GetCamera(const uint32_t _index) const;

      /// \brief Get a camera by name.
      /// \param[in] _name Name of the camera.
      /// \return Pointer to the camera. Or NULL if the name is invalid.
      public: CameraPtr GetCamera(const std::string &_name) const;

      /// \brief Create a user camera.
      ///
      /// A user camera is one design for use with a GUI.
      /// \param[in] _name Name of the UserCamera.
      /// \param[in] _stereoEnabled True to enable stereo rendering. This is
      /// here for compatibility with 3D monitors/TVs.
      /// \return A pointer to the new UserCamera.
      public: UserCameraPtr CreateUserCamera(const std::string &_name,
                  const bool _stereoEnabled = false);

      /// \brief Get the number of user cameras in this scene
      /// \return The number of user cameras.
      public: uint32_t UserCameraCount() const;

      /// \brief Get a user camera by index. The index value must be between
      /// 0 and Scene::GetUserCameraCount.
      /// \param[in] _index Index of the UserCamera to get.
      /// \return Pointer to the UserCamera, or NULL if the index was
      /// invalid.
      public: UserCameraPtr GetUserCamera(const uint32_t _index) const;

      /// \brief Remove a camera from the scene
      /// \param[in] _name Name of the camera.
      public: void RemoveCamera(const std::string &_name);

      /// \brief Get a light by name.
      /// \param[in] _name Name of the light to get.
      /// \return Pointer to the light, or NULL if the light was not found.
      public: LightPtr GetLight(const std::string &_name) const;

      /// \brief Get the count of the lights.
      /// \return The number of lights.
      public: uint32_t LightCount() const;

      /// \brief Get a light based on an index. The index must be between
      /// 0 and Scene::GetLightCount.
      /// \param[in] _index Index of the light.
      /// \return Pointer to the Light or NULL if index was invalid.
      public: LightPtr GetLight(const uint32_t _index) const;

      /// \brief Get a visual by name.
      /// \param[in] _name Name of the visual to retrieve.
      /// \return Pointer to the visual, NULL if not found.
      public: VisualPtr GetVisual(const std::string &_name) const;

      /// \brief Get a visual by id.
      /// \param[in] _id ID of the visual to retrieve.
      /// \return Pointer to the visual, NULL if not found.
      public: VisualPtr GetVisual(const uint32_t _id) const;

      /// \brief Select a visual by name.
      /// \param[in] _name Name of the visual to select.
      /// \param[in] _mode Selection mode (normal, or move).
      public: void SelectVisual(const std::string &_name,
                                const std::string &_mode);

      /// \brief Get an entity at a pixel location using a camera. Used for
      ///        mouse picking.
      /// \param[in] _camera The ogre camera, used to do mouse picking
      /// \param[in] _mousePos The position of the mouse in screen coordinates
      /// \param[out] _mod Used for object manipulation
      /// \return The selected entity, or NULL
      public: VisualPtr VisualAt(CameraPtr _camera,
                                 const ignition::math::Vector2i &_mousePos,
                                 std::string &_mod);

      /// \brief Move the visual to be ontop of the nearest visual below it.
      /// \param[in] _visualName Name of the visual to move.
      public: void SnapVisualToNearestBelow(const std::string &_visualName);

      /// \brief Get a visual at a mouse position.
      /// \param[in] _camera Pointer to the camera used to project the mouse
      /// position.
      /// \param[in] _mousePos The 2d position of the mouse in pixels.
      /// \return Pointer to the visual, NULL if none found.
      public: VisualPtr VisualAt(CameraPtr _camera,
                                 const ignition::math::Vector2i &_mousePos);

      /// \brief Get a model's visual at a mouse position.
      /// \param[in] _camera Pointer to the camera used to project the mouse
      /// position.
      /// \param[in] _mousePos The 2d position of the mouse in pixels.
      /// \return Pointer to the visual, NULL if none found.
      public: VisualPtr ModelVisualAt(CameraPtr _camera,
          const ignition::math::Vector2i &_mousePos);

      /// \brief Get the closest visual below a given visual.
      /// \param[in] _visualName Name of the visual to search below.
      /// \return Pointer to the visual below, or NULL if no visual.
      public: VisualPtr VisualBelow(const std::string &_visualName);

      /// \brief Get a visual directly below a point.
      /// \param[in] _pt 3D point to get the visual below.
      /// \param[out] _visuals The visuals below the point order in
      /// proximity.
      public: void VisualsBelowPoint(const ignition::math::Vector3d &_pt,
                                     std::vector<VisualPtr> &_visuals);

      /// \brief Get the Z-value of the first object below the given point.
      /// \param[in] _pt Position to search below for a visual.
      /// \return The Z-value of the nearest visual below the point. Zero
      /// is returned if no visual is found.
      public: double HeightBelowPoint(const ignition::math::Vector3d &_pt);

      /// \brief Get the world pos of a the first contact at a pixel location.
      /// \param[in] _camera Pointer to the camera.
      /// \param[in] _mousePos 2D position of the mouse in pixels.
      /// \param[out] _position 3D position of the first contact point.
      /// \return True if a valid object was hit by the raycast.
      public: bool FirstContact(CameraPtr _camera,
                                const ignition::math::Vector2i &_mousePos,
                                ignition::math::Vector3d &_position);

      /// \brief Print the scene graph to std_out.
      public: void PrintSceneGraph();

      /// \brief Hide or show a visual.
      /// \param[in] _name Name of the visual to change.
      /// \param[in] _visible True to make visual visible, False to make it
      /// invisible.
      public: void SetVisible(const std::string &_name, const bool _visible);

      /// \brief Draw a named line.
      /// \param[in] _start Start position of the line.
      /// \param[in] _end End position of the line.
      /// \param[in] _name Name of the line.
      public: void DrawLine(const ignition::math::Vector3d &_start,
                            const ignition::math::Vector3d &_end,
                            const std::string &_name);

      /// \brief Set the fog parameters.
      /// \param[in] _type Type of fog: "linear", "exp", or "exp2".
      /// \param[in] _color Color of the fog.
      /// \param[in] _density Fog density.
      /// \param[in] _start Distance from camera to start the fog.
      /// \param[in] _end Distance from camera at which the fog is at max
      /// density.
      public: void SetFog(const std::string &_type,
                          const common::Color &_color,
                          const double _density, const double _start,
                          const double _end);

      /// \brief Get the scene ID.
      /// \return The ID of the scene.
      public: uint32_t Id() const;

      /// \brief Get the scene Id as a string.
      /// \return The ID as a string.
      public: std::string IdString() const;

      /// \brief Set whether shadows are on or off
      /// \param[in] _value True to enable shadows, False to disable
      public: void SetShadowsEnabled(const bool _value);

      /// \brief Get whether shadows are on or off
      /// \return True if shadows are enabled.
      public: bool ShadowsEnabled() const;

      /// \brief Add a visual to the scene
      /// \param[in] _vis Visual to add.
      public: void AddVisual(VisualPtr _vis);

      /// \brief Remove a visual from the scene.
      /// \param[in] _vis Visual to remove.
      public: void RemoveVisual(VisualPtr _vis);

      /// \brief Remove a visual from the scene.
      /// \param[in] _id Id of the visual to remove.
      public: void RemoveVisual(const uint32_t _id);

      /// \internal
      /// \brief Set the id of a visual. Internally used when visual ids'
      /// are required to be updated from visual msgs.
      /// \param[in] _vis Pointer to visual.
      /// \param[in] _id New id to set to.
      public: void SetVisualId(VisualPtr _vis, const uint32_t _id);

      /// \brief Add a light to the scene
      /// \param[in] _light Light to add.
      public: void AddLight(LightPtr _light);

      /// \brief Remove a light to the scene
      /// \param[in] _light Light to Remove.
      public: void RemoveLight(LightPtr _light);

      /// \brief Set the grid on or off
      /// \param[in] _enabled Set to true to turn on the grid
      public: void SetGrid(const bool _enabled);

      /// \brief Show/hide the world origin indicator.
      /// \param[in] _show True to show the origin.
      public: void ShowOrigin(const bool _show);

      /// \brief Get the top level world visual.
      /// \return Pointer to the world visual.
      public: VisualPtr WorldVisual() const;

      /// \brief Remove the name of scene from a string.
      /// \param[in] _name Name to string the scene name from.
      /// \return The stripped name.
      public: std::string StripSceneName(const std::string &_name) const;

      /// \brief Get a pointer to the heightmap.
      /// \return Pointer to the heightmap, NULL if no heightmap.
      public: Heightmap *GetHeightmap() const;

      /// \brief Clear rendering::Scene
      public: void Clear();

      /// \brief Get the currently selected visual.
      /// \return Pointer to the currently selected visual, or NULL if
      /// nothing is selected.
      public: VisualPtr SelectedVisual() const;

      /// \brief Enable or disable wireframe for all visuals.
      /// \param[in] _show True to enable wireframe for all visuals.
      public: void SetWireframe(const bool _show);

      /// \brief Get whether wireframe is enabled for all visuals.
      /// \return True if wireframe is enabled for all visuals.
      public: bool Wireframe() const;

      /// \brief Enable or disable transparency for all visuals.
      /// \param[in] _show True to enable transparency for all visuals.
      public: void SetTransparent(const bool _show);

      /// \brief Enable or disable center of mass visualization.
      /// \param[in] _show True to enable center of mass visualization.
      public: void ShowCOMs(const bool _show);

      /// \brief Enable or disable inertia visualization.
      /// \param[in] _show True to enable inertia visualization.
      public: void ShowInertias(const bool _show);

      /// \brief Enable or disable link frame visualization.
      /// \param[in] _show True to enable link frame visualization.
      public: void ShowLinkFrames(const bool _show);

      /// \brief Enable or disable skeleton visualization.
      /// \param[in] _show True to enable skeleton visualization.
      public: void ShowSkeleton(const bool _show);

      /// \brief Enable or disable joint visualization.
      /// \param[in] _show True to enable joint visualization.
      public: void ShowJoints(const bool _show);

      /// \brief Enable or disable collision visualization.
      /// \param[in] _show True to enable collision visualization.
      public: void ShowCollisions(const bool _show);

      /// \brief Enable or disable contact visualization.
      /// \param[in] _show True to enable contact visualization.
      public: void ShowContacts(const bool _show);

      /// \brief Display clouds in the sky.
      /// \param[in] _show True to display clouds.
      public: void ShowClouds(const bool _show);

      /// \brief Get whether or not clouds are displayed.
      /// \return True if clouds are displayed.
      public: bool ShowClouds() const;

      /// \brief Set SkyX mode to enable/disable skyx components such as
      /// clouds and moon.
      /// \param[in] _mode SkyX mode bitmask.
      /// \sa Scene::SkyXMode
      public: void SetSkyXMode(const unsigned int _mode);

      /// \brief Get the sky in the scene.
      /// \return Pointer to the sky.
      public: SkyX::SkyX *GetSkyX() const;

      /// \brief Return true if the Scene has been initialized.
      /// \return True if the scene has been initialized.
      public: bool Initialized() const;

      /// \brief Get the scene simulation time.
      /// Note this is different from World::GetSimTime() because
      /// there is a lag between the time new poses are sent out by World
      /// and when they are received and applied by the Scene.
      /// \return The current simulation time in Scene
      public: common::Time SimTime() const;

      /// \brief Get the number of visuals.
      /// \return The number of visuals in the Scene.
      public: uint32_t VisualCount() const;

      /// \brief Remove all projectors.
      public: void RemoveProjectors();

      /// \brief Toggle layer visilibility. This will process all visuals.
      /// If a visual is on the specified layer its visiblity will be
      /// toggled. Visuals with a negative layer index are always visible.
      /// \param[in] _layer Index of the layer to toggle.
      public: void ToggleLayer(const int32_t _layer);

      /// \brief Helper function to setup the sky.
      private: void SetSky();

      /// \brief Initialize the deferred shading render path.
      private: void InitDeferredShading();

      /// \brief Helper function for GetVisualAt functions.
      /// \param[in] _camera Pointer to the camera.
      /// \param[in] _mousePos 2D position of the mouse in pixels.
      /// \param[in] _ignoreSelectionObj True to ignore selection objects,
      /// which are GUI objects use to manipulate objects.
      /// \return Pointer to the Ogre::Entity, NULL if none.
      private: Ogre::Entity *OgreEntityAt(CameraPtr _camera,
          const ignition::math::Vector2i &_mousePos,
          const bool _ignoreSelectionObj);

      /// \brief Get the mesh information for the given mesh.
      /// \param[in] _mesh Mesh to get info about.
      /// \param[out] _vertexCount Number of vertices in the mesh.
      /// \param[out] _vertices Array of the vertices.
      /// \param[out] _indexCount Number if indices.
      /// \param[out] _indices Array of the indices.
      /// \param[in] _position Position of the mesh.
      /// \param[in] _orient Orientation of the mesh.
      /// \param[in] _scale Scale of the mesh
      // Code found in Wiki: www.ogre3d.org/wiki/index.php/RetrieveVertexData
      private: void MeshInformation(const Ogre::Mesh *_mesh,
                                    size_t &_vertexCount,
                                    Ogre::Vector3* &_vertices,
                                    size_t &_indexCount,
                                    uint64_t* &_indices,
                                    const ignition::math::Vector3d &_position,
                                    const ignition::math::Quaterniond &_orient,
                                    const ignition::math::Vector3d &_scale);

      /// \brief Print scene graph.
      /// \param[in] _prefix String to prefix each line of output with.
      /// \param[in] _node The Ogre Node to print.
      private: void PrintSceneGraphHelper(const std::string &_prefix,
                                          Ogre::Node *_node);

      /// \brief Called when a scene message is received on the
      /// ~/scene topic
      /// \param[in] _msg The message.
      private: void OnScene(ConstScenePtr &_msg);

      /// \brief Response callback
      /// \param[in] _msg The message data.
      private: void OnResponse(ConstResponsePtr &_msg);

      /// \brief Request callback
      /// \param[in] _msg The message data.
      private: void OnRequest(ConstRequestPtr &_msg);

      /// \brief Joint message callback.
      /// \param[in] _msg The message data.
      private: void OnJointMsg(ConstJointPtr &_msg);

      /// \brief Sensor message callback.
      /// \param[in] _msg The message data.
      private: bool ProcessSensorMsg(ConstSensorPtr &_msg);

      /// \brief Process a joint message.
      /// \param[in] _msg The message data.
      private: bool ProcessJointMsg(ConstJointPtr &_msg);

      /// \brief Process a link message.
      /// \param[in] _msg The message data.
      private: bool ProcessLinkMsg(ConstLinkPtr &_msg);

      /// \brief Proces a scene message.
      /// \param[in] _msg The message data.
      private: bool ProcessSceneMsg(ConstScenePtr &_msg);

      /// \brief Process a model message.
      /// \param[in] _msg The message data.
      private: bool ProcessModelMsg(const msgs::Model &_msg);

      /// \brief Scene message callback.
      /// \param[in] _msg The message data.
      private: void OnSensorMsg(ConstSensorPtr &_msg);

      /// \brief Visual message callback.
      /// \param[in] _msg The message data.
      private: void OnVisualMsg(ConstVisualPtr &_msg);

      /// \brief Process a visual message.
      /// \param[in] _msg The message data.
      /// \param[in] _type Type of visual.
      /// \return True if message is processed successfully.
      private: bool ProcessVisualMsg(ConstVisualPtr &_msg,
          Visual::VisualType _type = Visual::VT_ENTITY);

      /// \brief Light factory message callback.
      /// \param[in] _msg The message data.
      private: void OnLightFactoryMsg(ConstLightPtr &_msg);

      /// \brief Light modify message callback.
      /// \param[in] _msg The message data.
      private: void OnLightModifyMsg(ConstLightPtr &_msg);

      /// \brief Process a light factory message.
      /// \param[in] _msg The message data.
      private: bool ProcessLightFactoryMsg(ConstLightPtr &_msg);

      /// \brief Process a light modify message.
      /// \param[in] _msg The message data.
      private: bool ProcessLightModifyMsg(ConstLightPtr &_msg);

      /// \brief Process a request message.
      /// \param[in] _msg The message data.
      private: void ProcessRequestMsg(ConstRequestPtr &_msg);

      /// \brief Sky message callback.
      /// \param[in] _msg The message data.
      private: void OnSkyMsg(ConstSkyPtr &_msg);

      /// \brief Model message callback.
      /// \param[in] _msg The message data.
      private: void OnModelMsg(ConstModelPtr &_msg);

      /// \brief Pose message callback.
      /// \param[in] _msg The message data.
      private: void OnPoseMsg(ConstPosesStampedPtr &_msg);

      /// \brief Skeleton animation callback.
      /// \param[in] _msg The message data.
      private: void OnSkeletonPoseMsg(ConstPoseAnimationPtr &_msg);

      /// \brief Road message callback.
      /// \param[in] _msg The message data.
      private: void OnRoadMsg(ConstRoadPtr &_msg);

      /// \brief Create a new center of mass visual.
      /// \param[in] _msg Message containing the link data.
      /// \param[in] _linkVisual Pointer to the link's visual.
      private: void CreateCOMVisual(ConstLinkPtr &_msg, VisualPtr _linkVisual);

      /// \brief Create a center of mass visual using SDF data.
      /// \param[in] _elem SDF element data.
      /// \param[in] _linkVisual Pointer to the link's visual.
      private: void CreateCOMVisual(sdf::ElementPtr _elem,
                                    VisualPtr _linkVisual);

      /// \brief Create a new inertia visual.
      /// \param[in] _msg Message containing the link data.
      /// \param[in] _linkVisual Pointer to the link's visual.
      private: void CreateInertiaVisual(ConstLinkPtr &_msg,
          VisualPtr _linkVisual);

      /// \brief Create an inertia visual using SDF data.
      /// \param[in] _elem SDF element data.
      /// \param[in] _linkVisual Pointer to the link's visual.
      private: void CreateInertiaVisual(sdf::ElementPtr _elem,
          VisualPtr _linkVisual);

      /// \brief Create a new link frame visual.
      /// \param[in] _msg Message containing the link data.
      /// \param[in] _linkVisual Pointer to the link's visual.
      private: void CreateLinkFrameVisual(ConstLinkPtr &_msg,
          VisualPtr _linkVisual);

      /// \brief Helper function to remove all visualizations attached to a
      /// visual.
      /// \param[in] _vis Visual that the visualizations are attached to.
      private: void RemoveVisualizations(VisualPtr _vis);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<ScenePrivate> dataPtr;
    };
    /// \}
  }
}
#endif
