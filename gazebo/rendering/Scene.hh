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
#ifndef _SCENE_HH_
#define _SCENE_HH_

#include <list>
#include <map>
#include <string>
#include <vector>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered/unordered_map.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <sdf/sdf.hh>

#include "gazebo/common/Events.hh"
#include "gazebo/common/Color.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo/math/Vector2i.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"

namespace SkyX
{
  class SkyX;
  class BasicController;
}

namespace Ogre
{
  class SceneManager;
  class RaySceneQuery;
  class Node;
  class Entity;
  class Mesh;
  class Vector3;
  class Quaternion;
}

namespace boost
{
  class mutex;
}

namespace gazebo
{
  namespace rendering
  {
    class Projector;
    class Light;
    class Visual;
    class Grid;
    class Heightmap;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class Scene Scene.hh rendering/rendering.hh
    /// \brief Representation of an entire scene graph.
    ///
    /// Maintains all the Visuals, Lights, and Cameras for a World.
    class GAZEBO_VISIBLE Scene : public boost::enable_shared_from_this<Scene>
    {
      public: enum SkyXMode {
        GZ_SKYX_ALL = 0x0FFFFFFF,
        GZ_SKYX_CLOUDS = 0x0000001,
        GZ_SKYX_MOON = 0x0000002,
        GZ_SKYX_NONE = 0
      };

      /// \brief Constructor.
      private: Scene() {}

      /// \brief Constructor.
      /// \param[in] _name Name of the scene.
      /// \param[in] _enableVisualizations True to enable visualizations,
      /// this should be set to true for user interfaces, and false for
      /// sensor generation.
      public: Scene(const std::string &_name,
                    bool _enableVisualizations = false,
                    bool _isServer = false);

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
      public: Ogre::SceneManager *GetManager() const;

      /// \brief Get the name of the scene.
      /// \return Name of the scene.
      public: std::string GetName() const;

      /// \brief Set the ambient color.
      /// \param[in] _color The ambient color to use.
      public: void SetAmbientColor(const common::Color &_color);

      /// \brief Get the ambient color.
      /// \return The scene's ambient color.
      public: common::Color GetAmbientColor() const;

      /// \brief Set the background color.
      /// \param[in] _color The background color.
      public: void SetBackgroundColor(const common::Color &_color);

      /// \brief Get the background color.
      /// \return The background color.
      public: common::Color GetBackgroundColor() const;

      /// \brief Create a square grid of cells.
      /// \param[in] _cellCount Number of grid cells in one direction.
      /// \param[in] _cellLength Length of one grid cell.
      /// \param[in] _lineWidth Width of the grid lines.
      /// \param[in] _color Color of the grid lines.
      public: void CreateGrid(uint32_t _cellCount, float _cellLength,
                              float _lineWidth, const common::Color &_color);

      /// \brief Get a grid based on an index. Index must be between 0 and
      /// Scene::GetGridCount.
      /// \param[in] _index Index of the grid.
      public: Grid *GetGrid(uint32_t _index) const;

      /// \brief Get the number of grids.
      /// \return The number of grids.
      public: uint32_t GetGridCount() const;

      /// \brief Create a camera
      /// \param[in] _name Name of the new camera.
      /// \param[in] _autoRender True to allow Gazebo to automatically
      /// render the camera. This should almost always be true.
      /// \return Pointer to the new camera.
      public: CameraPtr CreateCamera(const std::string &_name,
                                     bool _autoRender = true);

#ifdef HAVE_OCULUS
      /// \brief Create an oculus rift camera
      /// \param[in] _name Name of the new camera.
      /// \return Pointer to the new camera.
      public: OculusCameraPtr CreateOculusCamera(const std::string &_name);

      /// \brief Get the number of cameras in this scene
      /// \return Number of cameras.
      public: uint32_t GetOculusCameraCount() const;

#endif

      /// \brief Create depth camera
      /// \param[in] _name Name of the new camera.
      /// \param[in] _autoRender True to allow Gazebo to automatically
      /// render the camera. This should almost always be true.
      /// \return Pointer to the new camera.
      public: DepthCameraPtr CreateDepthCamera(const std::string &_name,
                                               bool _autoRender = true);

      /// \brief Create laser that generates data from rendering.
      /// \param[in] _name Name of the new laser.
      /// \param[in] _autoRender True to allow Gazebo to automatically
      /// render the camera. This should almost always be true.
      /// \return Pointer to the new laser.
      public: GpuLaserPtr CreateGpuLaser(const std::string &_name,
                                         bool _autoRender = true);

      /// \brief Get the number of cameras in this scene
      /// \return Number of cameras.
      public: uint32_t GetCameraCount() const;

      /// \brief Get a camera based on an index. Index must be between
      /// 0 and Scene::GetCameraCount.
      /// \param[in] _index Index of the camera to get.
      /// \return Pointer to the camera. Or NULL if the index is invalid.
      public: CameraPtr GetCamera(uint32_t _index) const;

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
                  bool _stereoEnabled = false);

      /// \brief Get the number of user cameras in this scene
      /// \return The number of user cameras.
      public: uint32_t GetUserCameraCount() const;

      /// \brief Get a user camera by index. The index value must be between
      /// 0 and Scene::GetUserCameraCount.
      /// \param[in] _index Index of the UserCamera to get.
      /// \return Pointer to the UserCamera, or NULL if the index was
      /// invalid.
      public: UserCameraPtr GetUserCamera(uint32_t _index) const;

      /// \brief Remove a camera from the scene
      /// \param[in] _name Name of the camera.
      public: void RemoveCamera(const std::string &_name);

      /// \brief Get a light by name.
      /// \param[in] _name Name of the light to get.
      /// \return Pointer to the light, or NULL if the light was not found.
      public: LightPtr GetLight(const std::string &_name) const;

      /// \brief Get the count of the lights.
      /// \return The number of lights.
      public: uint32_t GetLightCount() const;

      /// \brief Get a light based on an index. The index must be between
      /// 0 and Scene::GetLightCount.
      /// \param[in] _index Index of the light.
      /// \return Pointer to the Light or NULL if index was invalid.
      public: LightPtr GetLight(uint32_t _index) const;

      /// \brief Get a visual by name.
      /// \param[in] _name Name of the visual to retrieve.
      /// \return Pointer to the visual, NULL if not found.
      public: VisualPtr GetVisual(const std::string &_name) const;

      /// \brief Get a visual by id.
      /// \param[in] _id ID of the visual to retrieve.
      /// \return Pointer to the visual, NULL if not found.
      public: VisualPtr GetVisual(uint32_t _id) const;

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
      public: VisualPtr GetVisualAt(CameraPtr _camera,
                                    const math::Vector2i &_mousePos,
                                    std::string &_mod);

      /// \brief Move the visual to be ontop of the nearest visual below it.
      /// \param[in] _visualName Name of the visual to move.
      public: void SnapVisualToNearestBelow(const std::string &_visualName);

      /// \brief Get a visual at a mouse position.
      /// \param[in] _camera Pointer to the camera used to project the mouse
      /// position.
      /// \param[in] _mousePos The 2d position of the mouse in pixels.
      /// \return Pointer to the visual, NULL if none found.
      public: VisualPtr GetVisualAt(CameraPtr _camera,
                                    const math::Vector2i &_mousePos);

      /// \brief Get a model's visual at a mouse position.
      /// \param[in] _camera Pointer to the camera used to project the mouse
      /// position.
      /// \param[in] _mousePos The 2d position of the mouse in pixels.
      /// \return Pointer to the visual, NULL if none found.
      public: VisualPtr GetModelVisualAt(CameraPtr _camera,
                                         const math::Vector2i &_mousePos);


      /// \brief Get the closest visual below a given visual.
      /// \param[in] _visualName Name of the visual to search below.
      /// \return Pointer to the visual below, or NULL if no visual.
      public: VisualPtr GetVisualBelow(const std::string &_visualName);

      /// \brief Get a visual directly below a point.
      /// \param[in] _pt 3D point to get the visual below.
      /// \param[out] _visuals The visuals below the point order in
      /// proximity.
      public: void GetVisualsBelowPoint(const math::Vector3 &_pt,
                                        std::vector<VisualPtr> &_visuals);


      /// \brief Get the Z-value of the first object below the given point.
      /// \param[in] _pt Position to search below for a visual.
      /// \return The Z-value of the nearest visual below the point. Zero
      /// is returned if no visual is found.
      public: double GetHeightBelowPoint(const math::Vector3 &_pt);

      /// \brief Get the world pos of a the first contact at a pixel location.
      /// \param[in] _camera Pointer to the camera.
      /// \param[in] _mousePos 2D position of the mouse in pixels.
      /// \param[out] _position 3D position of the first contact point.
      /// \return True if a valid object was hit by the raycast.
      public: bool GetFirstContact(CameraPtr _camera,
                                   const math::Vector2i &_mousePos,
                                   math::Vector3 &_position);

      /// \brief Print the scene graph to std_out.
      public: void PrintSceneGraph();

      /// \brief Hide or show a visual.
      /// \param[in] _name Name of the visual to change.
      /// \param[in] _visible True to make visual visible, False to make it
      /// invisible.
      public: void SetVisible(const std::string &_name, bool _visible);

      /// \brief Draw a named line.
      /// \param[in] _start Start position of the line.
      /// \param[in] _end End position of the line.
      /// \param[in] _name Name of the line.
      public: void DrawLine(const math::Vector3 &_start,
                            const math::Vector3 &_end,
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
                           double _density, double _start, double _end);

      /// \brief Get the scene ID.
      /// \return The ID of the scene.
      public: uint32_t GetId() const;

      /// \brief Get the scene Id as a string.
      /// \return The ID as a string.
      public: std::string GetIdString() const;

      /// \brief Set whether shadows are on or off
      /// \param[in] _value True to enable shadows, False to disable
      public: void SetShadowsEnabled(bool _value);

      /// \brief Get whether shadows are on or off
      /// \return True if shadows are enabled.
      public: bool GetShadowsEnabled() const;

      /// \brief Add a visual to the scene
      /// \param[in] _vis Visual to add.
      public: void AddVisual(VisualPtr _vis);

      /// \brief Remove a visual from the scene.
      /// \param[in] _vis Visual to remove.
      public: void RemoveVisual(VisualPtr _vis);

      /// \brief Add a light to the scene
      /// \param[in] _light Light to add.
      public: void AddLight(LightPtr _light);

      /// \brief Remove a light to the scene
      /// \param[in] _light Light to Remove.
      public: void RemoveLight(LightPtr _light);

      /// \brief Set the grid on or off
      /// \param[in] _enabled Set to true to turn on the grid
      public: void SetGrid(bool _enabled);

      /// \brief Get the top level world visual.
      /// \return Pointer to the world visual.
      public: VisualPtr GetWorldVisual() const;

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
      public: VisualPtr GetSelectedVisual() const;

      /// \brief Enable or disable wireframe for all visuals.
      /// \param[in] _show True to enable wireframe for all visuals.
      public: void SetWireframe(bool _show);

      /// \brief Enable or disable transparency for all visuals.
      /// \param[in] _show True to enable transparency for all visuals.
      public: void SetTransparent(bool _show);

      /// \brief Enable or disable center of mass visualization.
      /// \param[in] _show True to enable center of mass visualization.
      public: void ShowCOMs(bool _show);

      /// \brief Enable or disable joint visualization.
      /// \param[in] _show True to enable joint visualization.
      public: void ShowJoints(bool _show);

      /// \brief Enable or disable collision visualization.
      /// \param[in] _show True to enable collision visualization.
      public: void ShowCollisions(bool _show);

      /// \brief Enable or disable contact visualization.
      /// \param[in] _show True to enable contact visualization.
      public: void ShowContacts(bool _show);

      /// \brief Display clouds in the sky.
      /// \param[in] _show True to display clouds.
      public: void ShowClouds(bool _show);

      /// \brief Get whether or not clouds are displayed.
      /// \return True if clouds are displayed.
      public: bool GetShowClouds() const;


      /// \brief Set SkyX mode to enable/disable skyx components such as
      /// clouds and moon.
      /// \param[in] _mode SkyX mode bitmask.
      /// \sa Scene::SkyXMode
      public: void SetSkyXMode(unsigned int _mode);

      /// \brief Return true if the Scene has been initialized.
      public: bool GetInitialized() const;

      /// \brief Get the scene simulation time.
      /// Note this is different from World::GetSimTime() because
      /// there is a lag between the time new poses are sent out by World
      /// and when they are received and applied by the Scene.
      /// \return The current simulation time in Scene
      public: common::Time GetSimTime() const;

      /// \brief Get the number of visuals.
      /// \return The number of visuals in the Scene.
      public: uint32_t GetVisualCount() const;

      /// \brief Remove all projectors.
      public: void RemoveProjectors();

      /// \brief Helper function to setup the sky.
      private: void SetSky();

      /// \brief Initialize the deferred shading render path.
      private: void InitDeferredShading();

      /// \brief Helper function for GetVisualAt functions.
      /// \param[in] _camera Pointer to the camera.
      /// \param[in] _mousePos 2D position of the mouse in pixels.
      /// \param[in] _ignorSelectionObj True to ignore selection objects,
      /// which are GUI objects use to manipulate objects.
      /// \return Pointer to the Ogre::Entity, NULL if none.
      private: Ogre::Entity *GetOgreEntityAt(CameraPtr _camera,
                                             const math::Vector2i &_mousePos,
                                             bool _ignorSelectionObj);

      /// \brief Get the mesh information for the given mesh.
      /// \param[in] _mesh Mesh to get info about.
      /// \param[out] _count Number of vertices in the mesh.
      /// \param[out] _vertices Array of the vertices.
      /// \param[out] _indexCount Number if indices.
      /// \param[out] _indices Array of the indices.
      /// \param[in] _position Position of the mesh.
      /// \param[in] _orient Orientation of the mesh.
      /// \param[in] _scale Scale of the mesh
      // Code found in Wiki: www.ogre3d.org/wiki/index.php/RetrieveVertexData
      private: void GetMeshInformation(const Ogre::Mesh *_mesh,
                                       size_t &_vertexCount,
                                       Ogre::Vector3* &_vertices,
                                       size_t &_indexCount,
                                       uint64_t* &_indices,
                                       const Ogre::Vector3 &_position,
                                       const Ogre::Quaternion &_orient,
                                       const Ogre::Vector3 &_scale);

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
      private: bool ProcessVisualMsg(ConstVisualPtr &_msg);

      /// \brief Light message callback.
      /// \param[in] _msg The message data.
      private: void OnLightMsg(ConstLightPtr &_msg);

      /// \brief Process a light message.
      /// \param[in] _msg The message data.
      private: bool ProcessLightMsg(ConstLightPtr &_msg);

      /// \brief Process a request message.
      /// \param[in] _msg The message data.
      private: void ProcessRequestMsg(ConstRequestPtr &_msg);

      /// \brief Selection message callback.
      /// \param[in] _msg The message data.
      private: void OnSelectionMsg(ConstSelectionPtr &_msg);

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

      /// \brief Create a new center of mass visual.
      /// \param[in] _msg Message containing the link data.
      /// \param[in] _linkVisual Pointer to the link's visual.
      private: void CreateCOMVisual(ConstLinkPtr &_msg, VisualPtr _linkVisual);

      /// \brief Create a center of mass visual using SDF data.
      /// \param[in] _elem SDF element data.
      /// \param[in] _linkVisual Pointer to the link's visual.
      private: void CreateCOMVisual(sdf::ElementPtr _elem,
                                    VisualPtr _linkVisual);

      /// \brief Name of the scene.
      private: std::string name;

      /// \brief Scene SDF element.
      private: sdf::ElementPtr sdf;

      /// \brief All the cameras.
      private: std::vector<CameraPtr> cameras;

      /// \brief All the user cameras.
      private: std::vector<UserCameraPtr> userCameras;

#ifdef HAVE_OCULUS
      /// \brief All the oculus cameras.
      private: std::vector<OculusCameraPtr> oculusCameras;
#endif

      /// \brief The ogre scene manager.
      private: Ogre::SceneManager *manager;

      /// \brief A ray query used to locate distances to visuals.
      private: Ogre::RaySceneQuery *raySceneQuery;

      /// \brief All the grids in the scene.
      private: std::vector<Grid *> grids;

      /// \brief Unique ID counter.
      private: static uint32_t idCounter;

      /// \brief The unique ID of this scene.
      private: uint32_t id;

      /// \brief String form of the id.
      private: std::string idString;

      /// \def VisualMsgs_L
      /// \brief List of visual messages.
      typedef std::list<boost::shared_ptr<msgs::Visual const> > VisualMsgs_L;

      /// \brief List of visual messages to process.
      private: VisualMsgs_L visualMsgs;

      /// \def LightMsgs_L.
      /// \brief List of light messages.
      typedef std::list<boost::shared_ptr<msgs::Light const> > LightMsgs_L;

      /// \brief List of light message to process.
      private: LightMsgs_L lightMsgs;

      /// \def PoseMsgs_L.
      /// \brief List of messages.
      typedef std::map<uint32_t, msgs::Pose> PoseMsgs_M;

      /// \brief List of pose message to process.
      private: PoseMsgs_M poseMsgs;

      /// \def SceneMsgs_L
      /// \brief List of scene messages.
      typedef std::list<boost::shared_ptr<msgs::Scene const> > SceneMsgs_L;

      /// \brief List of scene message to process.
      private: SceneMsgs_L sceneMsgs;

      /// \def JointMsgs_L
      /// \brief List of joint messages.
      typedef std::list<boost::shared_ptr<msgs::Joint const> > JointMsgs_L;

      /// \brief List of joint message to process.
      private: JointMsgs_L jointMsgs;

      /// \def LinkMsgs_L
      /// \brief List of link messages.
      typedef std::list<boost::shared_ptr<msgs::Link const> > LinkMsgs_L;

      /// \brief List of link message to process.
      private: LinkMsgs_L linkMsgs;

      /// \def ModelMsgs_L
      /// \brief List of model messages.
      typedef std::list<boost::shared_ptr<msgs::Model const> > ModelMsgs_L;
      /// \brief List of model message to process.
      private: ModelMsgs_L modelMsgs;

      /// \def SensorMsgs_L
      /// \brief List of sensor messages.
      typedef std::list<boost::shared_ptr<msgs::Sensor const> > SensorMsgs_L;

      /// \brief List of sensor message to process.
      private: SensorMsgs_L sensorMsgs;

      /// \def RequestMsgs_L
      /// \brief List of request messages.
      typedef std::list<boost::shared_ptr<msgs::Request const> > RequestMsgs_L;
      /// \brief List of request message to process.
      private: RequestMsgs_L requestMsgs;

      /// \def Visual_M
      /// \brief Map of visuals and their names.
      typedef std::map<uint32_t, VisualPtr> Visual_M;

      /// \brief Map of all the visuals in this scene.
      private: Visual_M visuals;

      /// \def Light_M
      /// \brief Map of lights
      typedef std::map<std::string, LightPtr> Light_M;

      /// \brief Map of all the lights in this scene.
      private: Light_M lights;

      /// \def SkeletonPoseMsgs_L
      /// \brief List of skeleton messages.
      typedef std::list<boost::shared_ptr<msgs::PoseAnimation const> >
                                                          SkeletonPoseMsgs_L;
      /// \brief List of skeleton message to process.
      private: SkeletonPoseMsgs_L skeletonPoseMsgs;

      /// \brief A message used to select an object.
      private: boost::shared_ptr<msgs::Selection const> selectionMsg;

      /// \brief Mutex to lock the various message buffers.
      private: boost::mutex *receiveMutex;

      /// \brief Mutex to lock the pose message buffers.
      private: boost::recursive_mutex poseMsgMutex;

      /// \brief Communication Node
      private: transport::NodePtr node;

      /// \brief Subscribe to sensor topic
      private: transport::SubscriberPtr sensorSub;

      /// \brief Subscribe to scene topic
      private: transport::SubscriberPtr sceneSub;

      /// \brief Subscribe to the request topic
      private: transport::SubscriberPtr requestSub;

      /// \brief Subscribe to visual topic
      private: transport::SubscriberPtr visSub;

      /// \brief Subscribe to light topics
      private: transport::SubscriberPtr lightSub;

      /// \brief Subscribe to pose updates
      private: transport::SubscriberPtr poseSub;

      /// \brief Subscribe to joint updates.
      private: transport::SubscriberPtr jointSub;

      /// \brief Subscribe to selection updates.
      private: transport::SubscriberPtr selectionSub;

      /// \brief Subscribe to reponses.
      private: transport::SubscriberPtr responseSub;

      /// \brief Subscribe to skeleton pose updates.
      private: transport::SubscriberPtr skeletonPoseSub;

      /// \brief Subscribe to sky updates.
      private: transport::SubscriberPtr skySub;

      /// \brief Subscribe to model info updates
      private: transport::SubscriberPtr modelInfoSub;

      /// \brief Publish light updates.
      private: transport::PublisherPtr lightPub;

      /// \brief Respond to requests.
      private: transport::PublisherPtr responsePub;

      /// \brief Publish requests
      private: transport::PublisherPtr requestPub;

      /// \brief Event connections
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief The top level in our tree of visuals
      private: VisualPtr worldVisual;

      /// \brief Pointer to a visual selected by a user via the GUI.
      private: VisualPtr selectedVis;

      /// \brief Selection mode (normal or move). Normal means the the
      /// object is just selection, and not being moved by the user. Move
      /// means the object is being actively moved by the user and the Scene
      /// should then ignore pose updates from the physics engine until
      /// after the move is complete.
      private: std::string selectionMode;

      /// \brief Keep around our request message.
      private: msgs::Request *requestMsg;

      /// \brief True if visualizations should be rendered.
      private: bool enableVisualizations;

      /// \brief The heightmap, if any.
      private: Heightmap *terrain;

      /// \brief All the projectors.
      private: std::map<std::string, Projector *> projectors;

      /// \brief Pointer to the sky.
      public: SkyX::SkyX *skyx;

      /// \brief Controls the sky.
      private: SkyX::BasicController *skyxController;

      /// \brief True when all COMs should be visualized.
      private: bool showCOMs;

      /// \brief True when all collisions should be visualized.
      private: bool showCollisions;

      /// \brief True when all joints should be visualized.
      private: bool showJoints;

      /// \brief True when all objects should be transparent.
      private: bool transparent;

      /// \brief True when all objects should be wireframe.
      private: bool wireframe;

      /// \brief Initialized.
      private: bool initialized;

      /// \brief SimTime of this Scene, as we receive PosesStamped from
      /// the world, we update this time accordingly.
      private: common::Time sceneSimTimePosesReceived;

      /// \brief SimTime of this Scene, after applying PosesStamped to
      /// scene, we update this time accordingly.
      private: common::Time sceneSimTimePosesApplied;

      /// \brief Keeps track of the visual ID for contact visualization.
      private: uint32_t contactVisId;

      /// \def JointMsgs_M
      /// \brief Map of joint names to joint messages.
      typedef boost::unordered_map<std::string,
          boost::shared_ptr<msgs::Joint const> > JointMsgs_M;

      /// \brief Keep track of data of joints.
      private: JointMsgs_M joints;
    };
    /// \}
  }
}
#endif
