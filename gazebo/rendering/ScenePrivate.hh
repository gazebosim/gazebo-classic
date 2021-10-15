/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef GAZEBO_RENDERING_SCENE_PRIVATE_HH_
#define GAZEBO_RENDERING_SCENE_PRIVATE_HH_

#include <condition_variable>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <boost/unordered/unordered_map.hpp>

#include <sdf/sdf.hh>

#include "gazebo/common/Events.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/rendering/MarkerManager.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace SkyX
{
  class SkyX;
  class BasicController;
}

namespace Ogre
{
  class SceneManager;
  class RaySceneQuery;
}

namespace gazebo
{
  namespace rendering
  {
    class Projector;
    class Visual;
    class Grid;
    class Heightmap;

    /// \def Visual_M
    /// \brief Map of visuals and their names.
    typedef std::map<uint32_t, VisualPtr> Visual_M;

    /// \def VisualMsgs_L
    /// \brief List of visual messages.
    typedef std::list<boost::shared_ptr<msgs::Visual const> > VisualMsgs_L;

    /// \def LightMsgs_L.
    /// \brief List of light messages.
    typedef std::list<boost::shared_ptr<msgs::Light const> > LightMsgs_L;

    /// \typedef PoseMsgs_M.
    /// \brief List of messages.
    typedef std::map<uint32_t, msgs::Pose> PoseMsgs_M;

    /// \typedef LightPoseMsgs_M.
    /// \brief List of messages.
    typedef std::map<std::string, msgs::Pose> LightPoseMsgs_M;

    /// \def SceneMsgs_L
    /// \brief List of scene messages.
    typedef std::list<boost::shared_ptr<msgs::Scene const> > SceneMsgs_L;

    /// \def JointMsgs_L
    /// \brief List of joint messages.
    typedef std::list<boost::shared_ptr<msgs::Joint const> > JointMsgs_L;

    /// \def LinkMsgs_L
    /// \brief List of link messages.
    typedef std::list<boost::shared_ptr<msgs::Link const> > LinkMsgs_L;

    /// \def ModelMsgs_L
    /// \brief List of model messages.
    typedef std::list<boost::shared_ptr<msgs::Model const> > ModelMsgs_L;

    /// \def SensorMsgs_L
    /// \brief List of sensor messages.
    typedef std::list<boost::shared_ptr<msgs::Sensor const> > SensorMsgs_L;

    /// \def RequestMsgs_L
    /// \brief List of request messages.
    typedef std::list<boost::shared_ptr<msgs::Request const> > RequestMsgs_L;

    /// \def Light_M
    /// \brief Map of lights
    typedef std::map<uint32_t, LightPtr> Light_M;

    /// \def SkeletonPoseMsgs_L
    /// \brief List of skeleton messages.
    typedef std::list<boost::shared_ptr<msgs::PoseAnimation const> >
                                                        SkeletonPoseMsgs_L;

    /// \def JointMsgs_M
    /// \brief Map of joint names to joint messages.
    typedef boost::unordered_map<std::string,
        boost::shared_ptr<msgs::Joint const> > JointMsgs_M;

    /// \def RoadMsgs_L
    /// \brief List of road messages
    typedef std::list<boost::shared_ptr<msgs::Road const> > RoadMsgs_L;

    /// \brief Private data for the Visual class
    class ScenePrivate
    {
/*      public: enum SkyXMode {
        GZ_SKYX_ALL = 0x0FFFFFFF,
        GZ_SKYX_CLOUDS = 0x0000001,
        GZ_SKYX_MOON = 0x0000002,
        GZ_SKYX_NONE = 0
      };*/

      /// \brief Name of the scene.
      public: std::string name;

      /// \brief Scene SDF element.
      public: sdf::ElementPtr sdf;

      /// \brief All the cameras.
      public: std::vector<CameraPtr> cameras;

      /// \brief All the user cameras.
      public: std::vector<UserCameraPtr> userCameras;

#ifdef HAVE_OCULUS
      /// \brief All the oculus cameras.
      public: std::vector<OculusCameraPtr> oculusCameras;
#endif

      /// \brief The ogre scene manager.
      public: Ogre::SceneManager *manager = nullptr;

      /// \brief A ray query used to locate distances to visuals.
      public: Ogre::RaySceneQuery *raySceneQuery = nullptr;

      /// \brief All the grids in the scene.
      public: std::vector<Grid *> grids;

      /// \brief Unique ID counter.
      public: static uint32_t idCounter;

      /// \brief The unique ID of this scene.
      public: uint32_t id;

      /// \brief String form of the id.
      public: std::string idString;

      /// \brief List of model visual messages to process.
      public: VisualMsgs_L modelVisualMsgs;

      /// \brief List of link visual messages to process.
      public: VisualMsgs_L linkVisualMsgs;

      /// \brief List of visual messages to process.
      public: VisualMsgs_L visualMsgs;

      /// \brief List of collision visual messages to process.
      public: VisualMsgs_L collisionVisualMsgs;

      /// \brief List of light factory message to process.
      public: LightMsgs_L lightFactoryMsgs;

      /// \brief List of light modify message to process.
      public: LightMsgs_L lightModifyMsgs;

      /// \brief List of pose message to process.
      public: PoseMsgs_M poseMsgs;

      /// \brief List of pose message to process.
      public: LightPoseMsgs_M lightPoseMsgs;

      /// \brief List of scene message to process.
      public: SceneMsgs_L sceneMsgs;

      /// \brief List of joint message to process.
      public: JointMsgs_L jointMsgs;

      /// \brief List of link message to process.
      public: LinkMsgs_L linkMsgs;

      /// \brief List of model message to process.
      public: ModelMsgs_L modelMsgs;

      /// \brief List of sensor message to process.
      public: SensorMsgs_L sensorMsgs;

      /// \brief List of request message to process.
      public: RequestMsgs_L requestMsgs;

      /// \brief Map of all the visuals in this scene.
      public: Visual_M visuals;

      /// \brief Map of all the lights in this scene.
      public: Light_M lights;

      /// \brief List of skeleton message to process.
      public: SkeletonPoseMsgs_L skeletonPoseMsgs;

      /// \brief List of road messages to process.
      public: RoadMsgs_L roadMsgs;

      /// \brief used to wake up upon new Pose available
      public: std::condition_variable newPoseCondition;

      /// \brief Flag indicating that a new pose msg is available
      public: bool newPoseAvailable = false;

      /// \brief Protects flag newPoseAvailable
      public: std::mutex newPoseMutex;

      /// \brief Mutex to lock the various message buffers.
      public: std::mutex *receiveMutex = nullptr;

      /// \brief Mutex to lock the pose message buffers.
      public: std::recursive_mutex poseMsgMutex;

      /// \brief Communication Node
      public: transport::NodePtr node;

      /// \brief Subscribe to sensor topic
      public: transport::SubscriberPtr sensorSub;

      /// \brief Subscribe to scene topic
      public: transport::SubscriberPtr sceneSub;

      /// \brief Subscribe to the request topic
      public: transport::SubscriberPtr requestSub;

      /// \brief Subscribe to visual topic
      public: transport::SubscriberPtr visSub;

      /// \brief Subscribe to light factory topic
      public: transport::SubscriberPtr lightFactorySub;

      /// \brief Subscribe to light modify topic
      public: transport::SubscriberPtr lightModifySub;

      /// \brief Subscribe to pose updates
      public: transport::SubscriberPtr poseSub;

      /// \brief Subscribe to joint updates.
      public: transport::SubscriberPtr jointSub;

      /// \brief Subscribe to reponses.
      public: transport::SubscriberPtr responseSub;

      /// \brief Subscribe to skeleton pose updates.
      public: transport::SubscriberPtr skeletonPoseSub;

      /// \brief Subscribe to sky updates.
      public: transport::SubscriberPtr skySub;

      /// \brief Subscribe to model info updates
      public: transport::SubscriberPtr modelInfoSub;

      /// \brief Respond to requests.
      public: transport::PublisherPtr responsePub;

      /// \brief Publish requests
      public: transport::PublisherPtr requestPub;

      /// \brief Subscribe to roads topic
      public: transport::SubscriberPtr roadSub;

      /// \brief Event connections
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief The top level in our tree of visuals
      public: VisualPtr worldVisual;

      /// \brief Visual representing the world origin frame.
      public: OriginVisualPtr originVisual;

      /// \brief Pointer to a visual selected by a user via the GUI.
      public: VisualPtr selectedVis;

      /// \brief Selection mode (normal or move). Normal means the the
      /// object is just selection, and not being moved by the user. Move
      /// means the object is being actively moved by the user and the Scene
      /// should then ignore pose updates from the physics engine until
      /// after the move is complete.
      public: std::string selectionMode;

      /// \brief Keep around our request message.
      public: std::unique_ptr<msgs::Request> requestMsg;

      /// \brief True if visualizations should be rendered.
      public: bool enableVisualizations;

      /// \brief True if this scene is running on the server.
      public: bool isServer;

      /// \brief The heightmap, if any.
      public: Heightmap *terrain = nullptr;

      /// \brief The heightmap level of detail
      public: unsigned int heightmapLOD = 0u;

      /// \brief The heightmap skirt length
      public: double heightmapSkirtLength = 1.0;

      /// \brief All the projectors.
      public: std::map<std::string, Projector *> projectors;

      /// \brief Pointer to the sky.
      public: SkyX::SkyX *skyx = nullptr;

      /// \brief Controls the sky.
      public: SkyX::BasicController *skyxController = nullptr;

      /// \brief True when all COMs should be visualized.
      public: bool showCOMs;

      /// \brief True when all inertias should be visualized.
      public: bool showInertias;

      /// \brief True when all link frames should be visualized.
      public: bool showLinkFrames;

      /// \brief True when all skeletons should be visualized.
      public: bool showSkeleton;

      /// \brief True when all collisions should be visualized.
      public: bool showCollisions;

      /// \brief True when all joints should be visualized.
      public: bool showJoints;

      /// \brief True when all objects should be transparent.
      public: bool transparent;

      /// \brief True when all objects should be wireframe.
      public: bool wireframe;

      /// \brief Initialized.
      public: bool initialized;

      /// \brief SimTime of this Scene, as we receive PosesStamped from
      /// the world, we update this time accordingly.
      public: common::Time sceneSimTimePosesReceived;

      /// \brief SimTime of this Scene, after applying PosesStamped to
      /// scene, we update this time accordingly.
      public: common::Time sceneSimTimePosesApplied;

      /// \brief Keeps track of the visual ID for contact visualization.
      public: uint32_t contactVisId;

      /// \brief Keep track of data of joints.
      public: JointMsgs_M joints;

      /// \brief Size of shadow texture
      public: unsigned int shadowTextureSize = 1024u;

      /// \brief Manager of marker visuals
      public: MarkerManager markerManager;

      /// \brief State of each layer where key is the layer id, and value is
      /// the layer's visibility.
      public: std::map<int32_t, bool> layerState;

      /// \brief Shadow caster material name
      public: std::string shadowCasterMaterialName = "Gazebo/shadow_caster";

      /// \brief Shadow caster render back faces
      public: bool shadowCasterRenderBackFaces = true;
    };
  }
}
#endif
