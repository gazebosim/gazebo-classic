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
#ifndef _SCENE_PRIVATE_HH_
#define _SCENE_PRIVATE_HH_

#include <list>
#include <map>
#include <string>
#include <vector>

#include <boost/unordered/unordered_map.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <sdf/sdf.hh>

#include "gazebo/common/Events.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo/msgs/msgs.hh"
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

namespace boost
{
  class mutex;
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

    /// \def PoseMsgs_L.
    /// \brief List of messages.
    typedef std::map<uint32_t, msgs::Pose> PoseMsgs_M;

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
    typedef std::map<std::string, LightPtr> Light_M;
    /// \def SkeletonPoseMsgs_L
    /// \brief List of skeleton messages.
    typedef std::list<boost::shared_ptr<msgs::PoseAnimation const> >
                                                        SkeletonPoseMsgs_L;

    /// \def JointMsgs_M
    /// \brief Map of joint names to joint messages.
    typedef boost::unordered_map<std::string,
        boost::shared_ptr<msgs::Joint const> > JointMsgs_M;

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
      public: Ogre::SceneManager *manager;

      /// \brief A ray query used to locate distances to visuals.
      public: Ogre::RaySceneQuery *raySceneQuery;

      /// \brief All the grids in the scene.
      public: std::vector<Grid *> grids;

      /// \brief Unique ID counter.
      public: static uint32_t idCounter;

      /// \brief The unique ID of this scene.
      public: uint32_t id;

      /// \brief String form of the id.
      public: std::string idString;

      /// \brief List of visual messages to process.
      public: VisualMsgs_L visualMsgs;

      /// \brief List of light message to process.
      public: LightMsgs_L lightMsgs;

      /// \brief List of pose message to process.
      public: PoseMsgs_M poseMsgs;

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

      /// \brief A message used to select an object.
      public: boost::shared_ptr<msgs::Selection const> selectionMsg;

      /// \brief Mutex to lock the various message buffers.
      public: boost::mutex *receiveMutex;

      /// \brief Mutex to lock the pose message buffers.
      public: boost::recursive_mutex poseMsgMutex;

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

      /// \brief Subscribe to light topics
      public: transport::SubscriberPtr lightSub;

      /// \brief Subscribe to pose updates
      public: transport::SubscriberPtr poseSub;

      /// \brief Subscribe to joint updates.
      public: transport::SubscriberPtr jointSub;

      /// \brief Subscribe to selection updates.
      public: transport::SubscriberPtr selectionSub;

      /// \brief Subscribe to reponses.
      public: transport::SubscriberPtr responseSub;

      /// \brief Subscribe to skeleton pose updates.
      public: transport::SubscriberPtr skeletonPoseSub;

      /// \brief Subscribe to sky updates.
      public: transport::SubscriberPtr skySub;

      /// \brief Subscribe to model info updates
      public: transport::SubscriberPtr modelInfoSub;

      /// \brief Publish light updates.
      public: transport::PublisherPtr lightPub;

      /// \brief Respond to requests.
      public: transport::PublisherPtr responsePub;

      /// \brief Publish requests
      public: transport::PublisherPtr requestPub;

      /// \brief Event connections
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief The top level in our tree of visuals
      public: VisualPtr worldVisual;

      /// \brief Pointer to a visual selected by a user via the GUI.
      public: VisualPtr selectedVis;

      /// \brief Selection mode (normal or move). Normal means the the
      /// object is just selection, and not being moved by the user. Move
      /// means the object is being actively moved by the user and the Scene
      /// should then ignore pose updates from the physics engine until
      /// after the move is complete.
      public: std::string selectionMode;

      /// \brief Keep around our request message.
      public: msgs::Request *requestMsg;

      /// \brief True if visualizations should be rendered.
      public: bool enableVisualizations;

      /// \brief The heightmap, if any.
      public: Heightmap *terrain;

      /// \brief All the projectors.
      public: std::map<std::string, Projector *> projectors;

      /// \brief Pointer to the sky.
      public: SkyX::SkyX *skyx;

      /// \brief Controls the sky.
      public: SkyX::BasicController *skyxController;

      /// \brief True when all COMs should be visualized.
      public: bool showCOMs;

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
    };
  }
}
#endif
