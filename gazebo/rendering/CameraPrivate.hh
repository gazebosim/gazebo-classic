/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_RENDERING_CAMERAPRIVATE_HH_
#define GAZEBO_RENDERING_CAMERAPRIVATE_HH_

#include <deque>
#include <mutex>
#include <utility>
#include <list>
#include <ignition/math/Pose3.hh>

#include "gazebo/common/PID.hh"
#include "gazebo/common/VideoEncoder.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/util/system.hh"

namespace Ogre
{
  class CompositorInstance;
}

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the Camera class
    class GZ_RENDERING_VISIBLE CameraPrivate
    {
      /// \brief Visual that the camera is tracking.
      public: VisualPtr trackedVisual;

      /// \brief Counter used to create unique camera names.
      public: static unsigned int cameraCounter;

      /// \brief Deferred shading geometry buffer.
      public: Ogre::CompositorInstance *dsGBufferInstance;

      /// \brief Deferred shading merge compositor.
      public: Ogre::CompositorInstance *dsMergeInstance;

      /// \brief Deferred lighting geometry buffer.
      public: Ogre::CompositorInstance *dlGBufferInstance;

      /// \brief Deferred lighting merge compositor.
      public: Ogre::CompositorInstance *dlMergeInstance;

      /// \brief Screen space ambient occlusion compositor.
      public: Ogre::CompositorInstance *ssaoInstance;

      /// \brief Lens distortion model
      public: DistortionPtr distortion;

      /// \brief Queue of move positions.
      public: std::deque<std::pair<ignition::math::Pose3d, double> >
              moveToPositionQueue;

      /// \brief Render period.
      public: common::Time renderPeriod;

      /// \brief Communication Node
      public: transport::NodePtr node;

      /// \brief Subscribe to camera command topic
      public: transport::SubscriberPtr cmdSub;

      /// \def CameraCmdMsgs_L
      /// \brief List for holding camera command messages.
      typedef std::list<boost::shared_ptr<msgs::CameraCmd const> >
        CameraCmdMsgs_L;

      /// \brief List of camera cmd messages.
      public: CameraCmdMsgs_L commandMsgs;

      /// \brief Mutex to lock the various message buffers.
      public: std::mutex receiveMutex;

      /// \brief If set to true, the position of the camera is static.
      public: bool trackIsStatic;

      /// \brief If set to true, the camera inherits the yaw rotation of the
      /// tracked model.
      public: bool trackInheritYaw;

      /// \brief If set to true, the position of the camera is relative to the
      /// tracked model, otherwise it's relative to the world origin. In either
      /// case, the track position is expressed in the world frame.
      public: bool trackUseModelFrame;

      /// \brief Position of the camera when tracking a model.
      public: ignition::math::Vector3d trackPos;

      /// \brief Minimum distance between the camera and tracked model.
      public: double trackMinDistance;

      /// \brief Maximum distance between the camera and tracked model.
      public: double trackMaxDistance;

      /// \brief Video encoder.
      public: common::VideoEncoder videoEncoder;

      /// \brief If set to true, the camera yaws around a fixed axis.
      public: bool yawFixed;

      /// \brief Fixed axis to yaw around.
      public: ignition::math::Vector3d yawFixedAxis;
    };
  }
}
#endif
