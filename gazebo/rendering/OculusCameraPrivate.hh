/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef _GAZEBO_RENDERING_OCULUS_CAMERA_PRIVATE_HH_
#define _GAZEBO_RENDERING_OCULUS_CAMERA_PRIVATE_HH_

#include <OVR_CAPI.h>
#include <ignition/transport.hh>
#include "gazebo/util/system.hh"

namespace Ogre
{
  class Camera;
  class Viewport;
  class SceneManager;
}

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the Oculus Camera class
    class GZ_RENDERING_VISIBLE OculusCameraPrivate
    {
      /// \brief Constructor
      public: OculusCameraPrivate()
              : rightCamera(NULL),
                externalCamera(NULL),
                rightViewport(NULL),
                externalViewport(NULL),
                externalSceneManager(NULL),
                node(NULL),
                controlSub(NULL),
                ready(false),
                frameIndex(1),
                oculusTrackingWarned(false) {}

      /// \brief Ogre camera for the right Oculus screen.
      public: Ogre::Camera *rightCamera;

      /// \brief Camera in the external scene that render the distortion
      /// meshes.
      public: Ogre::Camera *externalCamera;

      /// \brief View port for the right camera.
      public: Ogre::Viewport *rightViewport;

      /// \brief View port for the external camera.
      public: Ogre::Viewport *externalViewport;

      /// \brief External scene manager. This holds the distortion meshes
      /// and the external camera.
      public: Ogre::SceneManager *externalSceneManager;

      /// \brief An Oculus Head-Mounted display.
      public: ovrHmd hmd;

      /// \brief Transport node for using gazebo pub/sub.
      public: transport::NodePtr node;

      /// \brief Subscriber used to receive updates on world_control topic.
      public: transport::SubscriberPtr controlSub;

      /// \brief Node for ignition transport communication.
      public: ignition::transport::Node ignNode;

      /// \brief True when Oculus is connected and ready to use.
      public: bool ready;

      /// \brief Right camera render texture.
      public: Ogre::TexturePtr renderTextureRight;

      /// \brief Left camera render texture.
      public: Ogre::TexturePtr renderTextureLeft;

      /// \brief Frame index used for oculus timing
      public: unsigned int frameIndex;

      /// \brief Flag used to prevent multiple messages from being
      /// displayed.
      public: bool oculusTrackingWarned;
    };
  }
}
#endif
