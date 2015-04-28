/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#ifndef _MODEL_ALIGN_PRIVATE_HH_
#define _MODEL_ALIGN_PRIVATE_HH_

#include <string>
#include <vector>
#include <map>

#include "gazebo/common/MouseEvent.hh"
#include "gazebo/common/KeyEvent.hh"

#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  namespace gui
  {
    /// \class ModelAlignPrivate ModelAlignPrivate.hh
    /// \brief Private data for the ModelAlign class
    class ModelAlignPrivate
    {
      /// \brief Transportation node.
      public: transport::NodePtr node;

      /// \brief Model publisher that publishes model pose to the server.
      public: transport::PublisherPtr modelPub;

      /// \brief Pointer to the user camera.
      public: rendering::UserCameraPtr userCamera;

      /// \brief Pointer to the scene where models are in.
      public: rendering::ScenePtr scene;

      /// \brief The last selected visual which will be used for alignment.
      public: rendering::VisualPtr targetVis;

      /// \brief True if the model align tool is initialized.
      public: bool initialized;

      /// \brief selected visuals.
      public: std::vector<rendering::VisualPtr> selectedVisuals;

      /// \brief A list of connections. Currently used only
      /// to get the align configuration event.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Original model pose used when user resets alignment.
      public: std::map<rendering::VisualPtr, math::Pose> originalVisualPose;
    };
  }
}
#endif
