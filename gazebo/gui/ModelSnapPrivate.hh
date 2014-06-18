/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef _MODEL_SNAP_PRIVATE_HH_
#define _MODEL_SNAP_PRIVATE_HH_

#include <string>

#include "gazebo/common/MouseEvent.hh"
#include "gazebo/common/KeyEvent.hh"

#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the ModelSnap class
    class ModelSnapPrivate
    {
      /// \brief Transportation node.
      public: transport::NodePtr node;

      /// \brief Model publisher that publishes model pose to the server.
      public: transport::PublisherPtr modelPub;

      /// \brief Pointer to the user camera.
      public: rendering::UserCameraPtr userCamera;

      /// \brief Pointer to the scene where models are in.
      public: rendering::ScenePtr scene;

      /// \brief Ray query for selecting a surface of an entity.
      public: rendering::RayQueryPtr rayQuery;

      /// \brief Current mouse event.
      public: common::MouseEvent mouseEvent;

      /// \brief Current key event.
      public: common::KeyEvent keyEvent;

      /// \brief The current visual attached to the mouse.
      public: rendering::VisualPtr mouseMoveVis;

      /// \brief True if the model align tool is initialized.
      public: bool initialized;

      /// \brief Vertices of a mesh triangle used as the basis for alignment.
      public: std::vector<math::Vector3> selectedTriangle;

      /// \brief Currently selected visual.
      public: rendering::VisualPtr selectedVis;
    };
  }
}
#endif
