/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_MODELMANIPULATORPRIVATE_HH_
#define GAZEBO_GUI_MODELMANIPULATORPRIVATE_HH_

#include <string>
#include <vector>

#include <ignition/math/Box.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/MouseEvent.hh"
#include "gazebo/common/KeyEvent.hh"

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the ModelManipulator class
    class ModelManipulatorPrivate
    {
      /// \brief Selection object which users can interact with to manipulate
      /// the model.
      public: rendering::SelectionObjPtr selectionObj;

      /// \brief The current manipulation mode.
      public: std::string manipMode;

      /// \brief Keep track of the mouse start pose before a move action.
      public: ignition::math::Pose3d mouseMoveVisStartPose;

      /// \brief Keep track of the mouse start screen position.
      public: ignition::math::Vector2i mouseStart;

      /// \brief The current visual attached to the mouse.
      public: rendering::VisualPtr mouseMoveVis;

      /// \brief Transportation node.
      public: transport::NodePtr node;

      /// \brief Publish user command messages for the server to place in the
      /// undo queue.
      public: transport::PublisherPtr userCmdPub;

      /// \brief Pointer to the user camera.
      public: rendering::UserCameraPtr userCamera;

      /// \brief Pointer to the scene where models are in.
      public: rendering::ScenePtr scene;

      /// \brief Current mouse event.
      public: common::MouseEvent mouseEvent;

      /// \brief Current key event.
      public: common::KeyEvent keyEvent;

      /// \brief True if the model manipulator is initialized.
      public: bool initialized = false;

      /// \brief Scale of the visual attached to the mouse.
      public: ignition::math::Vector3d mouseVisualScale;

      /// \brief Scale of all the child visuals attached to the mouse.
      public: std::vector<ignition::math::Vector3d> mouseChildVisualScale;

      /// \brief Bounding box of the visual attached to the mouse (for scaling).
      public: ignition::math::Box mouseVisualBbox;

      /// \brief True to manipulate model in global frame.
      public: bool globalManip = false;

      /// \brief True when a model is being manipulated, false otherwise.
      public: bool transparent = false;
    };
  }
}
#endif
