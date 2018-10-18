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
#ifndef _GAZEBO_GUI_GLWIDGET_PRIVATE_HH_
#define _GAZEBO_GUI_GLWIDGET_PRIVATE_HH_

#include <mutex>
#include <string>
#include <vector>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/KeyEvent.hh"
#include "gazebo/common/MouseEvent.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/LightMaker.hh"
#include "gazebo/gui/ModelMaker.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    /// \class GLWidgetPrivate GLWidgetPrivate.hh
    /// \brief Private data for the GLWidget class.
    class GLWidgetPrivate
    {
      /// \brief Window identifier.
      public: int windowId;

      /// \brief Pointer to the user camera.
      public: rendering::UserCameraPtr userCamera;

      /// \brief Scene pointer.
      public: rendering::ScenePtr scene;

      /// \brief Render frame widget.
      public: QFrame *renderFrame;

      /// \brief The mouse event.
      public: common::MouseEvent mouseEvent;

      /// \brief The most recent keyboard event.
      public: common::KeyEvent keyEvent;

      /// \brief Store multiple connections.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Pointer to the current maker.
      public: EntityMaker *entityMaker;

      /// \brief Model maker.
      public: ModelMaker modelMaker;

      /// \brief Point light maker
      public: PointLightMaker pointLightMaker;

      /// \brief Spot light maker
      public: SpotLightMaker spotLightMaker;

      /// \brief Directional light maker
      public: DirectionalLightMaker directionalLightMaker;

      /// \brief Light maker
      public: LightMaker lightMaker;

      /// \brief A list of selected visuals.
      public: std::vector<rendering::VisualPtr> selectedVisuals;

      /// \brief Indicates how deep into the model to select.
      public: GLWidget::SelectionLevels selectionLevel;

      /// \brief The transport node.
      public: transport::NodePtr node;

      /// \brief Publishes information about user selections.
      public: transport::PublisherPtr selectionPub;

      /// \brief Susbcribes to the requests.
      public: transport::SubscriberPtr requestSub;

      /// \brief Text key.
      public: std::string keyText;

      /// \brief Key modifiers.
      public: Qt::KeyboardModifiers keyModifiers;

      /// \brief Gazebo mode state (select, translate, scale, ...).
      public: std::string state;

      /// \brief Name of entity that is being copied.
      public: std::string copyEntityName;

      /// \brief True if the model editor is up, false otherwise
      public: bool modelEditorEnabled;

      /// \brief Mutex to protect selectedVisuals array.
      public: std::mutex selectedVisMutex;

      /// \brief Timer used to update the render window.
      public: QTimer *updateTimer = nullptr;
    };
  }
}
#endif
