/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_GUI_TAPEMEASURE_PRIVATE_HH_
#define _GAZEBO_GUI_TAPEMEASURE_PRIVATE_HH_

#include <string>
#include <vector>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/MouseEvent.hh"
#include "gazebo/common/KeyEvent.hh"

#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  namespace gui
  {
    /// \class TapeMeasure TapeMeasure.hh
    /// \brief Private data for the TapeMeasure class
    class TapeMeasurePrivate
    {
      /// \brief Pointer to the user camera.
      public: rendering::UserCameraPtr userCamera;

      /// \brief Pointer to the scene where models are in.
      public: rendering::ScenePtr scene;

      /// \brief Ray query for selecting a surface of an entity.
      public: rendering::RayQueryPtr rayQuery;

      /// \brief Current mouse event.
      public: common::MouseEvent mouseEvent;

      /// \brief True if the model align tool is initialized.
      public: bool initialized;

      /// \brief Vertices of a mesh triangle used as the basis for alignment.
      public: ignition::math::Vector3d selectedVertex;

      /// \brief Vertices of a mesh triangle being hovered.
      public: ignition::math::Vector3d hoverVertex;

      /// \brief Currently selected visual.
      public: rendering::VisualPtr selectedVis;

      /// \brief Currently hovered visual.
      public: rendering::VisualPtr hoverVis;

      /// \brief Currently hovered visual.
      public: rendering::VisualPtr highlightVis;

      /// \brief Connection for the render event.
      public: event::ConnectionPtr renderConnection;

      /// \brief Mutex to protect the selected triangle vertices.
      public: boost::recursive_mutex *updateMutex;

      /// \brief A visual to represent the snap spot.
      public: rendering::VisualPtr lineVisual;

      /// \brief Lines to highlight the selected triangle.
      public: rendering::DynamicLines *dynamicLines;

      /// \brief A visual to represent the hovered area.
      public: std::vector<rendering::VisualPtr> pointVisuals;

      /// \brief A variable to indicate the selected triangle has changed.
      public: bool selectedVertexDirty;

      /// \brief A variable to indicate the hovered triangle has changed.
      public: bool hoverVertexDirty;
    };
  }
}
#endif
