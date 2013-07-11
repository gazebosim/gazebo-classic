/*
 * Copyright 2012 Open Source Robotics Foundation
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
#ifndef _MODEL_MANIPULATOR_HH_
#define _MODEL_MANIPULATOR_HH_

#include <string>

#include "gazebo/common/MouseEvent.hh"
#include "gazebo/common/KeyEvent.hh"

#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/common/SingletonT.hh"

namespace gazebo
{
  namespace gui
  {
    /// \class ModelManipulator ModelManipulator.hh gui/Gui.hh
    /// \brief Manipulator tool for translating/rotating/scaling models and
    /// links
    class ModelManipulator : public SingletonT<ModelManipulator>
    {
      /// \brief Constructor
      private: ModelManipulator();

      /// \brief Destructor
      private: virtual ~ModelManipulator();

      public: void Init();

      public: void SetManipulationMode(const std::string &_mode);

      public: void SetSelectionMode(const std::string &_mode);

      /// \brief Process an object translate mouse press event.
      public: void OnMousePressEvent(const common::MouseEvent &_event);

      /// \brief Process an object translate mouse move event.
      public: void OnMouseMoveEvent(const common::MouseEvent &_event);

      /// \brief Process an object translate mouse release event.
      public: void OnMouseReleaseEvent(const common::MouseEvent &_event);

      public: void OnKeyPressEvent(const common::KeyEvent &_event);

      public: void OnKeyReleaseEvent(const common::KeyEvent &_event);

      /// \brief Rotate entity.
      /// \param[in] _vis Visual representing the entity.
      /// \param[in] _local True to apply rotation in local frame.
      public: void RotateEntity(rendering::VisualPtr &_vis,
          const math::Vector3 &_axis,
          bool _local = false);

      /// \brief Translate entity.
      /// \param[in] _vis Visual representing the entity.
      /// \param[in] _local True to apply translation in local frame.
      public: void TranslateEntity(rendering::VisualPtr &_vis,
          const math::Vector3 &_axis,
          bool _local = false);

      /// \brief Scale entity.
      /// \param[in] _vis Visual representing the entity.
      /// \param[in] _local True to apply scaling in local frame.
      public: void ScaleEntity(rendering::VisualPtr &_vis,
          const math::Vector3 &_axis,
          bool _local = false);

      /// \brief Set the visual being moved, which will highlight the
      /// visual
      private: void SetMouseMoveVisual(rendering::VisualPtr _vis);

      /// \brief Publish visual's pose to the server
      /// \param[in] _vis Pointer to the visual whose pose is to be published.
      private: void PublishVisualPose(rendering::VisualPtr _vis);

      /// \brief Publish visual's scale to the server
      /// \param[in] _vis Pointer to the visual whose scale is to be published.
      private: void PublishVisualScale(rendering::VisualPtr _vis);

      private: rendering::SelectionObjPtr selectionObj;

      private: std::string manipMode;
      private: std::string selectionMode;

      private: math::Pose mouseMoveVisStartPose;

      private: rendering::VisualPtr selectedVis, mouseMoveVis;

      private: transport::NodePtr node;
      private: transport::PublisherPtr modelPub;
      private: transport::PublisherPtr lightPub;

      private: rendering::UserCameraPtr userCamera;
      private: rendering::ScenePtr scene;

      private: common::MouseEvent mouseEvent;

      private: common::KeyEvent keyEvent;

      private: bool initialized;

      private: math::Vector3 mouseVisualScale;

      /// \brief This is a singleton class.
      private: friend class SingletonT<ModelManipulator>;
    };
  }
}
#endif
