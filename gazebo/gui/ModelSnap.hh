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
#ifndef _GAZEBO_MODEL_SNAP_HH_
#define _GAZEBO_MODEL_SNAP_HH_

#include <string>
#include <vector>

#include "gazebo/common/MouseEvent.hh"
#include "gazebo/common/KeyEvent.hh"

#include "gazebo/math/Pose.hh"
#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/common/SingletonT.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class ModelSnapPrivate;

    /// \class ModelSnap ModelSnap.hh gui/Gui.hh
    /// \brief A gui tool for snapping one model to another.
    class GAZEBO_VISIBLE ModelSnap : public SingletonT<ModelSnap>
    {
      /// \brief Constructor
      private: ModelSnap();

      /// \brief Destructor
      private: virtual ~ModelSnap();

      /// \brief Initialize the model snapping tool.
      public: void Init();

      /// \brief Reset the model snapping tool.
      public: void Reset();

      /// \brief Calculate the translation and rotation needed to snap the
      /// centroid of a mesh triangle of a visual to another, taking into
      /// account any pose offsets.
      /// \param[in] _triangleSrc vertices of target triangle being snapped to.
      /// \param[in] _triangleDest vertices of the other triangle that will be
      /// moved.
      /// \param[in] _visualSrc Visual being moved by the snap action.
      public: void Snap(const std::vector<math::Vector3> &_triangleSrc,
          const std::vector<math::Vector3> &_triangleDest,
          rendering::VisualPtr _visualSrc);

      /// \brief Calculate the translation and rotation needed to snap the
      /// centroid of a mesh triangle of a visual to another, taking into
      /// account any pose offsets.
      /// \param[in] _triangleSrc vertices of target triangle being snapped to.
      /// \param[in] _triangleDest vertices of the other triangle that will be
      /// moved.
      /// \param[in] _poseSrc Pose offset of triangleB relative to its model
      /// visual.
      /// \param[out] _trans Translation output.
      /// \param[out] _rotation Rotation output.
      public: void GetSnapTransform(
          const std::vector<math::Vector3> &_triangleSrc,
          const std::vector<math::Vector3> &_triangleDest,
          const math::Pose &_poseSrc, math::Vector3 &_trans,
          math::Quaternion &_rot);

      /// \brief Process an object translate mouse press event.
      /// \param[in] _event Mouse event.
      public: void OnMousePressEvent(const common::MouseEvent &_event);

      /// \brief Process an object translate mouse move event.
      /// \param[in] _event Mouse event.
      public: void OnMouseMoveEvent(const common::MouseEvent &_event);

      /// \brief Process an object translate mouse release event.
      /// \param[in] _event Mouse event.
      public: void OnMouseReleaseEvent(const common::MouseEvent &_event);

      /// \brief Set the snap level.
      /// \param[in] _snapLevel Choose "model" to snap models to each other or
      /// "link" to snap links within a model.
      public: void SetSnapLevel(const std::string &_snapLevel);

      /// \brief Publish visual's pose to the server
      /// \param[in] _vis Pointer to the visual whose pose is to be published.
      private: void PublishVisualPose(rendering::VisualPtr _vis);

      /// \brief Update the visual representation of the snap spot.
      private: void Update();

      /// \brief This is a singleton class.
      private: friend class SingletonT<ModelSnap>;

      /// \internal
      /// \brief Pointer to private data.
      private: ModelSnapPrivate *dataPtr;
    };
  }
}
#endif
