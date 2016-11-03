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
#ifndef GAZEBO_GUI_MODELSNAP_HH_
#define GAZEBO_GUI_MODELSNAP_HH_

#include <memory>
#include <vector>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Triangle3.hh>

#include "gazebo/common/SingletonT.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class MouseEvent;
  }

  namespace math
  {
    class Pose;
    class Quaternion;
    class Vector3;
  }

  namespace gui
  {
    class ModelSnapPrivate;

    /// \class ModelSnap ModelSnap.hh gui/Gui.hh
    /// \brief A gui tool for snapping one model to another.
    class GZ_GUI_VISIBLE ModelSnap : public SingletonT<ModelSnap>
    {
      /// \brief Constructor
      private: ModelSnap();

      /// \brief Destructor
      private: virtual ~ModelSnap();

      /// \brief Initialize the model snapping tool.
      public: void Init();

      /// \brief Clear the model snapping tool. This explicity cleans up the
      /// internal state of the singleton and prepares it for exit.
      public: void Clear();

      /// \brief Reset the model snapping tool.
      public: void Reset();

      /// \brief Clean up the model snap tool.
      public: void Fini();

      /// \brief Calculate the translation and rotation needed to snap the
      /// centroid of a mesh triangle of a visual to another, taking into
      /// account any pose offsets.
      /// \param[in] _triangleSrc vertices of target triangle being snapped to.
      /// \param[in] _triangleDest vertices of the other triangle that will be
      /// moved.
      /// \param[in] _visualSrc Visual being moved by the snap action.
      /// \deprecated See function that accepts ignition::math parameters.
      public: void Snap(const std::vector<math::Vector3> &_triangleSrc,
          const std::vector<math::Vector3> &_triangleDest,
          rendering::VisualPtr _visualSrc) GAZEBO_DEPRECATED(8.0);

      /// \brief Calculate the translation and rotation needed to snap the
      /// centroid of a mesh triangle of a visual to another, taking into
      /// account any pose offsets.
      /// \param[in] _triangleSrc vertices of target triangle being snapped to.
      /// \param[in] _triangleDest vertices of the other triangle that will be
      /// moved.
      /// \param[in] _visualSrc Visual being moved by the snap action.
      public: void Snap(
          const ignition::math::Triangle3d &_triangleSrc,
          const ignition::math::Triangle3d &_triangleDest,
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
      /// \deprecated See function that accepts ignition::math parameters.
      public: void GetSnapTransform(
          const std::vector<math::Vector3> &_triangleSrc,
          const std::vector<math::Vector3> &_triangleDest,
          const math::Pose &_poseSrc, math::Vector3 &_trans,
          math::Quaternion &_rot) GAZEBO_DEPRECATED(8.0);

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
      public: static void SnapTransform(
          const ignition::math::Triangle3d &_triangleSrc,
          const ignition::math::Triangle3d &_triangleDest,
          const ignition::math::Pose3d &_poseSrc,
          ignition::math::Vector3d &_trans,
          ignition::math::Quaterniond &_rot);

      /// \brief Process an object translate mouse press event.
      /// \param[in] _event Mouse event.
      public: void OnMousePressEvent(const common::MouseEvent &_event);

      /// \brief Process an object translate mouse move event.
      /// \param[in] _event Mouse event.
      public: void OnMouseMoveEvent(const common::MouseEvent &_event);

      /// \brief Process an object translate mouse release event.
      /// \param[in] _event Mouse event.
      public: void OnMouseReleaseEvent(const common::MouseEvent &_event);

      /// \brief Publish visual's pose to the server
      /// \param[in] _vis Pointer to the visual whose pose is to be published.
      private: void PublishVisualPose(rendering::VisualPtr _vis);

      /// \brief Update the visual representation of the snap spot.
      private: void Update();

      /// \brief This is a singleton class.
      private: friend class SingletonT<ModelSnap>;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<ModelSnapPrivate> dataPtr;
    };
  }
}
#endif
