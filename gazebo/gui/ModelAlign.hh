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
#ifndef GAZEBO_GUI_MODELALIGN_HH_
#define GAZEBO_GUI_MODELALIGN_HH_

#include <memory>
#include <string>
#include <vector>

#include <ignition/math/Box.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/KeyEvent.hh"
#include "gazebo/common/MouseEvent.hh"
#include "gazebo/common/SingletonT.hh"

#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class ModelAlignPrivate;

    /// \class ModelAlign ModelAlign.hh gui/Gui.hh
    /// \brief A gui tool for aligning models
    class GZ_GUI_VISIBLE ModelAlign : public SingletonT<ModelAlign>
    {
      /// \brief Constructor
      private: ModelAlign();

      /// \brief Destructor
      private: virtual ~ModelAlign();

      /// \brief Initialize the model alignment tool.
      public: void Init();

      /// \brief Clear the model alignment tool. This explicity cleans up the
      /// internal state of the singleton and prepares it for exit.
      public: void Clear();

      /// \brief Callback when a specific alignment configuration is set.
      /// \param[in] _visuals Visuals to be aligned.
      /// \param[in] _axis Axis of alignment: x, y, or z.
      /// \param[in] _config Either a configuration (min, center, max),
      /// or "reset" to restore the original pose.
      /// \param[in] _publish True to publish new alignment pose
      /// \param[in] _inverted True to invert alignment direction.
      public: void AlignVisuals(std::vector<rendering::VisualPtr> _visuals,
          const std::string &_axis, const std::string &_config,
          const std::string &_target, const bool _publish = true,
          const bool _inverted = false);

      /// \brief Get the minimum and maximum values of a list of vertices.
      /// \param[in] _vertices A list of input vertices.
      /// \param[out] _min Minimum x, y, z values.
      /// \param[out] _max Maximum x, y, z values.
      private: void MinMax(
          const std::vector<ignition::math::Vector3d> &_vertices,
          ignition::math::Vector3d &_min,
          ignition::math::Vector3d &_max);

      /// \brief Transform a bounding box to the world space.
      /// \param[in] _bbox Input bounding box.
      /// \param[in] _worldPose Pose used to tranform the bounding box.
      /// \param[out] _vertices Vertices of the tranformed bounding box in
      /// world coordinates.
      private: void Transform(const ignition::math::Box &_bbox,
          const ignition::math::Pose3d &_worldPose,
          std::vector<ignition::math::Vector3d> &_vertices);

      /// \brief Change the transparency of the visual's leaf children to
      /// indicate a highlighted state or not. Must do it for each leaf as
      /// they might have different transparencies.
      /// \param[in] _vis Visual to be highlighted.
      /// \param[in] _highlight Whether to highlight or not.
      private: void SetHighlighted(const rendering::VisualPtr &_vis,
          const bool _highlight);

      /// \brief This is a singleton class.
      private: friend class SingletonT<ModelAlign>;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<ModelAlignPrivate> dataPtr;
    };
  }
}
#endif
