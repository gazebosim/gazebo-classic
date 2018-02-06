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
// This was leveraged from rviz.

#ifndef GAZEBO_RENDERING_GRID_HH_
#define GAZEBO_RENDERING_GRID_HH_

#include <memory>
#include <string>
#include <vector>

#include <ignition/math/Color.hh>

#include "gazebo/util/system.hh"

namespace Ogre
{
  class SceneNode;
  class Any;
}

namespace gazebo
{
  namespace common
  {
    class Color;
  }

  namespace rendering
  {
    class Scene;

    // Forward declare provate data
    class GridPrivate;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class Grid Grid.hh rendering/rendering.hh
    /// \brief Displays a grid of cells, drawn with lines
    ///
    /// Displays a grid of cells, drawn with lines.  A grid with an
    /// identity orientation is drawn along the XY plane.
    class GZ_RENDERING_VISIBLE Grid
    {
      /// \brief Constructor
      ///
      /// \param[in] _scene The scene this object is part of
      /// \param[in] _cellCount The number of cells to draw
      /// \param[in] _cellLength The size of each cell
      /// \param[in] _lineWidth The width of the lines to use
      /// \param[in] _color The color of the grid
      /// \deprecated Use constructor which accepts ignition::math::Color.
      public: Grid(Scene *_scene, const uint32_t _cellCount,
          const float _cellLength, const float _lineWidth,
          const common::Color &_color) GAZEBO_DEPRECATED(9.0);

      /// \brief Constructor
      ///
      /// \param[in] _scene The scene this object is part of
      /// \param[in] _cellCount The number of cells to draw
      /// \param[in] _cellLength The size of each cell
      /// \param[in] _color The color of the grid
      public: Grid(Scene *_scene, const uint32_t _cellCount,
          const float _cellLength, const ignition::math::Color &_color);

      /// \brief Destructor
      public: ~Grid();

      /// \brief Initialize the grid
      public: void Init();

      /// \brief Enable or disable the grid.
      ///
      /// \param[in] _enable Set to true to view the grid, false to make
      /// invisible.
      public: void Enable(const bool _enable);

      /// \brief Get thevisual associated with this grid
      /// \return The visual associated with this grid
      public: VisualPtr GridVisual() const;

      /// \brief Sets user data on all ogre objects we own
      /// \param[in] _data The user data
      public: void SetUserData(const Ogre::Any &_data);

      /// \brief Sets the color of the grid
      /// \param[in] _color The grid color
      /// \deprecated Use function which accepts ignition::math::Color.
      public: void SetColor(const common::Color &_color) GAZEBO_DEPRECATED(9.0);

      /// \brief Sets the color of the grid
      /// \param[in] _color The grid color
      public: void SetColor(const ignition::math::Color &_color);

      /// \brief Return the grid color
      /// \return The grid color
      public: ignition::math::Color Color() const;

      /// \brief Set the number of cells
      /// \param[in] _count The number of cells
      public: void SetCellCount(const uint32_t _count);

      /// \brief Get the number of cells
      /// \return The number of cells in each direction.
      public: uint32_t CellCount() const;

      /// \brief Set the cell length
      /// \param[in] _len The cell length
      public: void SetCellLength(const float _len);

      /// \brief Get the cell length
      /// \return The cell length
      public: float CellLength() const;

      /// \brief Set the line width
      /// \param[in] _width The width of the grid
      /// \deprecated Grid lines are always 1px wide.
      public: void SetLineWidth(const float _width) GAZEBO_DEPRECATED(9.0);

      /// \brief Get the width of the grid line
      /// \return The line width
      /// \deprecated Grid lines are always 1px wide.
      public: float LineWidth() const GAZEBO_DEPRECATED(9.0);

      /// \brief Set the height of the grid
      /// \param[in] _count Grid height
      public: void SetHeight(const uint32_t _count);

      /// \brief Get the number of cells in the normal direction of the grid.
      /// \return The height
      public: uint32_t Height() const;

      /// \brief Set the height offset of the grid.
      /// \param[in] _count Grid height offset.
      public: void SetHeightOffset(const double _offset);

      /// \brief Get the height offset.
      /// \return The height offset.
      public: double HeightOffset() const;

      /// \brief Create the grid.
      private: void Create();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<GridPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
