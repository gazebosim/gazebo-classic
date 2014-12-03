/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _GRID_HH_
#define _GRID_HH_

#include <stdint.h>
#include <vector>
#include <string>

// TODO: remove this line
#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Color.hh"
#include "gazebo/util/system.hh"

namespace Ogre
{
  class ManualObject;
  class SceneNode;
  class Any;
}

namespace gazebo
{
  namespace rendering
  {
    class Scene;

    /// \addtogroup gazebo_rendering
    /// \{

    ///  \class Grid Grid.hh rendering/rendering.hh
    ///  \brief Displays a grid of cells, drawn with lines
    ///
    ///  Displays a grid of cells, drawn with lines.  A grid with an
    ///  identity orientation is drawn along the XY plane.
    class GZ_RENDERING_VISIBLE Grid
    {
      /// \brief Constructor
      ///
      /// \param[in] _scene The scene this object is part of
      /// \param[in] _cellCount The number of cells to draw
      /// \param[in] _cellLength The size of each cell
      /// \param[in] _lineWidth The width of the lines to use
      /// \param[in] _color The color of the grid
      public: Grid(Scene *_scene, uint32_t _cellCount, float _cellLength,
                   float _lineWidth, const common::Color &_color);

      /// \brief Destructor
      public: ~Grid();

      /// \brief Initialize the grid
      public: void Init();

      /// \brief Enable or disable the grid.
      ///
      /// \param[in] _enable Set to true to view the grid, false to make
      /// invisible.
      public: void Enable(bool _enable);

      /// \brief Get the Ogre scene node associated with this grid
      /// \return The Ogre scene node associated with this grid
      public: Ogre::SceneNode *GetSceneNode() { return this->sceneNode; }

      /// \brief Sets user data on all ogre objects we own
      /// \param[in] _data The user data
      public: void SetUserData(const Ogre::Any &_data);

      /// \brief Sets the color of the grid
      /// \param[in] _color The grid color
      public: void SetColor(const common::Color &_color);

      /// \brief Return the grid color
      /// \return The grid color
      public: common::Color GetColor() const {return this->color;}

      /// \brief Set the number of cells
      /// \param[in] _count The number of cells
      public: void SetCellCount(uint32_t _count);

      /// \brief Get the number of cells
      public: uint32_t GetCellCount() const {return this->cellCount;}

      /// \brief Set the cell length
      /// \param[in] _len The cell length
      public: void SetCellLength(float _len);

      /// \brief Get the cell length
      /// \return The cell length
      public: float GetCellLength() const {return this->cellLength;}

      /// \brief Set the line width
      /// \param[in] _width The width of the grid
      public: void SetLineWidth(float _width);

      /// \brief Get the width of the grid line
      /// \return The line width
      public: float GetLineWidth() const {return this->lineWidth;}

      /// \brief Set the height of the grid
      /// \param[in] _count Grid height
      public: void SetHeight(uint32_t _count);

      /// \brief Get the height of the grid
      /// \return The height
      public: uint32_t GetHeight() const {return this->height;}

      private: void Create();

      private: Ogre::SceneNode *sceneNode;
      private: Ogre::ManualObject *manualObject;

      private: Ogre::MaterialPtr material;

      private: unsigned int cellCount;
      private: float cellLength;
      private: float lineWidth;
      private: common::Color color;
      private: float heightOffset;

      private: std::string name;
      private: unsigned int height;

      private: Scene *scene;
    };
    /// \}
  }
}
#endif
