/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_GUI_GRID_LINES_HH_
#define _GAZEBO_GUI_GRID_LINES_HH_

#include <memory>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class GridLinesPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class GridLines GridLines.hh
    /// \brief 2D grid lines.
    class GZ_GUI_VISIBLE GridLines : public  QGraphicsItem
    {
      /// \brief Constructor
      /// \param[in] _width Width of grid lines in pixels.
      /// \param[in] _height Height of grid lines in pixels.
      public: GridLines(int _width, int _height);

      /// \brief Destructor
      public: ~GridLines();

      /// \return Set the size of grid lines in pixels.
      /// \param[in] _width Width of grid lines in pixels.
      /// \param[in] _height Height of grid lines in pixels.
      public : void SetSize(int _width, int _height);

      /// \brief Get the bounding box of the grid lines
      /// \return The bounding box of the grid lines
      private: virtual QRectF boundingRect() const;

      /// \brief Qt paint function for drawing the grid lines.
      /// \param[in] _painter Qt painter object.
      /// \param[in] _option Qt style options for the item.
      /// \param[in] _widget Qt widget being painted on.
      private: virtual void paint(QPainter *_painter,
        const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      /// \brief Qt mouse move event
      /// \param[in] _event Qt mouse event
      private: virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse press event
      /// \param[in] _event Qt mouse event
      private: virtual void mousePressEvent(QGraphicsSceneMouseEvent *_event);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<GridLinesPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
