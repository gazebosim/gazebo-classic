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

#ifndef _GRID_LINES_HH_
#define _GRID_LINES_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class GridLines : public  QGraphicsItem
    {
      public : GridLines(int _width, int _height);

      public : ~GridLines();

      public : void HandleWindowSizeChanged(int _w, int _h);

      private: virtual QRectF boundingRect() const;

      private: virtual void paint (QPainter *_painter,
        const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      private: virtual void mouseMoveEvent (QGraphicsSceneMouseEvent *_event);

      private: virtual void mouseMoveEvent(QGraphicsSceneDragDropEvent *_event);

      private: virtual void mousePressEvent (QGraphicsSceneMouseEvent *_event);

      private: virtual void mousePressEvent(QGraphicsSceneDragDropEvent *_event);

      private: int width;

      private: int height;

      private: int space;
    };
  }
}
#endif
