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

#ifndef _POLYLINE_ITEM_HH_
#define _POLYLINE_ITEM_HH_

#include <gui/qt.h>

namespace gazebo
{
  namespace gui
  {
    class CornerGrabber;

    class PolylineItem : public QGraphicsPathItem
    {
      public: PolylineItem(QPointF _start, QPointF _end);

      public: ~PolylineItem();

      public: void AddPoint(QPointF _point);

      public: void PopEndPoint();

//      public: void RemovePoint(unsigned int _index);

      public: unsigned int GetCount();

      public: void SetVertexPosition(unsigned int _index, QPointF _pos);

      public: void TranslateVertex(unsigned int _index, QPointF _trans);

      private: void UpdatePath();

      private: void UpdatePathAtIndex(unsigned int _index, QPointF _pos);

      private: void UpdatePathAt(unsigned int _index, QPointF _pos);

      private: void AppendToPath(QPointF _point);

      private: bool sceneEventFilter(QGraphicsItem * watched,
        QEvent *_event) ;

      private: void hoverEnterEvent(QGraphicsSceneHoverEvent *_event);

      private: void hoverLeaveEvent(QGraphicsSceneHoverEvent *_event);

      private: void mouseReleaseEvent(QGraphicsSceneMouseEvent *_event);

      private: void mouseMoveEvent(QGraphicsSceneMouseEvent *_event);

      private: void mousePressEvent(QGraphicsSceneMouseEvent *_event);

      private: QPointF origin;

      private: QPointF location;

      private: QPointF dragStart;

      private: int gridSpace;

      private: std::vector<CornerGrabber*> corners;

      private: int cornerWidth;

      private: int cornerHeight;
    };
  }
}

#endif
