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

#include "gazebo/gui/qt.h"
#include "gazebo/gui/model_editor/EditorItem.hh"

namespace gazebo
{
  namespace gui
  {
    class CornerGrabber;

    class EditorItem;

    class LineSegmentItem;

    class PolylineItem : public EditorItem, public QGraphicsPathItem
    {
      public: PolylineItem(const QPointF &_start, const QPointF &_end);

      public: ~PolylineItem();

      public: void AddPoint(const QPointF &_point);

      public: void PopEndPoint();

      public: unsigned int GetVertexCount() const;

      public: unsigned int GetSegmentCount() const;

      public: void SetVertexPosition(unsigned int _index, const QPointF &_pos);

      public: void TranslateVertex(unsigned int _index, const QPointF &_trans);

      public: LineSegmentItem *GetSegment(unsigned int _index) const;

      public: void ShowCorners(bool _show);

      public: void SetThickness(double _thickness);

      public: void SetPosition(const QPointF &_pos);

      public: void Update();

      private: void UpdatePath();

      private: void UpdatePathAtIndex(unsigned int _index, const QPointF &_pos);

      private: void UpdatePathAt(unsigned int _index, const QPointF &_pos);

      private: void AppendToPath(const QPointF &_point);

      private: bool sceneEventFilter(QGraphicsItem * watched,
        QEvent *_event);

      private: virtual bool cornerEventFilter(CornerGrabber *_corner,
          QEvent *_event);

      private: virtual bool segmentEventFilter(LineSegmentItem *_item,
          QEvent *_event);

      private: void hoverEnterEvent(QGraphicsSceneHoverEvent *_event);

      private: void hoverMoveEvent(QGraphicsSceneHoverEvent *_event);

      private: void hoverLeaveEvent(QGraphicsSceneHoverEvent *_event);

      private: void mouseReleaseEvent(QGraphicsSceneMouseEvent *_event);

      private: void mouseMoveEvent(QGraphicsSceneMouseEvent *_event);

      private: void mousePressEvent(QGraphicsSceneMouseEvent *_event);

      private: QVariant itemChange(GraphicsItemChange _change,
        const QVariant &_value);

      private: void DrawBoundingBox(QPainter *_painter);

      private: void paint(QPainter *_painter,
          const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      protected: std::vector<CornerGrabber*> corners;

      protected: std::vector<LineSegmentItem*> segments;

      protected: QPointF segmentMouseMove;

      private: QPointF origin;

      private: QPointF location;

      private: int gridSpace;

      private: int cornerWidth;

      private: int cornerHeight;

      private: double lineThickness;
    };
  }
}

#endif
