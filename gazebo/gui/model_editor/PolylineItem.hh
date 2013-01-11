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

#include <vector>
#include "gazebo/gui/qt.h"
#include "gazebo/gui/model_editor/EditorItem.hh"

namespace gazebo
{
  namespace gui
  {
    class GrabberHandle;

    class EditorItem;

    class LineSegmentItem;

    class PolylineItem : public EditorItem, public QGraphicsPathItem
    {

      /// \brief Constructor
      /// param[in] _start Start position of the polyline item in pixel
      /// coordinates.
      /// param[in] _start End position of the polyline item in pixel
      /// coordinates.
      public: PolylineItem(const QPointF &_start, const QPointF &_end);

      /// \brief Destructor
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

      public: void ClosePath();

      public: bool IsClosed() const;

      public: void Update();

      private: void UpdatePath();

      private: void UpdatePathAtIndex(unsigned int _index, const QPointF &_pos);

      private: void UpdatePathAt(unsigned int _index, const QPointF &_pos);

      private: void AppendToPath(const QPointF &_point);

      private: bool sceneEventFilter(QGraphicsItem * watched,
        QEvent *_event);

      private: virtual bool grabberEventFilter(GrabberHandle *_grabber,
          QEvent *_event);

      private: virtual bool segmentEventFilter(LineSegmentItem *_item,
          QEvent *_event);

      /// \brief Qt mouse hover enter event.
      /// \param[in] _event Qt mouse hover event.
      private: void hoverEnterEvent(QGraphicsSceneHoverEvent *_event);

      /// \brief Qt mouse hover move event.
      /// \param[in] _event Qt mouse hover event.
      private: void hoverMoveEvent(QGraphicsSceneHoverEvent *_event);

      /// \brief Qt mouse hover leave event.
      /// \param[in] _event Qt mouse hover event.
      private: void hoverLeaveEvent(QGraphicsSceneHoverEvent *_event);

      /// \brief Qt mouse move event.
      /// \param[in] _event Qt mouse event.
      private: void mouseMoveEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse press event.
      /// \param[in] _event Qt mouse event.
      private: void mousePressEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse release event.
      /// \param[in] _event Qt mouse event.
      private: void mouseReleaseEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse press event during drag and drop.
      /// \param[in] _event Qt mouse drag and drop event.
      private: void mousePressEvent(QGraphicsSceneDragDropEvent *_event);

      /// \brief Qt mouse move event during drag and drop.
      /// \param[in] _event Qt mouse drag and drop event.
      private: void mouseMoveEvent(QGraphicsSceneDragDropEvent *_event);

      /// \brief React to item changes notified by Qt.
      /// \param[in] _change Qt change type, e.g. selected change, position
      /// change.
      /// \param[in] _value Value to be changed to.
      private: QVariant itemChange(GraphicsItemChange _change,
        const QVariant &_value);

      /// \brief Qt paint function for drawing the polyline.
      /// \param[in] _painter Qt painter object.
      /// \param[in] _painter Qt style options for the item.
      /// \param[in] _painter Qt widget being painted on.
      private: void paint(QPainter *_painter,
          const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      /// \brief Draw bounding box
      /// \param[in] _painter Qt painter object.
      protected: void DrawBoundingBox(QPainter *_painter);

      /// \brief A list of grabber handles for this item, one on each vertex.
      protected: std::vector<GrabberHandle *> grabbers;

      /// \brief A list of line segments of the polyline
      protected: std::vector<LineSegmentItem*> segments;

      /// \brief Keep track of mouse press position for translating segments
      protected: QPointF segmentMouseMove;

      /// \brief True to indicate that the polyline is closed, where the first
      /// and last vertices are connected
      protected: bool closed;

      /// \brief Polyline item origin (start vertex).
      private: QPointF origin;

//      private: int gridSpace;

      private: int grabberWidth;

      private: int grabberHeight;

      /// \brief Thickness of the polyline in pixels
      private: double lineThickness;
    };
  }
}

#endif
