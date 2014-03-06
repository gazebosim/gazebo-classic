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

#ifndef _POLYLINE_ITEM_HH_
#define _POLYLINE_ITEM_HH_

#include <vector>
#include "gazebo/gui/qt.h"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class GrabberHandle;
    class EditorItem;
    class LineSegmentItem;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class PolylineItem PolylineItem.hh
    /// \brief 2D polyline.
    class GAZEBO_VISIBLE PolylineItem
      : public EditorItem, public QGraphicsPathItem
    {
      /// \brief Constructor
      /// param[in] _start Start position of the polyline item in pixel
      /// coordinates.
      /// param[in] _start End position of the polyline item in pixel
      /// coordinates.
      public: PolylineItem(const QPointF &_start, const QPointF &_end);

      /// \brief Destructor
      public: ~PolylineItem();

      /// \brief Add a point to the polyline item.
      /// \param[in] _point Point to add
      public: void AddPoint(const QPointF &_point);

      /// \brief Pop the end point off the polyline item.
      public: void PopEndPoint();

      /// \brief Get the number of vertices the polyline item has.
      /// \return Number of vertices
      public: unsigned int GetVertexCount() const;

      /// \brief Get the number of line segments the polyline item has.
      /// \return Number of line segments
      public: unsigned int GetSegmentCount() const;

      /// \brief Set the position of a vertex of the polyline item.
      /// \param[in] _index Index of the vertex.
      /// \param[in] _pos Position in pixel coordinates.
      public: void SetVertexPosition(unsigned int _index,
          const QPointF &_pos);

      /// \brief Translate a vertex of the polyline item.
      /// \param[in] _index Index of the vertex.
      /// \param[in] _pos Translation vector in pixels.
      public: void TranslateVertex(unsigned int _index,
          const QPointF &_trans);

      /// \brief Get a line segment of the polyline item.
      /// \param[in] _index Index of the line segment.
      public: LineSegmentItem *GetSegment(unsigned int _index) const;

      /// \brief Show the grabber handles of the polyline item.
      /// \param[in] _show True to show the grabber handles, false to hide them.
      public: void ShowHandles(bool _show);

      /// \brief Set the thickness of the polyline item.
      /// \param[in] _thickness Thickness in pixels.
      public: void SetThickness(double _thickness);

      /// \brief Set the position of the polyline item.
      /// \param[in] _pos Position in pixel coordinates.
      public: void SetPosition(const QPointF &_pos);

      /// \brief Call the function to indicate that the start and end vertices
      /// are connected
      public: void ClosePath();

      /// \brief Get whether of not the polyline item is closed.
      /// \return True if the polyline item is closed, false otherwise.
      public: bool IsClosed() const;

      /// \brief Update by calling all line segments' Update function which
      /// emits Qt signals
      public: void Update();

      /// \brief Helper function for updating the underlying Qt painter path.
      private: void UpdatePath();

      /// \brief Helper function for updating a point on the underlying Qt
      /// painter path.
      /// \param[in] _index Index of the point on the painter path.
      /// \param[in] _pos New position in pixel coordinates.
      private: void UpdatePathAt(unsigned int _index, const QPointF &_pos);

      /// \brief Helper function for appending a point on the underlying Qt
      /// painter path.
      /// \param[in] _point A point to append to the painter path.
      private: void AppendToPath(const QPointF &_point);

      /// \brief Filter Qt events and redirect them to the another item.
      /// \param[in] _watched Item that watches and will handle the event.
      /// \param[in] _event Qt event.
      private: bool sceneEventFilter(QGraphicsItem * watched,
        QEvent *_event);

      /// \brief Filter Qt events and redirect them to the grabber handle.
      /// \param[in] _rotateHandle Grabber handle that will handle the event.
      /// \param[in] _event Qt event
      private: virtual bool GrabberEventFilter(GrabberHandle *_grabber,
          QEvent *_event);

      /// \brief Filter Qt events and redirect them to the line segment item.
      /// \param[in] _item Line segment item that will handle the event.
      /// \param[in] _event Qt event
      private: virtual bool SegmentEventFilter(LineSegmentItem *_item,
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
      /// \param[in] _option Qt style options for the item.
      /// \param[in] _widget Qt widget being painted on.
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

      /// \brief True to indicate that the polyline is closed
      protected: bool closed;

      /// \brief Polyline item origin (position of the first vertex).
      private: QPointF origin;

      // private: int gridSpace;

      /// \brief Width of grabber in pixels
      private: double grabberWidth;

      /// \brief Height of grabber in pixels
      private: double grabberHeight;

      /// \brief Thickness of the polyline in pixels
      private: double lineThickness;
    };
    /// \}
  }
}

#endif
