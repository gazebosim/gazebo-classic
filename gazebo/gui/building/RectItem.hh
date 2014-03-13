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

#ifndef _RECT_ITEM_HH_
#define _RECT_ITEM_HH_

#include <vector>
#include "gazebo/gui/qt.h"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class GrabberHandle;
    class RotateHandle;
    class EditorItem;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class RectItem RectItem.hh
    /// \brief 2D rectangle.
    class GAZEBO_VISIBLE RectItem : public EditorItem, public QGraphicsRectItem
    {
      Q_OBJECT

      /// \brief Resize flags used to indicate which dimension can be resized.
      public: enum ResizeFlags {NONE = 0x00,
          ITEM_WIDTH = 0x01, ITEM_HEIGHT = 0x02};

      /// \brief Constructor
      public: RectItem();

      /// \brief Destructor
      public: virtual ~RectItem();

      /// \brief Set the width of the rect item.
      /// \param[in] _width Width of the rect item in pixels.
      public: void SetWidth(int _width);

      /// \brief Set the height of the rect item.
      /// \param[in] _height Height of the rect item in pixels.
      public: void SetHeight(int _height);

      /// \brief Set the size of the rect item.
      /// \param[in] _size Size of the rect item in pixels.
      public: void SetSize(QSize _size);

      /// \brief Get the width of the rect item.
      /// \return Width of the rect item in pixels.
      public: double GetWidth() const;

      /// \brief Get the height of the rect item.
      /// \return Height of the rect item in pixels.
      public: double GetHeight() const;

      /// \brief Show the grabber and rotate handles of the rect item.
      /// \param[in] _show True to draw the handles, and false to hide them.
      public: void ShowHandles(bool _show);

      /// \brief Helper method for Updating the corner positions of the rect
      /// item.
      protected: void UpdateCornerPositions();

      /// \brief Draw bounding box
      /// \param[in] _painter Qt painter object.
      protected: void DrawBoundingBox(QPainter *_painter);

      /// \brief Set the position of the rect item
      /// \param[in] _pos Position in pixel coordinates.
      public: virtual void SetPosition(const QPointF &_pos);

      /// \brief Set the position of the rect item
      /// \param[in] _x X position in pixel coordinates.
      /// \param[in] _y Y position in pixel coordinates.
      public: virtual void SetPosition(double _x, double _y);

      /// \brief Set the rotation of the rect item.
      /// \param[in] _angle Rotation angle in degrees.
      public: virtual void SetRotation(double _angle);

      /// \brief Set the resize flag of the rect item.
      /// \param[in] _flag Resize flag which controls how the item can be
      /// resized.
      public: virtual void SetResizeFlag(unsigned int _flag);

      /// \brief Get the rotation of the rect item
      /// \return Rotation in degrees.
      public: virtual double GetRotation() const;

      // Documentation inherited
      public: virtual QVector3D GetSize() const;

      // Documentation inherited
      public: virtual QVector3D GetScenePosition() const;

      // Documentation inherited
      public: virtual double GetSceneRotation() const;

      /// \brief Get the bounding box of the rect item.
      /// \return The bounding box of the rect item.
      protected: virtual QRectF boundingRect() const;

      /// \brief Filter Qt events and redirect them to the rotate handle.
      /// \param[in] _rotateHandle Rotate handle that will handle the event.
      /// \param[in] _event Qt event
      private: virtual bool RotateEventFilter(RotateHandle *_rotateHandle,
          QEvent *_event);

      /// \brief Filter Qt events and redirect them to the grabber handle.
      /// \param[in] _rotateHandle Grabber handle that will handle the event.
      /// \param[in] _event Qt event
      private: virtual bool GrabberEventFilter(GrabberHandle *_grabber,
          QEvent *_event);

      /// \brief Qt paint function for drawing the rect item.
      /// \param[in] _painter Qt painter object.
      /// \param[in] _option Qt style options for the item.
      /// \param[in] _widget Qt widget being painted on.
      private: virtual void paint(QPainter *_painter,
          const QStyleOptionGraphicsItem *_option, QWidget *_widget);

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
      private: virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse press event.
      /// \param[in] _event Qt mouse event.
      private: virtual void mousePressEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse release event.
      /// \param[in] _event Qt mouse event.
      private: virtual void mouseReleaseEvent(
          QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse press event during drag and drop.
      /// \param[in] _event Qt mouse drag and drop event.
      private: virtual void mousePressEvent(
          QGraphicsSceneDragDropEvent *_event);

      /// \brief Qt mouse move event during drag and drop.
      /// \param[in] _event Qt mouse drag and drop event.
      private: virtual void mouseMoveEvent(
          QGraphicsSceneDragDropEvent *_event);

      /// \brief Qt mouse double click event.
      /// \param[in] _event Qt mouse event.
      private: virtual void mouseDoubleClickEvent(
          QGraphicsSceneMouseEvent *_event);

      /// \brief Filter Qt events and redirect them to another item.
      /// \param[in] _watched Item that handle that will handle the event.
      /// \param[in] _event Qt event.
      private: virtual bool sceneEventFilter(QGraphicsItem *_watched,
          QEvent *_event);

      /// \brief React to item changes notified by Qt.
      /// \param[in] _change Qt change type, e.g. selected change, position
      /// change.
      /// \param[in] _value Value to changed to.
      private: QVariant itemChange(GraphicsItemChange _change,
        const QVariant &_value);

      /// \brief Qt context menu event received on a mouse right click.
      /// \param[in] Qt context menu event.
      private: virtual void contextMenuEvent(
          QGraphicsSceneContextMenuEvent *_event);

      /// brief Emit size changed Qt signals.
      private: virtual void SizeChanged();

      /// brief Helper function for resizing the rect item.
      /// \param[in] _x Change in x (width).
      /// \param[in] _y Change in y (height).
      private: void AdjustSize(double _x, double _y);

      /// \brief Qt callback for opening the item inspector
      private slots: virtual void OnOpenInspector();

      /// \brief Qt callback when the item is being deleted.
      private slots: virtual void OnDeleteItem();

      /// \brief Width of rect item in pixels.
      protected: double width;

      /// \brief Height of rect item in pixels.
      protected: double height;

      /// \brief Actual width of rect item drawn in pixels.
      protected: double drawingWidth;

      /// \brief Actual height of rect item drawn in pixels.
      protected: double drawingHeight;

      /// \brief X origin of the rect item in pixels.
      protected: double drawingOriginX;

      /// \brief Y origin of the rect item in pixels.
      protected: double drawingOriginY;

      /// \brief Border color of the rect item.
      protected: QColor borderColor;

      /// \brief Rotation angle of the rect item in degrees.
      protected: double rotationAngle;

      /// \brief Z ordering of the rect item when idle (unselected.)
      protected: int zValueIdle;

      /// \brief Qt action for opening the inspector.
      protected: QAction *openInspectorAct;

      /// \brief Qt action for deleting the item.
      protected: QAction *deleteItemAct;

      /// \brief Mouse press position in pixel coordinates.
      private: QPointF mousePressPos;

      /// \brief Mouse press position in pixel coordinates.
      private: int gridSpace;

      /// \brief A list of grabber handles for this item. Four for corners and
      /// four for edges, going clockwise with 0 being top left
      private: std::vector<GrabberHandle *> grabbers;

      /// \brief Rotate handle for rotating the rect item.
      private: RotateHandle *rotateHandle;

      /// \brief A list of resize cursors used when the mouse hovers over the
      /// grabber handles.
      private: std::vector<Qt::CursorShape> cursors;

      /// \brieft Z ordering of the rect item when selected.
      private: int zValueSelected;

      /// \brieft Resize flag that controls how the rect item can be resized.
      private: unsigned int resizeFlag;
    };
    /// \}
  }
}

#endif
