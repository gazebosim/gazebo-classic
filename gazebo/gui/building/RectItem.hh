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

#ifndef _GAZEBO_GUI_BUILDING_RECTITEM_HH_
#define _GAZEBO_GUI_BUILDING_RECTITEM_HH_

#include <memory>
#include <vector>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/Color.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/building/EditorItem.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class GrabberHandle;
    class MeasureItem;
    class RotateHandle;

    // Forward declare private data.
    class RectItemPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class RectItem RectItem.hh
    /// \brief 2D rectangle.
    class GZ_GUI_VISIBLE RectItem :
      public EditorItem, public QGraphicsRectItem
    {
      Q_OBJECT

      /// \brief Resize flags used to indicate which dimension can be resized.
      public: enum ResizeFlags
          {
            /// \brief No dimensions.
            NONE = 0x00,

            /// \brief Width
            ITEM_WIDTH = 0x01,

            /// \brief Height
            ITEM_HEIGHT = 0x02
          };

      /// \brief Constructor
      public: RectItem();

      /// \brief Destructor
      public: virtual ~RectItem();

      /// \brief Set the width of the rect item.
      /// \param[in] _width Width of the rect item in pixels.
      public: void SetWidth(const int _width);

      /// \brief Set the height of the rect item.
      /// \param[in] _height Height of the rect item in pixels.
      public: void SetHeight(const int _height);

      /// \brief Set the size of the rect item.
      /// \param[in] _size Size of the rect item in pixels.
      public: void SetSize(const ignition::math::Vector2i &_size);

      /// \brief Get the width of the rect item.
      /// \return Width of the rect item in pixels.
      public: double Width() const;

      /// \brief Get the height of the rect item.
      /// \return Height of the rect item in pixels.
      public: double Height() const;

      /// \brief Set the position of this item inside its parent wall.
      /// \param[in] _positionOnWall New normalized position on wall.
      public: void SetPositionOnWall(const double _positionOnWall);

      /// \brief Get the position of this item inside its parent wall.
      /// \return Normalized position on parent wall.
      public: double PositionOnWall() const;

      /// \brief Set the angle of this item inside its parent wall.
      /// \param[in] _angleOnWall New angle on wall, either 0 or 180 degrees.
      public: void SetAngleOnWall(const double _angleOnWall);

      /// \brief Get the angle of this item inside its parent wall.
      /// \return Angle on parent wall in degrees.
      public: double AngleOnWall() const;

      /// \brief Show the grabber and rotate handles of the rect item.
      /// \param[in] _show True to draw the handles, and false to hide them.
      public: void ShowHandles(const bool _show);

      // Documentation inherited
      public: void SetHighlighted(const bool _highlighted);

      /// \brief Detach the rect item from its parent.
      public: void DetachFromParent();

      /// \brief Helper method for Updating the corner positions of the rect
      /// item.
      protected: void UpdateCornerPositions();

      /// \brief Draw bounding box
      /// \param[in] _painter Qt painter object.
      protected: void DrawBoundingBox(QPainter *_painter);

      /// \brief Set the position of the rect item
      /// \param[in] _pos Position in pixel coordinates.
      public: virtual void SetPosition(const ignition::math::Vector2d &_pos);

      /// \brief Set the position of the rect item
      /// \param[in] _x X position in pixel coordinates.
      /// \param[in] _y Y position in pixel coordinates.
      public: virtual void SetPosition(const double _x, const double _y);

      /// \brief Set the rotation of the rect item.
      /// \param[in] _angle Rotation angle in degrees.
      public: virtual void SetRotation(const double _angle);

      /// \brief Set the resize flag of the rect item.
      /// \param[in] _flag Resize flag which controls how the item can be
      /// resized.
      public: virtual void SetResizeFlag(const unsigned int _flag);

      /// \brief Get the rotation of the rect item
      /// \return Rotation in degrees.
      public: virtual double Rotation() const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d Size() const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d ScenePosition() const;

      // Documentation inherited
      public: virtual double SceneRotation() const;

      /// \brief Get the bounding box of the rect item.
      /// \return The bounding box of the rect item.
      protected: virtual QRectF boundingRect() const;

      /// \brief Update this item's measures.
      protected: void UpdateMeasures();

      /// \brief Filter Qt events and redirect them to the rotate handle.
      /// \param[in] _rotateHandle Rotate handle that will handle the event.
      /// \param[in] _event Qt event
      private: virtual bool RotateEventFilter(RotateHandle *_rotateHandle,
          QEvent *_event);

      /// \brief Filter Qt events and redirect them to the grabber handle.
      /// \param[in] _grabber Grabber handle that will handle the event.
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
      private: void AdjustSize(const double _x, const double _y);

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
      protected: common::Color borderColor;

      /// \brief Rotation angle of the rect item in degrees.
      protected: double rotationAngle;

      /// \brief Qt action for opening the inspector.
      protected: QAction *openInspectorAct;

      /// \brief Qt action for deleting the item.
      protected: QAction *deleteItemAct;

      /// \brief A vector containing this item's measure items.
      /// Currently only used for windows and doors, containing one measure
      /// towards each end of this item's parent wall.
      protected: std::vector<MeasureItem *> measures;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<RectItemPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
