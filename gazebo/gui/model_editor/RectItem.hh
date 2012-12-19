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

#ifndef _RECT_ITEM_HH_
#define _RECT_ITEM_HH_

#include "gui/qt.h"
#include "EditorItem.hh"

namespace gazebo
{
  namespace gui
  {
    class CornerGrabber;

    class RotateHandle;

    class EditorItem;

    class RectItem : public EditorItem, public QGraphicsItem
    {
        public: RectItem();

        public: virtual ~RectItem();

        public: void SetGridSpace(int _space);

        public: void SetWidth(int _width);

        public: void SetHeight(int _height);

        public: void SetSize(QSize _size);

        public: int GetWidth();

        public: int GetHeight();

        public: void showCorners(bool _show);

        protected: void UpdateCornerPositions();

        protected: void DrawBoundingBox(QPainter *_painter);

        public: void SetPosition(QPointF _pos);

        public: void SetPosition(double _x, double _y);

        public: void SetRotation(double _angle);

        public: double GetRotation();

        public: virtual QVector3D GetSize();

        public: virtual QVector3D GetScenePosition();

        public: virtual double GetSceneRotation();

        protected: virtual QRectF boundingRect() const;

        private: virtual bool rotateEventFilter(RotateHandle *_rotateHandle,
            QEvent *_event);

        private: virtual bool cornerEventFilter(CornerGrabber *_corner,
            QEvent *_event);

        private: virtual void paint (QPainter *_painter,
            const QStyleOptionGraphicsItem *_option, QWidget *_widget);

        private: virtual void hoverEnterEvent(QGraphicsSceneHoverEvent *_event);

        private: virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent *_event);

        private: virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *_event);

        private: virtual void mousePressEvent(QGraphicsSceneMouseEvent *_event);

        private: virtual void mouseReleaseEvent(
            QGraphicsSceneMouseEvent *_event);

        private: virtual void mouseMoveEvent(
            QGraphicsSceneDragDropEvent *_event);

        private: virtual void mousePressEvent(
            QGraphicsSceneDragDropEvent *_event);

        private: virtual void mouseDoubleClickEvent(
            QGraphicsSceneMouseEvent *_event);

        private: virtual bool sceneEventFilter(QGraphicsItem *_watched,
            QEvent *_event);

        private: QVariant itemChange(GraphicsItemChange _change,
          const QVariant &_value);

        private: void AdjustSize(double _x, double _y);

        protected: double width;

        protected: double height;

        protected: double drawingWidth;

        protected: double drawingHeight;

        protected: double drawingOriginX;

        protected: double drawingOriginY;

        protected: QColor borderColor;

        private: QPointF location;

        protected: QPointF pivot;

        private: int gridSpace;

        /// \brief Four corners and four edges, going clockwise with
        /// 0 being top left
        private: CornerGrabber *corners[8];

        private: RotateHandle *rotateHandle;

        protected: double rotationAngle;

        private: std::vector<Qt::CursorShape> cursors;

        private: int zValueSelected;

        protected: int zValueIdle;

    };
  }
}

#endif
