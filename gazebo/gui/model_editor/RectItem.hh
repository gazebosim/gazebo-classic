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

#include <vector>
#include "gazebo/gui/qt.h"
#include "gazebo/gui/model_editor/EditorItem.hh"

namespace gazebo
{
  namespace gui
  {
    class GrabberHandle;

    class RotateHandle;

    class EditorItem;

    class RectItem : public EditorItem, public QGraphicsRectItem
    {
        Q_OBJECT

        public: enum ResizeFlags {NONE = 0x00,
            ITEM_WIDTH = 0x01, ITEM_HEIGHT = 0x02};

        public: RectItem();

        public: virtual ~RectItem();

        public: void SetGridSpace(int _space);

        public: void SetWidth(int _width);

        public: void SetHeight(int _height);

        public: void SetSize(QSize _size);

        public: double GetWidth() const;

        public: double GetHeight() const;

        public: void ShowCorners(bool _show);

        protected: void UpdateCornerPositions();

        protected: void DrawBoundingBox(QPainter *_painter);

        public: virtual void SetPosition(const QPointF &_pos);

        public: virtual void SetPosition(double _x, double _y);

        public: virtual void SetRotation(double _angle);

        public: virtual void SetResizeFlag(unsigned int _flag);

        public: double GetRotation() const;

        public: virtual QVector3D GetSize() const;

        public: virtual QVector3D GetScenePosition() const;

        public: virtual double GetSceneRotation() const;

        protected: virtual QRectF boundingRect() const;

        private: virtual bool rotateEventFilter(RotateHandle *_rotateHandle,
            QEvent *_event);

        private: virtual bool grabberEventFilter(GrabberHandle *_grabber,
            QEvent *_event);

        private: virtual void paint(QPainter *_painter,
            const QStyleOptionGraphicsItem *_option, QWidget *_widget);

        private: virtual void hoverEnterEvent(QGraphicsSceneHoverEvent *_event);

        private: virtual void hoverMoveEvent(QGraphicsSceneHoverEvent *_event);

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

        private: virtual void contextMenuEvent(
            QGraphicsSceneContextMenuEvent *_event);

        private: virtual void SizeChanged();

        private: void AdjustSize(double _x, double _y);

        private slots: virtual void OnOpenInspector();

        private slots: virtual void OnDeleteItem();

        protected: double width;

        protected: double height;

        protected: double drawingWidth;

        protected: double drawingHeight;

        protected: double drawingOriginX;

        protected: double drawingOriginY;

        protected: QColor borderColor;

        protected: double rotationAngle;

        protected: int zValueIdle;

        protected: QPointF pivot;

        protected: QAction *openInspectorAct;

        protected: QAction *deleteItemAct;

        private: QPointF location;

        private: QPointF mousePressPos;

        private: int gridSpace;

        /// \brief Four grabbers and four edges, going clockwise with
        /// 0 being top left
        private: std::vector<GrabberHandle *> grabbers;

        private: RotateHandle *rotateHandle;

        private: std::vector<Qt::CursorShape> cursors;

        private: int zValueSelected;

        private: unsigned int resizeFlag;
    };
  }
}

#endif
