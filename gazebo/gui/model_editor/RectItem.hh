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

#ifndef _RECT_ITEM_H_
#define _RECT_ITEM_H_

#include "gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class CornerGrabber;

    class RectItem : public QGraphicsItem
    {
        public: RectItem();

        public: virtual ~RectItem();

        public: void SetGridSpace(int _space);

        protected: void UpdateCornerPositions();

        private: void AdjustSize(int _x, int _y);

        private: virtual QRectF boundingRect() const;

        private: virtual void paint (QPainter *_painter,
            const QStyleOptionGraphicsItem *_option, QWidget *_widget);

        private: virtual void hoverEnterEvent(QGraphicsSceneHoverEvent *_event);

        private: virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent *_event);

        private: virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *_event);

        private: virtual void mousePressEvent(QGraphicsSceneMouseEvent *_event);

        private: virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *_event);

        private: virtual void mouseMoveEvent(QGraphicsSceneDragDropEvent *_event);

        private: virtual void mousePressEvent(QGraphicsSceneDragDropEvent *_event);

        private: virtual void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event);

        private: virtual bool sceneEventFilter(QGraphicsItem *_watched,
            QEvent *_event);

        protected: qreal width;

        protected: qreal height;

        protected: qreal drawingWidth;

        protected: qreal drawingHeight;

        protected: qreal drawingOriginX;

        protected: qreal drawingOriginY;

        protected: QColor outterBorderColor;

        private: QPointF location;

        private: QPointF dragStart;

        private: QPointF rotateStart;

        private: int gridSpace;

        private: QPointF cornerDragStart;

        private: int xCornerGrabBuffer;

        private: int yCornerGrabBuffer;

        /// \brief Corners of the rectangle, going clockwise with 0 being top left
        CornerGrabber*  corners[4];
    };
  }
}

#endif
