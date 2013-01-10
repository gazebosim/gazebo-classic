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

#ifndef _FLOOR_ITEM_HH_
#define _FLOOR_ITEM_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/gui/model_editor/RectItem.hh"
#include "gazebo/gui/model_editor/BuildingItem.hh"


namespace gazebo
{
  namespace gui
  {
    class BuildingItem;

    class WallItem;

    class FloorItem : public RectItem, public BuildingItem
    {
        Q_OBJECT

        public: FloorItem();

        public: ~FloorItem();

        public: virtual QVector3D GetSize() const;

        public: virtual QVector3D GetScenePosition() const;

        public: virtual double GetSceneRotation() const;

        public: void AttachWall(WallItem *_wallItem);

        private: virtual void paint (QPainter *_painter,
            const QStyleOptionGraphicsItem *_option, QWidget *_widget);

        private: virtual void mousePressEvent(QGraphicsSceneMouseEvent *_event);

        private: virtual void contextMenuEvent(
            QGraphicsSceneContextMenuEvent *_event);

        private slots: void NotifyChange();

        private slots: void RecalculateBoundingBox();

        private slots: void WallDeleted();

        private: void Update();

        private: void FloorChanged();

        private: void SizeChanged();

        private: double floorDepth;

        private: double floorHeight;

        private: double floorWidth;

        private: QPointF floorPos;

        private: bool dirty;

        private: double scale;

        private: std::vector<WallItem *> walls;

        private: QGraphicsItemGroup *wallGroup;

        private: QPolygonF floorBoundingRect;
    };
  }
}

#endif
