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

namespace gazebo
{
  namespace gui
  {
    class BuildingItem;

    class FloorItem : public RectItem, public BuildingItem
    {
        public: FloorItem();

        public: ~FloorItem();

        public: virtual QVector3D GetSize() const;

        public: virtual QVector3D GetScenePosition() const;

        public: virtual double GetSceneRotation() const;

        private: virtual void paint (QPainter *_painter,
            const QStyleOptionGraphicsItem *_option, QWidget *_widget);

        private: virtual void mousePressEvent(QGraphicsSceneMouseEvent *_event);

        private: void FloorChanged();

        private: void SizeChanged();

        private: double floorDepth;

        private: double floorHeight;

        private: double floorWidth;

        private: QPointF floorPos;

        private: double scale;
    };
  }
}

#endif
