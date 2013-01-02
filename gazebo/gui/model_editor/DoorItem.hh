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

#ifndef _DOOR_ITEM_HH_
#define _DOOR_ITEM_HH_

#include "gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class RectItem;

    class BuildingItem;

    class DoorItem : public RectItem, public BuildingItem
    {
        public: DoorItem();

        public: ~DoorItem();

        public: virtual QVector3D GetSize();

        public: virtual QVector3D GetScenePosition();

        public: virtual double GetSceneRotation();

        private: virtual void paint (QPainter *_painter,
            const QStyleOptionGraphicsItem *_option, QWidget *_widget);

        private: void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event);

        private: void DoorChanged();

        private: void SizeChanged();

        private: double doorDepth;

        private: double doorHeight;

        private: double doorWidth;

        private: double doorElevation;

        private: QPointF doorPos;

        private: double scale;
    };
  }
}

#endif
