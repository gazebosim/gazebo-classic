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

#ifndef _STAIRS_ITEM_HH_
#define _STAIRS_ITEM_HH_

#include "gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class RectItem;

    class StairsItem : public RectItem
    {
        public: StairsItem();

        public: ~StairsItem();

        public: int GetLevel();

        public: void SetLevel(int _level);

        public: virtual QVector3D GetSize();

        public: virtual QVector3D GetScenePosition();

        public: virtual double GetSceneRotation();

        public: int GetSteps();

        private: virtual void paint (QPainter *_painter,
            const QStyleOptionGraphicsItem *_option, QWidget *_widget);

        private: void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event);

        private: void StairsChanged();

        private: double stairsDepth;

        private: double stairsHeight;

        private: double stairsWidth;

        private: double stairsSideBar;

        private: QPointF stairsPos;

        private: double stairsElevation;

        private: int stairsSteps;

        private: double scale;

        private: int level;

//        private: double stairsUnitRise;

//        private: double stairsUnitRun;
    };
  }
}
#endif
