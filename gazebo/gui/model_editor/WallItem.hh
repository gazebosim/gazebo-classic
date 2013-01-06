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

#ifndef _WALL_ITEM_HH_
#define _WALL_ITEM_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/gui/model_editor/PolylineItem.hh"
#include "gazebo/gui/model_editor/BuildingItem.hh"

namespace gazebo
{
  namespace gui
  {
    class PolylineItem;

    class CornerGrabber;

    class LineSegmentItem;

    class BuildingItem;

    class WallInspectorDialog;

    class WallItem : public PolylineItem, public BuildingItem
    {
        Q_OBJECT

        public: WallItem(const QPointF &_start, const QPointF &_end);

        public: ~WallItem();

        public: double GetHeight() const;

        public: void SetHeight(double _height);

        public: WallItem *Clone() const;

        public: void Update();

        private: bool cornerEventFilter(CornerGrabber *_corner,
            QEvent *_event);

        private: bool segmentEventFilter(LineSegmentItem *_segment,
            QEvent *_event);

        private: void contextMenuEvent(QGraphicsSceneContextMenuEvent *_event);

        private slots: void OnApply();

        private slots: void OnOpenInspector();

        private: void WallChanged();

        private: void SetSegmentSelected(unsigned int _index, bool _selected);

        private: double wallThickness;

        private: double wallHeight;

        private: double scale;

        private: LineSegmentItem* selectedSegment;

        private: QAction *openInspectorAct;

        private: WallInspectorDialog *inspector;
    };
  }
}

#endif
