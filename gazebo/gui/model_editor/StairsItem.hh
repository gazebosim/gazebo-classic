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

#include "gazebo/gui/qt.h"
#include "gazebo/gui/model_editor/RectItem.hh"
#include "gazebo/gui/model_editor/BuildingItem.hh"

namespace gazebo
{
  namespace gui
  {
    class RectItem;

    class BuildingItem;

    class StairsInspectorDialog;

    class StairsItem :  public RectItem, public BuildingItem
    {
      Q_OBJECT

      public: StairsItem();

      public: ~StairsItem();

      public: virtual QVector3D GetSize() const;

      public: virtual QVector3D GetScenePosition() const;

      public: virtual double GetSceneRotation() const;

      public: int GetSteps() const;

      private: virtual void paint (QPainter *_painter,
          const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      private: virtual bool rotateEventFilter(RotateHandle *_rotateHandle,
          QEvent *_event);

      private: void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event);

      private slots: void OnApply();

      private slots: void OnOpenInspector();

      private: void StairsChanged();

      private: void StepsChanged();

      private: double stairsDepth;

      private: double stairsHeight;

      private: double stairsWidth;

      private: double stairsSideBar;

      private: QPointF stairsPos;

      private: double stairsElevation;

      private: int stairsSteps;

      private: double scale;

      private: StairsInspectorDialog* inspector;

//        private: double stairsUnitRise;

//        private: double stairsUnitRun;
    };
  }
}
#endif
