/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/BuildingItem.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class RectItem;
    class BuildingItem;
    class StairsInspectorDialog;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class StairsItem StairsItem.hh
    /// \brief 2D representation of a staircase.
    class GAZEBO_VISIBLE StairsItem :  public RectItem, public BuildingItem
    {
      Q_OBJECT

      /// \brief Constructor
      public: StairsItem();

      /// \brief Destructor
      public: ~StairsItem();

      // Documentation inherited
      public: virtual QVector3D GetSize() const;

      // Documentation inherited
      public: virtual QVector3D GetScenePosition() const;

      // Documentation inherited
      public: virtual double GetSceneRotation() const;

      /// \brief Get the number of steps in the staircase
      /// \return The number of steps in the staircase
      public: int GetSteps() const;

      // Documentation inherited
      private: virtual void paint(QPainter *_painter,
          const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      // Documentation inherited
      private: virtual bool RotateEventFilter(RotateHandle *_rotateHandle,
          QEvent *_event);

      // Documentation inherited
      private: void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event);

      // Documentation inherited
      private slots: void OnApply();

      // Documentation inherited
      private slots: void OnOpenInspector();

      // Documentation inherited
      private slots: void OnDeleteItem();

      /// \brief Emit stairs changed Qt signals.
      public: void StairsChanged();

      /// \brief Emit steps changed Qt signals.
      private: void StepsChanged();

      /// \brief Depth of staircase item in pixels.
      private: double stairsDepth;

      /// \brief Height of staircase item in pixels.
      private: double stairsHeight;

      /// \brief Width of staircase item in pixels.
      private: double stairsWidth;

      /// \brief Scene position of staircase item in pixel coordinates.
      private: QPointF stairsPos;

      /// \brief Elevation of staircase item in pixels.
      private: double stairsElevation;

      /// \brief Number of steps in the staircase item.
      private: int stairsSteps;

      /// \brief Inspector for configuring the staircase item.
      private: StairsInspectorDialog *inspector;

      // private: double stairsUnitRise;

      // private: double stairsUnitRun;
    };
    /// \}
  }
}
#endif
