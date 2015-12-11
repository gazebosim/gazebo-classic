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

#ifndef _GAZEBO_BUILDING_STAIRS_ITEM_HH_
#define _GAZEBO_BUILDING_STAIRS_ITEM_HH_

#include "gazebo/gui/building/RectItem.hh"

namespace gazebo
{
  namespace gui
  {
    class RectItem;
    class BuildingItem;
    class StairsInspectorDialog;
    class StairsItemPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class StairsItem StairsItem.hh
    /// \brief 2D representation of a staircase.
    class GZ_GUI_VISIBLE StairsItem :
      public RectItem, public BuildingItem
    {
      Q_OBJECT

      /// \brief Constructor
      public: StairsItem();

      /// \brief Destructor
      public: ~StairsItem() = default;

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

      /// \internal
      /// \brief Pointer to private data.
      protected: std::shared_ptr<StairsItemPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
