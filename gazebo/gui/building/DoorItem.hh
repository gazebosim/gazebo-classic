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

#ifndef _GAZEBO_BUILDING_DOOR_ITEM_HH_
#define _GAZEBO_BUILDING_DOOR_ITEM_HH_

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
    class WindowDoorInspectorDialog;
    class DoorItemPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class DoorItem DoorItem.hh
    /// \brief 2D representation of a door
    class GZ_GUI_VISIBLE DoorItem :
      public RectItem, public BuildingItem
    {
      Q_OBJECT

      /// \brief Constructor
      public: DoorItem();

      /// \brief Destructor
      public: ~DoorItem() = default;

      // Documentation inherited
      public: virtual QVector3D GetSize() const;

      // Documentation inherited
      public: virtual QVector3D GetScenePosition() const;

      // Documentation inherited
      public: virtual double GetSceneRotation() const;

      // Documentation inherited
      private: virtual void paint(QPainter *_painter,
          const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      // Documentation inherited
      private: void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event);

      // Documentation inherited
      private slots: void OnApply();

      // Documentation inherited
      private slots: void OnOpenInspector();

      // Documentation inherited
      private slots: void OnDeleteItem();

      /// \brief Emit door changed signals
      public: void DoorChanged();

      /// \brief Emit size changed signals
      private: void SizeChanged();

      /// \internal
      /// \brief Pointer to private data.
      protected: std::shared_ptr<DoorItemPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
