/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

    /// \addtogroup gazebo_gui
    /// \{

    /// \class DoorItem DoorItem.hh
    /// \brief 2D representation of a door
    class GAZEBO_VISIBLE DoorItem : public RectItem, public BuildingItem
    {
      Q_OBJECT

      /// \brief Constructor
      public: DoorItem();

      /// \brief Destructor
      public: ~DoorItem();

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

      /// \brief Door depth in pixels
      private: double doorDepth;

      /// \brief Door height in pixels
      private: double doorHeight;

      /// \brief Door width in pixels
      private: double doorWidth;

      /// \brief Door elevation in pixels
      private: double doorElevation;

      /// \brief Door scene position in pixel coordinates.
      private: QPointF doorPos;

      /// \brief Inspector for configuring the door item.
      private: WindowDoorInspectorDialog *inspector;
    };
    /// \}
  }
}

#endif
