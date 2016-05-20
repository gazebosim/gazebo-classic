/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_GUI_BUILDING_STAIRSITEM_HH_
#define _GAZEBO_GUI_BUILDING_STAIRSITEM_HH_

#include <memory>
#include <ignition/math/Vector3.hh>

#include "gazebo/gui/building/RectItem.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class RotateHandle;

    // Forward declare private data.
    class StairsItemPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class StairsItem StairsItem.hh
    /// \brief 2D representation of a staircase.
    class GZ_GUI_VISIBLE StairsItem : public RectItem
    {
      Q_OBJECT

      /// \brief Constructor
      public: StairsItem();

      /// \brief Destructor
      public: ~StairsItem();

      // Documentation inherited
      public: virtual ignition::math::Vector3d Size() const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d ScenePosition() const;

      // Documentation inherited
      public: virtual double SceneRotation() const;

      /// \brief Get the number of steps in the staircase
      /// \return The number of steps in the staircase
      public: int Steps() const;

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
      private: std::unique_ptr<StairsItemPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
