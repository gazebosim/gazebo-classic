/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_GUI_BUILDING_MEASUREITEM_HH_
#define _GAZEBO_GUI_BUILDING_MEASUREITEM_HH_

#include <memory>
#include <ignition/math/Vector2.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/building/SegmentItem.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data.
    class MeasureItemPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class MeasureItem MeasureItem.hh
    /// \brief Measurement lines and values.
    class GZ_GUI_VISIBLE MeasureItem : public SegmentItem
    {
      Q_OBJECT

      /// \brief Constructor
      /// param[in] _start Start position of the measure item in pixel
      /// coordinates.
      /// param[in] _end End position of the measure item in pixel coordinates.
      public: MeasureItem(const ignition::math::Vector2d &_start,
          const ignition::math::Vector2d &_end);

      /// \brief Destructor
      public: ~MeasureItem();

      // Documentation inherited
      private: virtual void paint(QPainter *_painter,
          const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      /// \brief Get distance between end points
      /// \return Distance between end points.
      public: double Distance() const;

      /// \brief Set value in meters
      /// \param[in] _value Value measured in meters.
      public: void SetValue(double _value);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<MeasureItemPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
