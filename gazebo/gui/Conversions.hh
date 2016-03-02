/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_GUI_CONVERSIONS_HH_
#define _GAZEBO_GUI_CONVERSIONS_HH_

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/gui/qt.h"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class Color;
  }

  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \brief Conversions Conversions.hh gui/Conversions.hh
    /// \brief A set of utility function to convert between Gazebo and Qt
    /// data types
    class GZ_GUI_VISIBLE Conversions
    {
      /// \brief Return the equivalent qt color
      /// \param[in] _color Gazebo color to convert
      /// \return Qt color value
      public: static QColor Convert(const common::Color &_color);

      /// \brief Return the equivalent gazebo color
      /// \param[in] _color Qt color to convert
      /// \return Gazebo color value
      public: static common::Color Convert(const QColor &_color);

      /// \brief Return the equivalent QPointF.
      /// \param[in] _point Ignition vector to convert.
      /// \return QPointF.
      public: static QPointF Convert(const ignition::math::Vector2d &_pt);

      /// \brief Return the equivalent ignition vector.
      /// \param[in] _color QPointF to convert
      /// \return Ignition Vector2d.
      public: static ignition::math::Vector2d Convert(const QPointF &_pt);

      /// \brief Return the equivalent qt vector 3d.
      /// \param[in] _vec Ignition vector 3d to convert.
      /// \return Qt vector 3d value.
      public: static QVector3D Convert(const ignition::math::Vector3d &_vec);

      /// \brief Return the equivalent ignition vector 3d.
      /// \param[in] _vec Qt vector 3d to convert.
      /// \return Ignition vector 3d value
      public: static ignition::math::Vector3d Convert(const QVector3D &_vec);
    };
    /// \}
  }
}
#endif
