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

#include "gazebo/common/Color.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/Conversions.hh"

using namespace gazebo;
using namespace gui;

//////////////////////////////////////////////////
QColor Conversions::Convert(const common::Color &_color)
{
  return QColor(_color.r*255.0, _color.g*255.0, _color.b*255.0, _color.a*255.0);
}

//////////////////////////////////////////////////
common::Color Conversions::Convert(const QColor &_color)
{
  return common::Color(_color.red() / 255.0,
                       _color.green() / 255.0,
                       _color.blue() / 255.0,
                       _color.alpha() / 255.0);
}

//////////////////////////////////////////////////
QPointF Conversions::Convert(const ignition::math::Vector2d &_pt)
{
  return QPointF(_pt.X(), _pt.Y());
}

//////////////////////////////////////////////////
ignition::math::Vector2d Conversions::Convert(const QPointF &_pt)
{
  return ignition::math::Vector2d(_pt.x(), _pt.y());
}

//////////////////////////////////////////////////
QVector3D Conversions::Convert(const ignition::math::Vector3d &_vec)
{
  return QVector3D(_vec.X(), _vec.Y(), _vec.Z());
}

//////////////////////////////////////////////////
ignition::math::Vector3d Conversions::Convert(const QVector3D &_vec)
{
  return ignition::math::Vector3d(_vec.x(), _vec.y(), _vec.z());
}

