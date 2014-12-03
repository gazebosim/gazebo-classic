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

#ifndef _AXISVISUAL_HH_
#define _AXISVISUAL_HH_

#include <string>

#include "gazebo/math/Vector3.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering Rendering
    /// \{

    /// \class AxisVisual AxisVisual.hh rendering/rendering.hh
    /// \brief Basic axis visualization
    class GZ_RENDERING_VISIBLE AxisVisual : public Visual
    {
      /// \brief Constructor
      /// \param[in] _name Name of the AxisVisual
      /// \param[in] _vis Parent visual
      public: AxisVisual(const std::string &_name, VisualPtr _vis);

      /// \brief Destructor
      public: virtual ~AxisVisual();

      /// \brief Load the axis visual
      public: virtual void Load();

      /// \brief Show the rotation tube
      /// \param[in] _axis Axis index. 0: x, 1: y, 2: z
      /// \param[in] _show True to show the axis rotation tube.
      public: void ShowAxisRotation(unsigned int _axis, bool _show);

      /// \brief Show the axis arrow shaft.
      /// \param[in] _axis Axis index. 0: x, 1: y, 2: z
      /// \param[in] _show True to show the axis arrow shaft.
      public: void ShowAxisShaft(unsigned int _axis, bool _show);

      /// \brief Show the axis arrow head.
      /// \param[in] _axis Axis index. 0: x, 1: y, 2: z
      /// \param[in] _show True to show the axis arrow head.
      public: void ShowAxisHead(unsigned int _axis, bool _show);

      /// \brief Scale the X axis
      /// \param[in] _scale Scaling factor
      public: void ScaleXAxis(const math::Vector3 &_scale);

      /// \brief Scale the Y axis
      /// \param[in] _scale Scaling factor
      public: void ScaleYAxis(const math::Vector3 &_scale);

      /// \brief Scale the Z axis
      /// \param[in] _scale Scaling factor
      public: void ScaleZAxis(const math::Vector3 &_scale);

      /// \brief Set the material used to render and axis
      /// \param[in] _axis The number of the axis (0, 1, 2 = x,y,z)
      /// \param[in] _material The name of the material to apply to the axis
      public: void SetAxisMaterial(unsigned int _axis,
                                   const std::string &_material);

      /// \brief Set whether the axis will be visible
      /// \param[in] _axis The number of the axis (0, 1, 2 = x,y,z)
      /// \param[in] _visible True to set the axis to be visible.
      public: void SetAxisVisible(unsigned int _axis, bool _visible);
    };
    /// \}
  }
}
#endif
