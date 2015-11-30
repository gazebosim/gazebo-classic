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
/* Desc: Dynamic line generator
 * Author: Nate Koenig
 * Date: 28 June 2007
 */

#ifndef DYNAMICLINES_HH
#define DYNAMICLINES_HH

#include <vector>
#include <string>

#include "gazebo/common/CommonIface.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/DynamicRenderable.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class DynamicLines DynamicLines.hh rendering/rendering.hh
    /// \brief Class for drawing lines that can change
    class GZ_RENDERING_VISIBLE DynamicLines : public DynamicRenderable
    {
      /// \brief Constructor
      /// \param[in] _opType The type of Line
      public: DynamicLines(RenderOpType _opType = RENDERING_LINE_STRIP);

      /// \brief Destructor
      public: virtual ~DynamicLines();

      /// \brief Get type of movable
      /// \return This returns "gazebo::dynamiclines"
      public: static std::string GetMovableType();

      /// \brief Overridden function from Ogre's base class.
      /// \return Returns "gazebo::ogredynamicslines"
      public: virtual const Ogre::String &getMovableType() const;

      /// \brief Add a point to the point list
      /// \param[in] _pt math::Vector3 point
      /// \param[in] _color common::Color Point color
      /// \deprecated See function that accepts ignition::math parameters
      public: void AddPoint(const math::Vector3 &_pt,
            const common::Color &_color = common::Color::White)
              GAZEBO_DEPRECATED(7.0);

      /// \brief Add a point to the point list
      /// \param[in] _pt ignition::math::Vector3d point
      /// \param[in] _color common::Color Point color
      public: void AddPoint(const ignition::math::Vector3d &_pt,
            const common::Color &_color = common::Color::White);

      /// \brief Add a point to the point list.
      /// \param[in] _x X position
      /// \param[in] _y Y position
      /// \param[in] _z Z position
      /// \param[in] _color common::Color Point color
      public: void AddPoint(double _x, double _y, double _z,
            const common::Color &_color = common::Color::White);

      /// \brief Change the location of an existing point in the point list
      /// \param[in] _index Index of the point to set
      /// \param[in] _value math::Vector3 value to set the point to
      /// \deprecated See function that accepts ignition::math parameters
      public: void SetPoint(unsigned int _index, const math::Vector3 &_value)
              GAZEBO_DEPRECATED(7.0);

      /// \brief Change the location of an existing point in the point list
      /// \param[in] _index Index of the point to set
      /// \param[in] _value ignition::math::Vector3d value to set the point to
      public: void SetPoint(unsigned int _index,
                  const ignition::math::Vector3d &_value);

      /// \brief Change the color of an existing point in the point list
      /// \param[in] _index Index of the point to set
      /// \param[in] _color common::Color Pixelcolor color to set the point to
      public: void SetColor(unsigned int _index, const common::Color &_color);

      /// \brief Return the location of an existing point in the point list
      /// \param[in] _index Number of the point to return
      /// \return math::Vector3 value of the point
      /// \deprecated See function that returns an ignition::math object
      /// \throws Throws an gazebo::common::Exception if the _index is out
      /// of bounds
      public: math::Vector3 GetPoint(unsigned int _index) const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Return the location of an existing point in the point list
      /// \param[in] _index Number of the point to return
      /// \return ignition::math::Vector3d value of the point
      /// \throws Throws an gazebo::common::Exception if the _index is out
      /// of bounds
      public: const ignition::math::Vector3d &Point(
                  const unsigned int _index) const;

      /// \brief Return the total number of points in the point list
      /// \return Number of points
      public: unsigned int GetPointCount() const;

      /// \brief Remove all points from the point list
      public: void Clear();

      /// \brief Call this to update the hardware buffer after making changes.
      public: void Update();

      /// \brief Implementation DynamicRenderable,
      /// creates a simple vertex-only decl
      private: virtual void  CreateVertexDeclaration();

      /// \brief Implementation DynamicRenderable, pushes point
      /// list out to hardware memory
      private: virtual void FillHardwareBuffers();

      /// \brief List of points for the line
      private: std::vector<ignition::math::Vector3d> points;

      /// \brief Used to indicate if the lines require an update
      private: bool dirty;

      /// \brief List of colors
      private: std::vector<common::Color> colors;
    };
    /// \}
  }
}
#endif
