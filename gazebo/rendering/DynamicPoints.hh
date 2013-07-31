/*
 * Copyright 2013 Open Source Robotics Foundation
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
/* Desc: Dynamic point generator
 * Author: Carlos Agüero
 * Date: 30 July 2013
 */

#ifndef DYNAMICPOINTS_HH
#define DYNAMICPOINTS_HH

#include <vector>
#include <string>

#include "gazebo/math/Vector3.hh"
#include "gazebo/rendering/DynamicRenderable.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class DynamicPoints DynamicPoints.hh rendering/rendering.hh
    /// \brief Class for drawing points that can change
    class DynamicPoints : public DynamicRenderable
    {
      /// \brief Constructor
      /// \param[in] _opType Point type
      public: DynamicPoints(RenderOpType _opType = RENDERING_POINT_LIST);

      /// \brief Destructor
      public: virtual ~DynamicPoints();

      /// \brief Get type of movable
      /// \return This returns "gazebo::DynamicPoints"
      public: static std::string GetMovableType();

      /// \brief Overridden function from Ogre's base class.
      /// \return Returns "gazebo::ogredynamicspoints"
      public: virtual const Ogre::String &getMovableType() const;

      /// \brief Add a point to the point list
      /// \param[in] pt math::Vector3 point
      public: void AddPoint(const math::Vector3 &_pt,
                            const Ogre::ColourValue _color = DynamicPoints::WHITE);

      /// \brief Add a point to the point list.
      /// \param[in] _x X position.
      /// \param[in] _y Y position.
      /// \param[in] _z Z position.
      public: void AddPoint(double _x, double _y, double _z,
                            const Ogre::ColourValue _color = DynamicPoints::WHITE);

      /// \brief Change the location of an existing point in the point list
      /// \param[in] _index Index of the point to set
      /// \param[in] _value math::Vector3 value to set the point to
      public: void SetPoint(unsigned int _index, const math::Vector3 &_value);

      /// \brief Change the color of an existing point in the point list
      /// \param[in] _index Index of the point to set
      /// \param[in] _color pixelcolor color to set the point to
      public: void SetColor(unsigned int _index, const Ogre::ColourValue _color);

      /// \brief Return the location of an existing point in the point list
      /// \param[in] _index Number of the point to return
      /// \return math::Vector3 value of the point
      public: const math::Vector3& GetPoint(unsigned int _index) const;

      /// \brief Return the total number of points in the point list
      /// \return Number of points
      public: unsigned int GetPointCount() const;

      /// \brief Remove all points from the point list
      public: void Clear();

      /// \brief Call this to update the hardware buffer after making changes.
      public: void Update();

      /// \brief Implementation DynamicRenderable,
      /// creates a simple vertex-only decl
      protected: virtual void  CreateVertexDeclaration();

      /// \brief Implementation DynamicRenderable, pushes point
      /// list out to hardware memory
      protected: virtual void FillHardwareBuffers();

      private: static const Ogre::ColourValue WHITE;

      /// \brief List of points
      private: std::vector<math::Vector3> points;

      /// \brief List of colors
      private: std::vector<Ogre::ColourValue> colors;

      /// \brief Used to indicate if the lines require an update
      private: bool dirty;
    };
    /// \}
  }
}
#endif
