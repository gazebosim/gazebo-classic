/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_REFERENCE_GEOMETRY_HH_
#define _GAZEBO_REFERENCE_GEOMETRY_HH_

#include <string>

#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class ReferenceGeometry ReferenceGeometry.hh rendering/rendering.hh
    /// \brief Visualization for a reference geometry
    class GZ_RENDERING_VISIBLE ReferenceGeometry : public Visual
    {

      /// \brief Type of reference geometry
      public: enum ReferenceGeometryType
      {
        /// \brief None visual
        RGT_NONE,
        /// \brief Axis type
        RGT_AXIS
      };

      /// \brief Constructor
      /// \param[in] _name Name of the visual
      /// \param[in] _vis Pointer to the parent visual
      public: ReferenceGeometry(const std::string &_name, VisualPtr _vis);

      /// \brief Destructor
      public: virtual ~ReferenceGeometry();

      /// \brief Load the reference geometry visual.
      public: void Load();

      /// \brief Create an axis and attach it to the reference geometry visual.
      /// \returns Newly created axis visual.
      public: VisualPtr CreateAxis();

      /// \brief Set type of reference geometry.
      /// \param[in] _type Reference geometry type.
      void SetReferenceGeometryType(ReferenceGeometryType _type);

      /// \brief Get type of reference geometry.
      /// \return Reference geometry type.
      ReferenceGeometryType GetReferenceGeometryType() const;
    };
    /// \}
  }
}
#endif
