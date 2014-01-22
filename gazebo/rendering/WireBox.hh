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
#ifndef _WIREBOX_HH_
#define _WIREBOX_HH_

#include <string>

#include "gazebo/math/Box.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/DynamicLines.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class WireBox WireBox.hh rendering/rendering.hh
    /// \brief Draws a wireframe box.
    class WireBox
    {
      /// \brief Constructor
      /// \param[in] _box Dimenision of the box to draw.
      public: explicit WireBox(VisualPtr _parent, const math::Box &_box);

      /// \brief Destructor.
      public: ~WireBox();

      /// \brief Builds the wireframe line list.
      /// \param[in] _box Box to build a wireframe from.
      public: void Init(const math::Box &_box);

      /// \brief Set the visibility of the box.
      /// \param[in] _visible True to make the box visible, False to hide.
      public: void SetVisible(bool _visible);

      /// \brief The lines which outline the box.
      private: DynamicLines *lines;

      /// \brief The visual which this box is attached to.
      private: VisualPtr parent;
    };
    /// \}
  }
}
#endif
