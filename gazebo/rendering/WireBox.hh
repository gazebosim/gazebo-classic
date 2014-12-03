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

#include "gazebo/math/Box.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    class WireBoxPrivate;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class WireBox WireBox.hh rendering/rendering.hh
    /// \brief Draws a wireframe box.
    class GZ_RENDERING_VISIBLE WireBox
    {
      /// \brief Constructor
      /// \param[in] _box Dimension of the box to draw.
      /// \param[in] _parent Parent visual of the box.
      public: explicit WireBox(VisualPtr _parent, const math::Box &_box);

      /// \brief Destructor.
      public: ~WireBox();

      /// \brief Builds the wireframe line list.
      /// \param[in] _box Box to build a wireframe from.
      public: void Init(const math::Box &_box);

      /// \brief Set the visibility of the box.
      /// \param[in] _visible True to make the box visible, False to hide.
      public: void SetVisible(bool _visible);

      /// \brief Get the visibility of the box.
      /// \return True if the box is visual.
      public: bool GetVisible() const;

      /// \brief Get the wireframe box.
      /// \return The wireframe box.
      public: math::Box GetBox() const;

      /// \internal
      /// \brief Pointer to private data.
      private: WireBoxPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
